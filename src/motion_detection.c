/***************************Include Files*************************************/
//OpenCV dependencies
#include "cv.h"
#include "highgui.h"
//Standart dependencies // review ones necessary
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

//RTAI dependencies
#include "rtai_msg.h"
#include "rtai_sem.h"
#include "rtai_shm.h"
/*****************************************************************************/

/********************************Defines**************************************/
// Debug
#define DEBUG
// RTAI
#define PERIOD_NS 1000000 // Control loop period in nanoseconds -> 1.00 KHz
#define PERIOD_S  (PERIOD_NS/1.0e9) // Control loop period in seconds        
#define DELTA_T   100000 // Max. allowed timing error in nanoseconds -> 0.1 ms

// OpenCV
#define CAM_NUM 0         // Number of device used
#define _THRESVAL 10      // Threshold value
#define _SIGMA 2          // Sigma value
/*****************************************************************************/

/*********************************Global**************************************/
//Task and threads
RT_TASK *maintask, // RTAI main task
        *task,     // handler for tasks
        *t1,       // T1 task structure
        *t2,       // T2 task structure
        *t3;       // T3 task structure
pthread_t *task1,  // Capture task
          *task2,  // Process task
          *task3;  // Display task

// Task period and interprocess communications timeout
RTIME period, timeout;

// Task status array, 0: normal operation, other value: error
unsigned char blocked[3] = {0};

// Barrier semaphore used for tasks synchronization
SEM *cap_sem, // Capture data semaphore
    *sub_sem; // Processed data semaphore

// Shared matrix for images
IplImage *image=0;   // Capture image matrix
IplImage *img_bin=0; // Processed image to display

/*****************************************************************************/

/**************************Function definition********************************/
void cleanup(void);
void *capture_data(void);
void *img_subs(void);
void *display_data(void);
/*****************************************************************************/

/*******************************Main Function*********************************/
int main( int argc, char** argv ) {

   rt_allow_nonroot_hrt();
   // RTAI => Initialize main task : supervision
   if(!( maintask = rt_task_init_schmod(nam2num("MAINTK"), 1, 0, 0, SCHED_FIFO, 0xFF))) {
      printf("Cannot init MAINTK task\n");
      exit(1);
    }

   // Memory locking => avoid process to being swapped out
   mlockall( MCL_CURRENT | MCL_FUTURE );

   rt_make_soft_real_time();
#ifndef DEBUG
   rt_make_hard_real_time(); // If not in debug make it har real time
#endif

   // Start the RT timer
   rt_set_periodic_mode();               // set periodic mode to function
   period = nano2count(PERIOD_NS);       /* PERIOD_NS */
   //timeout = nano2count( 10*PERIOD_NS );
   timeout = nano2count( 100*PERIOD_NS );
   start_rt_timer(period);               /* start the timer with period 
                                            defined, this interrup each
                                            period */

   // Prepare the binary semaphores for the two shared matrix
   cap_sem = rt_typed_sem_init(nam2num("CAPSEM"), 1, BIN_SEM | FIFO_Q );
   sub_sem = rt_typed_sem_init(nam2num("SUBSEM"), 1, BIN_SEM | FIFO_Q );

   // Create thread for task1
   printf("\nInit TSK1TH thread: Capture\n");
   if(!(task1 = (pthread_t*) rt_thread_create(capture_data, NULL, 0))) {
      printf("Error in creating TSK1TH thread\n");
      cleanup();
      exit(1);
   }
   // Wait until task is created
   while( !rt_get_adr(nam2num("TSK1TH"))) { rt_sleep(nano2count(10e+6)); }

   // Create thread for task2
   printf("\nInit TSK2TH thread: Process\n");
   if(!(task2 = (pthread_t*) rt_thread_create(img_subs, NULL, 0 ))) {
      printf("Error in creating TSK2TH thread\n");
      cleanup();
      exit(1);
   }
   // Wait until task is created
   while( !rt_get_adr(nam2num("TSK2TH"))) { rt_sleep(nano2count(10e+6)); }

   // Create thread for task3
   printf("\nInit TSK3TH thread: Display\n");
   if(!(task3 = (pthread_t*) rt_thread_create(display_data, NULL, 0 ))) {
      printf("Error in creating TSK3TH thread\n");
      cleanup();
      exit(1);
   }
   // Wait until task is created
   while( !rt_get_adr(nam2num("TSK3TH"))) { rt_sleep(nano2count(10e+6)); }

   // Supervision task initialization
   rt_task_make_periodic(maintask, rt_get_time(), period);
   //FIXME// Wait for barrier until the other tasks finish initializing
   //rt_sem_wait_barrier(barrier);

   // Counters for software watchdog
   unsigned long internal_count = 0, count, count_prev[3] = {0xFFFFFFFE};

   // Infinite program loop
   while(1) {
      // Task 1 supervision
      if((task = rt_get_adr(nam2num("TSK1TH")))) {
         // Send a message and obtained a response before a predefined timeout
         task = rt_rpc_timed( task, internal_count, &count, timeout );
         if( task == 0 ) {
            blocked[0] = 1;
            printf("TSK1TH message timeout\n");
             break;
         } else if( count == count_prev[0]) {
            blocked[0] = 2;
            printf("TSK1TH is blocked\n");
            break;
         }  else if( count == 0xFFFFFFFF ) {
            blocked[0] = 3;
            printf("TSK1TH reported an error\n");
            break;
         }
         count_prev[0] = count;
      } else {
         printf("TSK1TH is invalid\n");
         break;
      }

      // Task 2 supervision
      if((task = rt_get_adr(nam2num("TSK2TH")))) {
         // Send a message and obtained a response before a predefined timeout
         task = rt_rpc_timed( task, internal_count, &count, timeout );
         if( task == 0 ) {
            blocked[1] = 1;
            printf("TSK2TH message timeout\n");
            break;
         } else if( count == count_prev[1] ) {
            blocked[1] = 2;
            printf("TSK2TH is blocked\n");
            break;
         } else if( count == 0xFFFFFFFF ) {
            blocked[1] = 3;
            printf("TSK2TH reported an error\n");
            break;
         }
         count_prev[1] = count;
      } else {
         printf("TSK2TH is invalid\n");
         break;
      }

      if((task = rt_get_adr(nam2num("TSK3TH")))) {
         // Send a message and obtained a response before a predefined timeout
         task = rt_rpc_timed( task, internal_count, &count, timeout );
         if( task == 0 ) {
            blocked[2] = 1;
            printf("TSK3TH message timeout\n");
            break;
         } else if( count == count_prev[2] ) {
            blocked[2] = 2;
            printf("TSK3TH is blocked\n");
            //break;
         } else if( count == 0xFFFFFFFF ) {
            blocked[2] = 3;
            printf("TSK3TH reported an error\n");
            break;
         }
         count_prev[2] = count;
      } else {
         printf("TSK3TH is invalid\n");
         break;
      }

      // Increment internal count for software watchdog
      internal_count++;
      if(internal_count == 0xFFFFFFFF) { internal_count = 0;}

      // Wait to fill a predefined time slice of PERIOD_NS defined in parameters.h
      rt_task_wait_period();

      // Check for termination
      char c = (char)cvWaitKey(50);
      if( c == 27 ) { //Stop if Esc is pressed
         break;
      }

   }

   // RTAI cleanup
   cleanup();

   return 0;
 }

/*******************************Functions*************************************/
// Cleanup
void cleanup() {
   // LOcal variables
   RT_TASK *task_h;   // task handler
   unsigned long msg; // Variable used for task RPC messaging

   // Finish TSK1TH thread
   if((task_h = rt_get_adr(nam2num("TSK1TH"))) && !blocked[0]) {
      // This code send a msg to the thread to terminate, do it diff
      msg = 0xFFFFFFFF; // Message indicating the thread to terminate
      rt_rpc_timed( task_h, msg, &msg, timeout );
      printf("\nWaiting TSK1TH to finish...\n");
      // Wait for RT task to finish
      while(rt_get_adr(nam2num("TSK1TH"))) { rt_sleep(nano2count(10e+6)); }
      rt_thread_join( (int) task1 );
      printf("TSK1TH terminated\n");
   }

   // Finish TSK2TH thread
   if((task_h = rt_get_adr(nam2num("TSK2TH"))) && !blocked[1]) {
      msg = 0xFFFFFFFF; // Message indicating the thread to terminate
      rt_rpc_timed( task_h, msg, &msg, timeout );
      printf("\nWaiting TSK2TH to finish...\n");
      // Wait for RT task to finish
      while(rt_get_adr(nam2num("TSK2TH"))) { rt_sleep(nano2count(10e+6)); }
         rt_thread_join( (int) task2 );
         printf("TSK2TH terminated\n");
   }

   // Finish TSK3TH thread
   if((task_h = rt_get_adr(nam2num("TSK3TH"))) && !blocked[2]) {
      msg = 0xFFFFFFFF; // Message indicating the thread to terminate
      rt_rpc_timed( task_h, msg, &msg, timeout );
      printf("\nWaiting TSK3TH to finish...\n");
      // Wait for RT task to finish
      while(rt_get_adr(nam2num("TSK3TH"))) { rt_sleep(nano2count(10e+6)); }
      rt_thread_join( (int) task3 );
      printf("TSK3TH terminated\n");
   }

   // Stop the RT
   rt_make_soft_real_time();
   stop_rt_timer();
   // Free the memory
   // Global OpenCV
   cvReleaseImage(&image);
   cvReleaseImage(&img_bin);
   // RTAI
   rt_sem_delete(cap_sem);
   rt_sem_delete(sub_sem);
   rt_task_delete(maintask);
   printf("Delete maintask... done!\n");
   printf( "\nEnd MAIN task %p\n", maintask );

}

// Capture data from the camera
void *capture_data() {
   // Local variables
   long long int overruns = 0,
                 measured_period_ns = 0, measured_period_ns_max = 0,
                 activation_start_time_ns = 0, last_activation_start_time_ns = 0;
   // Variable for receiving instructions from main routine, e.g. termination
   unsigned long internal_count = 0, count, count_prev = 0xFFFFFFFE;
   unsigned char error = 0;
   // Task handler
   RT_TASK *task_h;
   // Video capture to store recolected data
   CvCapture *cap = 0;
   // Frame to store the capture data
   IplImage *frame = 0;

   // Task initialization
   if(!(t1 = rt_task_init_schmod( nam2num("TSK1TH"), 1, 0, 0, SCHED_FIFO, 0xFF))){
      printf( "Cannot init TSK1TH task" );
      return 0;
   }
   // Memory Locking
   mlockall( MCL_CURRENT | MCL_FUTURE );
   // Set RT mode
   rt_make_soft_real_time();
#ifndef DEBUG
   rt_make_hard_real_time(); // If not in debug make it hard real time
#endif
   // Set the task period
   rt_task_make_periodic( t1, rt_get_time(), period );
   printf("Init TSK1TH loop\n");
   last_activation_start_time_ns = rt_get_time_ns();
   printf( "\nInit TSK1TH task %p\n", t1 );

   // Review if the camera can be opened
   cap = cvCaptureFromCAM(CAM_NUM);
   if(!(cap)) { // camera couldn't be opened
      printf( "***Could not initialize capturing...***\n");
      exit(1);
   }

   // Infinite loop
   for(;;) {
      // Opencv capture
      frame = cvQueryFrame(cap); // Set the capture on the matrix frame
      if(!frame){ // If no capture terminate infinite loop
         break; // capture couldn't be execute
         printf("Capture couldn't be execute\n");
      }

      // Share resource image
      rt_sem_wait_barrier(cap_sem); // Wait for the semaphore
      cvReleaseImage(&image);
      image = cvCloneImage(frame);
      rt_sem_signal(cap_sem);       // give up semaphore

      // RT loop stats
      activation_start_time_ns = rt_get_time_ns();
      measured_period_ns = activation_start_time_ns - last_activation_start_time_ns;
      last_activation_start_time_ns = activation_start_time_ns;

      if (measured_period_ns > (PERIOD_NS + DELTA_T)) {
         overruns++;
         if(measured_period_ns > measured_period_ns_max) measured_period_ns_max = measured_period_ns;
      } // Debug variable to count number of loops for which the time constraint is not met

      // Check for termination
      if ((task_h = rt_receive_if( 0, &count ))) {
         if( count == count_prev ) {
            rt_return( task_h, 0xFFFFFFFF );
            printf("TSK1TH Supervision task blocked\n");
            break;
         } else if ( count == 0xFFFFFFFF ) {
            rt_return( task_h, internal_count );
            printf( "TSK1TH received termination message\n" );
            break;
         } else if( error ) {
            rt_return( task_h, 0xFFFFFFFF );
            printf("TSK1TH error detected: %d\n", error);
            break;
         } else {
            rt_return( task_h, internal_count );
         }
         count_prev = count;
      }
      internal_count++;
      if( internal_count == 0xFFFFFFFF ) { internal_count = 0; }

      // Wait until filling time slice
      rt_task_wait_period();

   }
   // Release the capture device and image
   cvReleaseImage(&frame);
   cvReleaseCapture(&cap);
   // Finish task
   rt_make_soft_real_time();
   rt_task_delete(t1);
   printf("\nEnd TSK1TH task %p\n", t1);
   printf("Overruns TSK1TH = %llu\n", overruns );
   printf("Worst case in ns TSK1TH: %llu\n", measured_period_ns_max );

   return 0;
}

// Process data. Perform image substraction
void *img_subs() {
   // Local variables
   long long int overruns = 0,
                 measured_period_ns = 0, measured_period_ns_max = 0,
                 activation_start_time_ns = 0, last_activation_start_time_ns = 0;
   // Variable for receiving instructions from main routine, e.g. termination
   unsigned long internal_count = 0, count, count_prev = 0xFFFFFFFE;
   unsigned char error = 0;
   // Task handler
   RT_TASK *task_h;
   // First frame flag
   bool first_frame = true;   // FIXME review if may be optimize to remove this
   // Subtraction variables
   const char *win_diff = "Diff on Thread 2"; // Result window name
   int thresval = _THRESVAL;  // Threshold value
   float sigma =  _SIGMA;     // Sigma value
   IplImage *frame = 0;       // Copy of the capture image
   IplImage *img_prev=0;      // Matrix for previous image
   IplImage *img_gray=0;      // Matrix for image in gray scale
   IplImage *img_diff=0;      // Matrix for the image diference
   IplImage *img_bin_local=0; // Local image bin to proces and then pass it

   cvNamedWindow(win_diff, CV_WINDOW_AUTOSIZE); // Create a window to show the diff with first image
   // Task initialization
   if(!(t2 = rt_task_init_schmod( nam2num("TSK2TH"), 1, 0, 0, SCHED_FIFO, 0xFF))){
      printf( "Cannot init TSK2TH task" );
      return 0;
   }
   // Memory Locking
   mlockall( MCL_CURRENT | MCL_FUTURE );
   // Set RT mode
   rt_make_soft_real_time();
#ifndef DEBUG
   rt_make_hard_real_time(); // If not in debug make it hard real time
#endif
   // Set the task period
   rt_task_make_periodic( t2, rt_get_time(), period );
   printf("Init TSK2TH loop\n");
   last_activation_start_time_ns = rt_get_time_ns();
   printf( "\nInit TSK2TH task %p\n", t2 );

   // Infinite loop
   for(;;) {
      // Share resource image. Read value
      cvReleaseImage(&frame);
      rt_sem_wait_barrier(cap_sem); // Wait for the semaphore
      if (image != 0) {
         frame = cvCloneImage(image);
      }
      rt_sem_signal(cap_sem);       // give up semaphore

      if (frame != 0) {
      // Flip frame
      cvFlip(frame,frame,1);
      // Set in gray scale
      cvReleaseImage(&img_gray);
      img_gray = cvCreateImage(cvGetSize(frame), 8, 1);
      cvCvtColor(frame, img_gray, CV_BGR2GRAY);
      // Reduce noise and detail by blurring the image
//      cvSmooth(img_gray, img_gray, CV_GAUSSIAN, 0, 0, sigma, 0);

      if (first_frame) { // FIXME review if may be optimize to remove this
         cvReleaseImage(&img_prev);
         img_prev = cvCloneImage(img_gray);
         first_frame = false;
         continue;
      }

      // Substraction and filter process
      cvReleaseImage(&img_diff); 
      img_diff = cvCreateImage(cvGetSize(img_gray), 8, 1);
      cvAbsDiff(img_gray, img_prev, img_diff);
      cvReleaseImage(&img_bin_local);
      img_bin_local = cvCreateImage(cvGetSize(img_diff), 8, 1);
      cvThreshold(img_diff, img_bin_local, thresval, 255, CV_THRESH_BINARY);
      cvErode(img_bin_local, img_bin_local, NULL, 3);
      cvDilate(img_bin_local, img_bin_local, NULL, 1);

      // Copy image to global resource to pass it to display
      rt_sem_wait_barrier(sub_sem); // Wait for the semaphore
      cvReleaseImage(&img_bin);
      img_bin = cvCloneImage(img_bin_local);
      rt_sem_signal(sub_sem);       // give up semaphore

      cvShowImage(win_diff, img_bin_local ); // Show image
      // Store image in previous matrix
      cvReleaseImage(&img_prev);
      img_prev= cvCloneImage(img_gray);
      }

      // RT loop stats
      activation_start_time_ns = rt_get_time_ns();
      measured_period_ns = activation_start_time_ns - last_activation_start_time_ns;
      last_activation_start_time_ns = activation_start_time_ns;

      if (measured_period_ns > (PERIOD_NS + DELTA_T)) {
         overruns++;
         if(measured_period_ns > measured_period_ns_max) measured_period_ns_max = measured_period_ns;
      } // Debug variable to count number of loops for which the time constraint is not met

      // Check for termination
      if ((task_h = rt_receive_if( 0, &count ))) {
         if( count == count_prev ) {
            rt_return( task_h, 0xFFFFFFFF );
            printf("TSK2TH Supervision task blocked\n");
            break;
         } else if ( count == 0xFFFFFFFF ) {
            rt_return( task_h, internal_count );
            printf( "TSK2TH received termination message\n" );
            break;
         } else if( error ) {
            rt_return( task_h, 0xFFFFFFFF );
            printf("TSK2TH error detected: %d\n", error);
            break;
         } else {
            rt_return( task_h, internal_count );
         }
         count_prev = count;
      }
      internal_count++;
      if( internal_count == 0xFFFFFFFF ) { internal_count = 0; }

      // Wait until filling time slice
      rt_task_wait_period();
   }

   // Finish task
   // Free mem
   cvReleaseImage(&frame);
   cvReleaseImage(&img_prev);
   cvReleaseImage(&img_gray);
   cvReleaseImage(&img_diff); 
   cvReleaseImage(&img_bin_local);
   // Free RTAI
   rt_make_soft_real_time();
   rt_task_delete(t2);
   printf("\nEnd TSK2TH task %p\n", t2);
   printf("Overruns TSK2TH = %llu\n", overruns );
   printf("Worst case in ns TSK2TH: %llu\n", measured_period_ns_max );

   return 0;
}

// Display data
void *display_data() {
   // Local variables RTAI
   long long int overruns = 0,
                 measured_period_ns = 0, measured_period_ns_max = 0,
                 activation_start_time_ns = 0, last_activation_start_time_ns = 0;
   // Variable for receiving instructions from main routine, e.g. termination
   unsigned long internal_count = 0, count, count_prev = 0xFFFFFFFE;
   unsigned char error = 0;
   // Task handler
   RT_TASK *task_h;
   // Local variables to store data
   IplImage *local_image=0;
   IplImage *local_img_bin=0;
   // Name for the windows
   const char *win_cap  = "Capture";               // Capture window name
   const char *win_diff1 = "Diff from prev frame"; // Result window name

   // Create the container windows
   cvNamedWindow(win_cap, CV_WINDOW_AUTOSIZE ); // Create window to show the capture
   cvNamedWindow(win_diff1, CV_WINDOW_AUTOSIZE); // Create a window to show the diff with first image
   // Task initialization
   if(!(t3 = rt_task_init_schmod( nam2num("TSK3TH"), 1, 0, 0, SCHED_FIFO, 0xFF))){
      printf( "Cannot init TSK3TH task" );
      return 0;
   }
   // Memory Locking
   mlockall( MCL_CURRENT | MCL_FUTURE );
   // Set RT mode
   rt_make_soft_real_time();
#ifndef DEBUG
   rt_make_hard_real_time(); // If not in debug make it har real time
#endif
   // Set the task period
   rt_task_make_periodic( t3, rt_get_time(), period );
   printf("Init TSK3TH loop\n");
   last_activation_start_time_ns = rt_get_time_ns();
   printf( "\nInit TSK3TH task %p\n", t3 );

   // Infinite loop
   for(;;) {
      // Copy images form global resources
      // Release previous local allocated mem
      cvReleaseImage(&local_img_bin);
      cvReleaseImage(&local_image);
      // Copy image
      rt_sem_wait_barrier(cap_sem); // Wait for the semaphore
      if (image != 0) {
         local_image = cvCloneImage(image);
      }
      rt_sem_signal(cap_sem);       // give up semaphore
      // Copy processed
      rt_sem_wait_barrier(sub_sem); // Wait for the semaphore
      if (img_bin != 0) {
         local_img_bin = cvCloneImage(img_bin);
      }
      rt_sem_signal(sub_sem);       // give up semaphore

      // Display images
      if ((local_image != 0) && (local_img_bin != 0)) {
      cvShowImage(win_cap, local_image ); // Show image
      cvShowImage(win_diff1, local_img_bin);
      }

      // RT loop stats
      activation_start_time_ns = rt_get_time_ns();
      measured_period_ns = activation_start_time_ns - last_activation_start_time_ns;
      last_activation_start_time_ns = activation_start_time_ns;

      if (measured_period_ns > (PERIOD_NS + DELTA_T)) {
         overruns++;
         if(measured_period_ns > measured_period_ns_max) measured_period_ns_max = measured_period_ns;
      } // Debug variable to count number of loops for which the time constraint is not met

      // Check for termination
      if ((task_h = rt_receive_if( 0, &count ))) {
         if( count == count_prev ) {
            rt_return( task_h, 0xFFFFFFFF );
            printf("TSK3TH Supervision task blocked\n");
            break;
         } else if ( count == 0xFFFFFFFF ) {
            rt_return( task_h, internal_count );
            printf( "TSK3TH received termination message\n" );
            break;
         } else if( error ) {
            rt_return( task_h, 0xFFFFFFFF );
            printf("TSK3TH error detected: %d\n", error);
            break;
         } else {
            rt_return( task_h, internal_count );
         }
         count_prev = count;
      }
      internal_count++;
      if( internal_count == 0xFFFFFFFF ) { internal_count = 0; }

      // Wait until filling time slice
      rt_task_wait_period();
   }
   // Free local img
   cvReleaseImage(&local_img_bin);
   cvReleaseImage(&local_image);
   // destroy display windows
   cvDestroyWindow(win_cap);
   cvDestroyWindow(win_diff1);
   // Finish task
   rt_make_soft_real_time();
   rt_task_delete(t3);
   printf("\nEnd TSK3TH task %p\n", t3);
   printf("Overruns TSK3TH = %llu\n", overruns );
   printf("Worst case in ns TSK3TH: %llu\n", measured_period_ns_max );

   return 0;
}
/*****************************************************************************/


