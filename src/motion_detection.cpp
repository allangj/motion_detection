/***************************Include Files*************************************/
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <ctype.h>

#include <time.h>
#include <stdio.h>
/*****************************************************************************/

/******************************Namespace**************************************/
using namespace cv;
using namespace std;
/*****************************************************************************/

/********************************Defines**************************************/
#define CAM_NUM 0 // NUmber of device used
#define FILENAME "snapshot.jpg" // name for a saved file if wanted
#define _THRESVAL 10
#define _SIGMA 2
#define DEBUG_TIME 
/*******************************Main Function*********************************/
int main( int argc, char** argv ) {

   int t;        // time
   int t_offset; // offset time for iteration

   int thresval = _THRESVAL;
   float sigma =  _SIGMA;
   bool first_frame = true;
   const char *win_cap  = "Capture";
#ifndef DEBUG_TIME
   const char *win_diff1 = "Diff from first frame";
#endif
   const char *win_diff2 = "Diff from prev frame"; 

   VideoCapture cap;
   Mat image; /* 2D or multi-dimensional dense array (can be used to store 
              matrices, images, histograms, feature descriptors, voxel 
              volumes etc.)*/

   Mat img_gray, img_prev, img_diff, img_bin;

#ifndef DEBUG_TIME
   Mat img_first;
#endif

   namedWindow(win_cap, CV_WINDOW_AUTOSIZE ); // Create window to show the capture
#ifndef DEBUG_TIME
   namedWindow(win_diff1, CV_WINDOW_AUTOSIZE); // Create a window to show the diff with first image
#endif
   namedWindow(win_diff2, CV_WINDOW_AUTOSIZE); // Create a window to show diff with last frame

   /* Try to open the camera */
   cap.open(CAM_NUM);

   if(!cap.isOpened()) { // camera couldn't be opened
      cout << "***Could not initialize capturing...***\n";
      return -1;
   }


   for(;;) { /* Infinite loop */
      Mat frame;
      cap >> frame; // Set the capture on the matrix frame
      if(frame.empty()){ // If no capture terminate infinite loop
         break;
      }
      frame.copyTo(image);
      imshow(win_cap, image ); // Show image

#ifdef DEBUG_TIME
      // Processing start here. Took time
      t_offset = clock();
      printf ("Offset %d clicks (%f seconds).\n",(t_offset),((float)(t_offset))/CLOCKS_PER_SEC);
#endif

      flip(frame,frame,1);
      cvtColor(frame, img_gray, CV_BGR2GRAY);
      //Gaussian blur can be used in order to obtain a smooth grayscale digital image of a halftone print
      GaussianBlur(img_gray, img_gray, Size(0,0), sigma, sigma); // Reduce noise and detail by blurring the image

      if (first_frame) {
         img_prev = img_gray.clone();
#ifndef DEBUG_TIME
         img_first = img_gray.clone();
#endif
         first_frame = false;
         continue;
      }

      absdiff(img_gray, img_prev, img_diff);
      threshold(img_diff, img_bin, thresval, 255, THRESH_BINARY);
      erode(img_bin, img_bin, Mat(), Point(-1,-1), 3);
      dilate(img_bin, img_bin, Mat(), Point(-1,-1), 1);
      imshow(win_diff2, img_bin);

#ifndef DEBUG_TIME
      absdiff(img_gray, img_first, img_diff);
      threshold(img_diff, img_bin, thresval, 255, THRESH_BINARY);
      erode(img_bin, img_bin, Mat(), Point(-1,-1), 3);
      dilate(img_bin, img_bin, Mat(), Point(-1,-1), 1);
      imshow(win_diff1, img_bin);
#endif

      img_prev=img_gray.clone();
#ifdef DEBUG_TIME
      t = clock();
      printf ("Time %d clicks (%f seconds).\n",(t),((float)(t))/CLOCKS_PER_SEC);
      printf ("It took me %d clicks (%f seconds).\n\n",(t-t_offset),((float)(t-t_offset))/CLOCKS_PER_SEC);
#endif

      char c = (char)waitKey(50);
      if( c == 27 ) { //Stop if Esc is pressed
         break;
      }

      if (c == 'c') { // capture image and save
         cout << "Saving image\n";
         imwrite(FILENAME, image);
#ifndef DEBUG_TIME
         img_first = img_gray.clone();
#endif
      }
   }

   // Release the capture device housekeeping
   destroyWindow(win_cap);
#ifndef DEBUG_TIME
   destroyWindow(win_diff1);
#endif
   destroyWindow(win_diff2);
   cap.release();
   return 0;
 }
