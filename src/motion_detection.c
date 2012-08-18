/***************************Include Files*************************************/
//#include <opencv2/video/tracking.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include "cv.h"
#include "highgui.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

//#include <iostream>
//#include <ctype.h>
/*****************************************************************************/

/******************************Namespace**************************************/
//using namespace cv;
//using namespace std;
/*****************************************************************************/

/********************************Defines**************************************/
#define CAM_NUM 0 // NUmber of device used
#define FILENAME "snapshot.jpg" // name for a saved file if wanted
#define _THRESVAL 10
#define _SIGMA 2
/*******************************Main Function*********************************/
int main( int argc, char** argv ) {

   int thresval = _THRESVAL;
   float sigma =  _SIGMA;
   bool first_frame = true;
   const char *win_cap  = "Capture";
   const char *win_diff1 = "Diff from prev frame";

   CvCapture *cap = 0;
   IplImage *image = 0; /* 2D or multi-dimensional dense array (can be used to store 
              matrices, images, histograms, feature descriptors, voxel 
              volumes etc.)*/

   IplImage *img_gray=0, *img_first=0, *img_prev=0, *img_diff=0, *img_bin=0;

   cvNamedWindow(win_cap, CV_WINDOW_AUTOSIZE ); // Create window to show the capture
   cvNamedWindow(win_diff1, CV_WINDOW_AUTOSIZE); // Create a window to show the diff with first image

   /* Try to open the camera */
   //cap.open(CAM_NUM);

   cap = cvCaptureFromCAM(CAM_NUM);
   if(!(cap)) { // camera couldn't be opened
      printf( "***Could not initialize capturing...***\n");
      return -1;
   }


   for(;;) { /* Infinite loop */
      IplImage *frame = 0;
      //if(!cvGrabFrame(cap)){ // If no capture terminate infinite loop
      //   printf("Couldn't capture frame");
      //   break;
      //}
      frame = cvQueryFrame(cap); // Set the capture on the matrix frame
      image = cvCloneImage(frame);
      cvShowImage(win_cap, image ); // Show image

      //img_gray = cvCloneImage(frame);
      cvFlip(frame,frame,1);
      img_gray = cvCreateImage(cvGetSize(frame), 8, 1);
      cvCvtColor(frame, img_gray, CV_BGR2GRAY);
      //Gaussian blur can be used in order to obtain a smooth grayscale digital image of a halftone print
      cvSmooth(img_gray, img_gray, CV_GAUSSIAN, 0, 0, sigma, 0);

      if (first_frame) {
         img_prev = cvCloneImage(img_gray);
         img_first = cvCloneImage(img_gray);
         first_frame = false;
         continue;
      }

      img_diff = cvCreateImage(cvGetSize(img_gray), 8, 1);
      cvAbsDiff(img_gray, img_prev, img_diff);
      img_bin = cvCreateImage(cvGetSize(img_diff), 8, 1);
      cvThreshold(img_diff, img_bin, thresval, 255, CV_THRESH_BINARY);
      cvErode(img_bin, img_bin, cvGetSize(img_bin), Point(-1,-1), 3);
      //cvDilate(img_bin, img_bin, cvMat(), Point(-1,-1), 1);
      cvShowImage(win_diff1, img_bin);

      img_prev= cvCloneImage(img_gray);

      char c = (char)cvWaitKey(50);
      if( c == 27 ) { //Stop if Esc is pressed
         break;
      }

      if (c == 'c') { // capture image and save
         printf( "Saving image\n");
         //img_first = cvCloneImage(img_gray);
      }
   }

   // Release the capture device housekeeping
   printf("Closing App");
   cvDestroyWindow(win_cap);
   cvDestroyWindow(win_diff1);
   cvReleaseCapture(&cap);
   return 0;
 }

