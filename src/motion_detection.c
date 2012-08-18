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
   const char *win_diff1 = "Diff from first frame";
   const char *win_diff2 = "Diff from prev frame"; 

   CvCapture *cap = 0;
   IplImage *image = 0; /* 2D or multi-dimensional dense array (can be used to store 
              matrices, images, histograms, feature descriptors, voxel 
              volumes etc.)*/

   IplImage *img_gray=0, *img_first=0, *img_prev=0, *img_diff=0, *img_bin=0;

   cvNamedWindow(win_cap, CV_WINDOW_AUTOSIZE ); // Create window to show the capture
   cvNamedWindow(win_diff1, CV_WINDOW_AUTOSIZE); // Create a window to show the diff with first image
   cvNamedWindow(win_diff2, CV_WINDOW_AUTOSIZE); // Create a window to show diff with last frame

   /* Try to open the camera */
   //cap.open(CAM_NUM);

   if(!(cvCaptureFromCAM(CAM_NUM))) { // camera couldn't be opened
      printf( "***Could not initialize capturing...***\n");
      return -1;
   }


   for(;;) { /* Infinite loop */
      IplImage *frame = 0;
      //if(!cvGrabFrame(cap)){ // If no capture terminate infinite loop
      //   printf("Couldn't capture frame");
      //   break;
      //}
      frame = cvGrabFrame( cap ); // Set the capture on the matrix frame
      if (frame) {
      cvCopy(frame, image, 0);
      //frame.copyTo(image);
      cvShowImage(win_cap, image ); // Show image

      cvFlip(frame,frame,1);
      cvCvtColor(frame, img_gray, CV_BGR2GRAY);
      //Gaussian blur can be used in order to obtain a smooth grayscale digital image of a halftone print
      cvSmooth(img_gray, img_gray, CV_GAUSSIAN, 0, 0, sigma, 0);
      //cvGaussianBlur(img_gray, img_gray,cvSize(0,0), sigma, sigma); // Reduce noise and detail by blurring the image

      if (first_frame) {
         img_prev = cvCloneImage(img_gray);
         img_first = cvCloneImage(img_gray);
         first_frame = false;
         continue;
      }

      cvAbsDiff(img_gray, img_prev, img_diff);
      cvThreshold(img_diff, img_bin, thresval, 255, CV_THRESH_BINARY);
      //cvErode(img_bin, img_bin, cvMat(), Point(-1,-1), 3);
      //cvDilate(img_bin, img_bin, cvMat(), Point(-1,-1), 1);
      cvShowImage(win_diff1, img_bin);

      //absdiff(img_gray, img_first, img_diff);
      //threshold(img_diff, img_bin, thresval, 255, CV_THRESH_BINARY);
      //erode(img_bin, img_bin, Mat(), Point(-1,-1), 3);
      //dilate(img_bin, img_bin, Mat(), Point(-1,-1), 1);
      //imshow(win_diff2, img_bin);

      img_prev= cvCloneImage(img_gray);

      }
      char c = (char)cvWaitKey(50);
      if( c == 27 ) { //Stop if Esc is pressed
         break;
      }

      if (c == 'c') { // capture image and save
         printf( "Saving image\n");
       //  imwrite(FILENAME, image);
         img_first = cvCloneImage(img_gray);
      }
   }

   // Release the capture device housekeeping
   printf("Closing App");
   cvDestroyWindow(win_cap);
   cvDestroyWindow(win_diff1);
   cvDestroyWindow(win_diff2);
   cvReleaseCapture(&cap);
   return 0;
 }

