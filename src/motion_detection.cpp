/***************************Include Files*************************************/
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <ctype.h>
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
/*******************************Main Function*********************************/
int main( int argc, char** argv ) {

   int thresval = _THRESVAL;
   float sigma =  _SIGMA;
   bool first_frame = true;
   const char *win_cap  = "Capture";
   const char *win_diff1 = "Diff from first frame";
   const char *win_diff2 = "Diff from prev frame"; 

   VideoCapture cap;
   Mat image; /* 2D or multi-dimensional dense array (can be used to store 
              matrices, images, histograms, feature descriptors, voxel 
              volumes etc.)*/

   Mat img_gray, img_first, img_prev, img_diff, img_bin;

   namedWindow(win_cap, CV_WINDOW_AUTOSIZE ); // Create window to show the capture
   namedWindow(win_diff1, CV_WINDOW_AUTOSIZE); // Create a window to show the diff with first image
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

      flip(frame,frame,1);
      cvtColor(frame, img_gray, CV_BGR2GRAY);
      //Gaussian blur can be used in order to obtain a smooth grayscale digital image of a halftone print
      GaussianBlur(img_gray, img_gray, Size(0,0), sigma, sigma); // Reduce noise and detail by blurring the image

      if (first_frame) {
         img_prev = img_gray.clone();
         img_first = img_gray.clone();
         first_frame = false;
         continue;
      }

      absdiff(img_gray, img_prev, img_diff);
      threshold(img_diff, img_bin, thresval, 255, THRESH_BINARY);
      erode(img_bin, img_bin, Mat(), Point(-1,-1), 3);
      dilate(img_bin, img_bin, Mat(), Point(-1,-1), 1);
      imshow(win_diff1, img_bin);

      absdiff(img_gray, img_first, img_diff);
      threshold(img_diff, img_bin, thresval, 255, THRESH_BINARY);
      erode(img_bin, img_bin, Mat(), Point(-1,-1), 3);
      dilate(img_bin, img_bin, Mat(), Point(-1,-1), 1);
      imshow(win_diff2, img_bin);

      img_prev=img_gray.clone();

      char c = (char)waitKey(50);
      if( c == 27 ) { //Stop if Esc is pressed
         break;
      }

      if (c == 'c') { // capture image and save
         cout << "Saving image\n";
         imwrite(FILENAME, image);
         img_first = img_gray.clone();
      }
   }

   // Release the capture device housekeeping
   destroyWindow(win_cap);
   destroyWindow(win_diff1);
   destroyWindow(win_diff2);
   cap.release();
   return 0;
 }

