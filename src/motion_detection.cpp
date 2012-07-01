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
/*******************************Main Function*********************************/
int main( int argc, char** argv ) {

   VideoCapture cap;
   Mat image; /* 2D or multi-dimensional dense array (can be used to store 
              matrices, images, histograms, feature descriptors, voxel 
              volumes etc.)*/

   /* Try to open the camera */
   cap.open(CAM_NUM);

   if(!cap.isOpened()) { // camera couldn't be opened
      cout << "***Could not initialize capturing...***\n";
      return -1;
   }

   namedWindow( "Detector", CV_WINDOW_AUTOSIZE ); // Create Detector window

   for(;;) { /* Infinite loop */
      Mat frame;
      cap >> frame; // Set the capture on the matrix frame
      if(frame.empty()){ // If no capture terminate infinite loop
         break;
      }
      frame.copyTo(image);
      imshow( "Detector", image ); // Show image
      char c = (char)waitKey(10);
      if( c == 27 ) { //Stop if Esc is pressed
         break;
      }
      if (c == 'c') { // capture image and save
         cout << "Saving image\n";
         imwrite(FILENAME, image);
      }
   }

   // Release the capture device housekeeping
   destroyWindow( "Detector" );
   cap.release();
   return 0;
 }

