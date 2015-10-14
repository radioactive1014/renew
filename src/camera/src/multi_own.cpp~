#include <string>
#include <cv.h>
#include <highgui.h>


#include <sstream>
#include <string>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include "object.h"

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;


class colored 

{


	public:
	 int iLowH  ;
	int iHighH;

	int iLowS; 
	int iHighS;

	int iLowV;
	int iHighV;
	const char* color;
	const char* window;
	int posX;
	int posY;
	
  	int iLastX ; 
 	int iLastY ; 
	int lastx;
	int lasty;
	cv::Mat imgThresholded;
	

};

colored red,green,blue;

	
/*
void trackbar( colored item,  int lH, int hH, int lS, int hS, int lV, int hV)

{
	cout<<red.color <<endl;


	
    	item.iLowH = lH;
	item.iHighH = hH;

	item.iLowS = lS; 
	item.iHighS = hS;

	item.iLowV = lV;
	item.iHighV = hV;
	cv::namedWindow(item.color, CV_WINDOW_AUTOSIZE); //create a window called "Control"

	createTrackbar("LowH", item.color, &item.iLowH, 255); //Hue (0 - 179)
	createTrackbar("HighH", item.color, &item.iHighH, 255);

	createTrackbar("LowS", item.color, &item.iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", item.color, &item.iHighS, 255);

	createTrackbar("LowV", item.color, &item.iLowV, 255); //Value (0 - 255)
	createTrackbar("HighV", item.color, &item.iHighV, 255);



}
*/

vector<int>  threshold(colored item, Mat imgHSV, Mat imgLines )

{
	
	
	//cv::Mat imgHSV;
	//cout<<red.iLowH <<endl;
	item.iLastX = -1; 
 	item.iLastY = -1;
	
	vector<int>  result(2);

	cv::inRange(imgHSV, cv::Scalar(item.iLowH, item.iLowS, item.iLowV), cv::Scalar(item.iHighH, item.iHighS, item.iHighV), item.imgThresholded); //Threshold the image


	cv::erode(item.imgThresholded, item.imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
	cv::dilate( item.imgThresholded, item.imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );



	//morphological closing (fill small holes in the foreground)
	cv::dilate( item.imgThresholded, item.imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
	cv::erode(item.imgThresholded, item.imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

	cv::Moments oMoments = cv::moments(item.imgThresholded);

	double dM01 = oMoments.m01;
	double dM10 = oMoments.m10;
	double dArea = oMoments.m00;

	if (dArea > 10000)
	{
	//calculate the position of the ball
	item.posX = dM10 / dArea;
	item.posY = dM01 / dArea;        
	// printf("x= %d,y=%d\n", item.posX,item.posY);  
	if (item.iLastX >= 0 && item.iLastY >= 0 && item.posX >= 0 && item.posY >= 0)
	{
	//Draw a red line from the previous point to the current point
	cv::line(imgLines, cv::Point(item.posX, item.posY), cv::Point(item.iLastX, item.iLastY), cv::Scalar(0,0,255), 2);
	}
	result[0]= item.posX ;
	result[1]= item.posY;
	
	

	item.iLastX =item.posX;
	item.iLastY = item.posY;
	}

	
	cv::imshow(item.window, item.imgThresholded); //show the thresholded image
cv::imshow(item.window, item.imgThresholded); //show the thresholded image
	cv::waitKey(3);

	//int result[2]= {item.posX,item.posY} ;
	
	return result; 	

}



int callback(const sensor_msgs::ImageConstPtr& original_image)  
{	

	cv::Mat imgHSV;
	cv_bridge::CvImagePtr cv_ptr;
	
	try
	{
	//Always copy, returning a mutable CvImage
	//OpenCV expects color images to use BGR channel order.
	cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	//if there is an error during conversion, display it
	ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
	return 0;
	}


	//trackbar(red, 114,179,74,255,45,255);
	//trackbar(green, 114,179,74,255,45,255);


	cv::Mat imgLines = cv::Mat::zeros( cv_ptr->image.size(), CV_8UC3 );
	cv::cvtColor(cv_ptr->image, imgHSV, CV_BGR2HSV);
	
	//threshold(green, imgHSV,imgLines);
	vector<int>  result_red(2), result_green(2), result_blue(2);
	result_red= threshold(red, imgHSV,imgLines);

	result_green=threshold(green, imgHSV,imgLines);
		result_blue=threshold(blue, imgHSV,imgLines);
	//printf("result %d , %d , %d, %d \n", result[0], result[1], result[2], result[3] );
	
	
	circle(cv_ptr->image, Point(result_red[0],result_red[1]), 10,Scalar( 0, 0, 255 ),1,8 );
	circle(cv_ptr->image, Point(result_blue[0],result_blue[1]), 10,Scalar( 255, 0, 0 ),1,8 );
	circle(cv_ptr->image, Point(result_green[0],result_green[1]), 10,Scalar( 0, 255, 0 ),1,8 );


	cv::imshow("Original",cv_ptr->image ); //show the original image
	cv::waitKey(3);
	return 0;

}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "image_processor");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub;

   	image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, callback);

	pub = it.advertise("camera/image_processed", 1);
	red.color = "red";
	green.color = "green";
	blue.color = "blue";

	red.window = "red_window";
	green.window = "green_window";
	blue.window = "blue_window";

	red.iLowH= 142;
	red.iHighH = 179 ;
	red.iLowS = 142;
	red.iHighS= 224;
	red.iLowV = 58;
	red.iHighV = 255;

	blue.iLowH= 61;
	blue.iHighH = 130 ;
	blue.iLowS = 86;
	blue.iHighS= 246;
	blue.iLowV = 57 ;
	blue.iHighV = 255;

	green.iLowH= 32;
	green.iHighH = 60 ;
	green.iLowS = 11;
	green.iHighS= 255;
	green.iLowV = 69 ;
	green.iHighV = 140;
	
	
	
	

	cv::namedWindow(red.color, CV_WINDOW_AUTOSIZE); //create a window called "Control"

	createTrackbar("LowH", red.color, &red.iLowH, 255); //Hue (0 - 179)
	createTrackbar("HighH", red.color, &red.iHighH, 255);

	createTrackbar("LowS", red.color, &red.iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", red.color, &red.iHighS, 255);

	createTrackbar("LowV", red.color, &red.iLowV, 255); //Value (0 - 255)
	createTrackbar("HighV", red.color, &red.iHighV, 255);

	cv::namedWindow(green.color, CV_WINDOW_AUTOSIZE); //create a window called "Control"

	createTrackbar("LowH", green.color, &green.iLowH, 255); //Hue (0 - 179)
	createTrackbar("HighH", green.color, &green.iHighH, 255);

	createTrackbar("LowS", green.color, &green.iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", green.color, &green.iHighS, 255);

	createTrackbar("LowV", green.color, &green.iLowV, 255); //Value (0 - 255)
	createTrackbar("HighV", green.color, &green.iHighV, 255);


	cv::namedWindow(blue.color, CV_WINDOW_AUTOSIZE); //create a window called "Control"

	createTrackbar("LowH", blue.color, &blue.iLowH, 255); //Hue (0 - 179)
	createTrackbar("HighH", blue.color, &blue.iHighH, 255);

	createTrackbar("LowS", blue.color, &blue.iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", blue.color, &blue.iHighS, 255);

	createTrackbar("LowV", blue.color, &blue.iLowV, 255); //Value (0 - 255)
	createTrackbar("HighV", blue.color, &blue.iHighV, 255);



	//trackbar("red", 114,179,74,255,45,255);
	ros::spin();

}














