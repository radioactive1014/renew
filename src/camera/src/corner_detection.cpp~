#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>


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

namespace enc = sensor_msgs::image_encodings;




cv::Point2f center(0,0);


cv::Point2f computeIntersect(cv::Vec4i a, cv::Vec4i b)
{
	int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3], x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];
	float denom;
	if (float d = ((float)(x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4)))
	{
	cv::Point2f pt;
	pt.x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d;
	pt.y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d;
	return pt;
	}
	else
	return cv::Point2f(-1, -1);
}


void sortCorners(std::vector<cv::Point2f>& corners, cv::Point2f center)
{
	std::vector<cv::Point2f> top, bot;
	for (int i = 0; i < corners.size(); i++)
	{
	if (corners[i].y < center.y)
	top.push_back(corners[i]);
	else
	bot.push_back(corners[i]);
	}
	corners.clear();
	if (top.size() == 2 && bot.size() == 2){
	cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
	cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
	cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
	cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];
	corners.push_back(tl);
	corners.push_back(tr);
	corners.push_back(br);
	corners.push_back(bl);
	}
}





int callback(const sensor_msgs::ImageConstPtr& original_image)
{

printf("doing nothing \n");

	 //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
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
	

	
	cv::Mat bw;
	cv::cvtColor(cv_ptr->image, bw, CV_BGR2GRAY);
	cv::blur(cv_ptr->image, bw, cv::Size(3, 3));
	cv::Canny(cv_ptr->image, bw, 100, 100, 3);

	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(bw, lines, 1, CV_PI/180, 70, 30, 10);

	// Expand the lines
	for (int i = 0; i < lines.size(); i++)
	{
	cv::Vec4i v = lines[i];
	lines[i][0] = 0;
	lines[i][1] = ((float)v[1] - v[3]) / (v[0] - v[2]) * -v[0] + v[1];
	lines[i][2] = cv_ptr->image.cols;
	lines[i][3] = ((float)v[1] - v[3]) / (v[0] - v[2]) * (cv_ptr->image.cols - v[2]) + v[3];
	}

	

	
	std::vector<cv::Point2f> corners;
	for (int i = 0; i < lines.size(); i++)
	{
	for (int j = i+1; j < lines.size(); j++)
	{
	cv::Point2f pt = computeIntersect(lines[i], lines[j]);
	if (pt.x >= 0 && pt.y >= 0)
	corners.push_back(pt);
	}
	}

	std::vector<cv::Point2f> approx;
	cv::approxPolyDP(cv::Mat(corners), approx, cv::arcLength(cv::Mat(corners), true) * 0.02, true);
	if (approx.size() != 4)
	{
	std::cout << "The object is not quadrilateral!" << std::endl;
	return -1;
	}

	// Get mass center
	for (int i = 0; i < corners.size(); i++)
	center += corners[i];
	center *= (1. / corners.size());
	sortCorners(corners, center);
	if (corners.size() == 0){
	std::cout << "The corners were not sorted correctly!" << std::endl;
	return -1;
	}
	cv::Mat dst = cv_ptr->image.clone();


	// Draw lines
	for (int i = 0; i < lines.size(); i++)
	{
	cv::Vec4i v = lines[i];
	cv::line(dst, cv::Point(v[0], v[1]), cv::Point(v[2], v[3]), CV_RGB(0,255,0));
	}
	// Draw corner points
	cv::circle(dst, corners[0], 3, CV_RGB(255,0,0), 2);
	cv::circle(dst, corners[1], 3, CV_RGB(0,255,0), 2);
	cv::circle(dst, corners[2], 3, CV_RGB(0,0,255), 2);
	cv::circle(dst, corners[3], 3, CV_RGB(255,255,255), 2);

	// Draw mass center
	cv::circle(dst, center, 3, CV_RGB(255,255,0), 2);
	cv::Mat quad = cv::Mat::zeros(300, 220, CV_8UC3);

	std::vector<cv::Point2f> quad_pts;
	quad_pts.push_back(cv::Point2f(0, 0));
	quad_pts.push_back(cv::Point2f(quad.cols, 0));
	quad_pts.push_back(cv::Point2f(quad.cols, quad.rows));
	quad_pts.push_back(cv::Point2f(0, quad.rows));
	cv::Mat transmtx = cv::getPerspectiveTransform(corners, quad_pts);
	cv::warpPerspective(cv_ptr->image, quad, transmtx, quad.size());
	cv::imshow("image", dst);
	cv::imshow("quadrilateral", quad);
	cv::waitKey();	








	 cv::imshow("Original",cv_ptr->image ); //show the original image
	    cv::waitKey(3);
	return 0;


}







int main(int argc, char **argv)
{


   ros::init(argc, argv, "image_processor");
    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub;


    image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, callback);

	 cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);
pub = it.advertise("camera/image_processed", 1);


  ros::spin();

}









