/*
* Brief : Implement the novel algorithm for visual navigation with single camera.
* Author : Soonhac Hong (sh2723@columbia.edu)
* Usage : $rosmake vNavigation
*		  $rosmake wireless_camera
*		  $roscore
*         $rosrun wireless_camera wireless_camera.py
*         $rosrun vNavigation vNavigation
*/

#include <iostream>
#include "stdlib.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "vNavigation/irobot_create.h"
#include "vNavigation/SpaceFinder.h"

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"

using namespace std;

SpaceFinder sf;

//====================================================================
//Define class to get video (it also calls functions to control robot)
//====================================================================
class visualNavigation
{
	public:
	visualNavigation(ros::NodeHandle &n) :
		n_(n), it_(n_)
	{
		justStarted = true;
		choice = 27;
		image_pub_ = it_.advertise("vNavigation/image",1);
//		image_sub_ = it_.subscribe("camera1394/camera/image_raw", 1, &visualNavigation::imageCallback, this);		//"camera_1394/camera/image_raw" is the image published from a camera package
		image_sub_ = it_.subscribe("wireless_camera/image", 1, &visualNavigation::imageCallback, this);		//"camera_1394/camera/image_raw" is the image published from a camera package
	}

	~visualNavigation()
	{
		
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
	{
		//ROS_INFO("Image Subscribed...\n");
		cv_image = NULL;
		try
		{
		  cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
		}
		catch (sensor_msgs::CvBridgeException error)
		{
		  ROS_ERROR("error");
		}

		sf.setSource(cv_image);		//copy the current image massage to the current frame

		if(justStarted)
		{
			justStarted = false;
			sf.init();				//Initialize the variables
		}else{
			choice = cvWaitKey(10);
			switch(choice){
				case 'd' : if(!sf.driving){
								sf.driving = true;
								cout << "\n** Manual control enabled - use the arrow keys to steer robot **\n";
							}else{
								sf.driving = false;
								cout << "\n** Manual control disabled **\n";
							}
							break;
				case 'q' : sf.stopRobot();
						   cout << "\n\n** Program Terminated **\n\n";
						   exit(0);
						   break;
				case 'f' : sf.viewFPS = !sf.viewFPS;break;
				case 's' : sf.isFinding = !sf.isFinding;break;
				default : sf.viewImage();
			}
			if(sf.driving)
				sf.driveRobot(choice);
		}
	}

	protected:

	ros::NodeHandle n_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	sensor_msgs::CvBridge bridge_;
	image_transport::Publisher image_pub_;
	
	bool justStarted;
	IplImage* cv_image;
	char choice;
};

//==========================================================
//Start getting and analyzing video
//============================================================
int main(int argc, char** argv)
{
	cout << "\n\n** Program initiated **\n\n";
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle n;
	visualNavigation vn(n);
	ros::spin();
	
	sf.stopRobot();
	return 0;
}











