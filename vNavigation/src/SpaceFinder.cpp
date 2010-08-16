/*
* Author: Soonhac Hong
* Updated: 07/15/2010
* Brief : This provides functions for finding and tracking a color blob
* Usage :	$rosmake icreate
*					$rosmake wireless_camera
*					$roscore
*         $rosrun wireless_camera wireless_camera.py
*         $rosrun icreate track
*/

#include <iostream>

#include "vNavigation/SpaceFinder.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "vNavigation/irobot_create.h"

using namespace std;

CvPoint g_testPoint1;
CvPoint g_testPoint2;
bool waiting = false;	
	
SpaceFinder::SpaceFinder(){}
//==============================================================
//Initialize variables and robot
//==============================================================
void SpaceFinder::init()
{
	//ERR_side = 0.1;
	//ERR_size = 0.4;
	
	//These two variables are re-set when user selects object
	//object.normSize = -1700;
	//area_thresh = -300;
	
	drive_speed = 50;
	turn_speed = 60;
	
	//robot.Start();		//7/20/2010 by soonhac for testing without a Create
	//cout << "\n Started robot!\n";
	
	

	//g_testPoint1 = cvPoint(-1,-1);
	//g_testPoint2 = cvPoint(-1,-1);
	//min_loc = cvPoint(-1,-1);
	//max_loc = cvPoint(-1,-1);

	//light_pixel_threshold = 10;
	//percent_light_threshold = 0.25;

	waiting = true;
	initialized = false;
	driving = false;
	viewFPS = false;
	loadImage = true;
	isFinding = false;

	bwframe = cvCreateImage(cvGetSize(frame), 8, 1);
	back_project = cvCreateImage(cvGetSize(frame), 8, 1);
	img_edge = cvCreateImage(cvGetSize(frame),8,1);
	hsv2 = cvCreateImage(cvGetSize(frame), 8, 3);
	h_plane2 = cvCreateImage(cvGetSize(frame), 8, 1);
	s_plane2 = cvCreateImage(cvGetSize(frame), 8, 1);
	v_plane2 = cvCreateImage(cvGetSize(frame), 8, 1);
	//planes2[0] = h_plane2;
	//planes2[1] = s_plane2;

	screen_width = cvGetSize(frame).width;
	screen_height = cvGetSize(frame).height;
	center_x = (double)screen_width/2.0;
	buffer_w = screen_width * 0.25;
	buffer_h = screen_height * 0.25;
	imageCenter = cvPoint((int)screen_width/2,(int)screen_height/2);

	time_diff = 0;
	frame_rate = 0;	
	frames = 0;
	
	//cout << "\n*** Press space to select source frame ***\n";
	//cout << "    (you may press 'd' to steer the robot with the arrow keys)\n";
	//cvNamedWindow("Back Projection");
	cvNamedWindow("Video");
	cvNamedWindow("Image Processing");
	//cvNamedWindow("Draw a box around object");
	//cvNamedWindow("Hold space to select source frame");
}

//==============================================================
//Calls all other needed functions to find object and move robot
//==============================================================
void SpaceFinder::centerObject()
{
	locateObject();
	if(!driving)
		moveRobot();
		
		
	time(&end);
	frames++;
	time_diff = difftime(end,start);
	frame_rate = frames/time_diff;
	if(viewFPS)
	{
		printf ("\n   Average frame rate: %.2lf fps\n", frame_rate);
		viewFPS = false;
	}
}

/**
 * View the current frame.
 */
void SpaceFinder::viewImage(){
	if(isFinding){
		findSpace();
	}

	cvShowImage("Video", src);
}

/**
 * Find the space for moving a robot
 */
void SpaceFinder::findSpace(){
	//ROS_INFO("Start to find the space.");

//	int width=100,height=100;
//
//	safeArea = cvRect(imageCenter.x - width/2,imageCenter.y - height/2,width,height);		//cvRect(x,y,width,height);
//	//ROS_INFO("%d,%d,%d,%d",safeArea.x,safeArea.y,safeArea.width,safeArea.width);
//	CvPoint point1 = cvPoint(safeArea.x, safeArea.y);
//	CvPoint point2 = cvPoint(safeArea.x + safeArea.width, safeArea.y + safeArea.height);
//	cvDrawRect(frame, point1, point2, cvScalar(0,255,0), 4);

	//Get the gray image
	cvCvtColor(frame ,bwframe, CV_RGB2GRAY);

	//Edge detection
	// Edge Detection Variables
	int kernelSize = 3;
	double lowThresh = 10;
	double highThresh = 60;
	cvCanny(bwframe,img_edge,lowThresh*kernelSize*kernelSize,highThresh*kernelSize*kernelSize,kernelSize);

	cvShowImage("Image Processing", img_edge);
	//viewImage();



}

//============================================================
//Allows user to select source frame and object
//============================================================
void SpaceFinder::getObjectFromUser()
{	
	if(!initialized) //Need to get source frame
	{
		if(loadImage && (src = cvLoadImage("source.jpg")) != NULL) //Source frame was previously saved
		{
			cvShowImage("Hold space to select source frame", src);
			cout << "\n*** Load saved source frame? (y/n) " << flush;
			char choice = cvWaitKey();
			cout << endl << endl;
			if(choice == 'y')
			{
				cout << "\n** Loaded saved source frame **";
				initialized = true;	
				cvDestroyWindow("Hold space to select source frame");
				frame = cvLoadImage("source.jpg");
			}
			else
			{
				loadImage = false;
				cout << "\n*** Press space to select source frame ***\n";
				cout << "    (you may press 'd' to steer the robot with the arrow keys)\n";
			}
		}
		else
		{
			cvShowImage("Hold space to select source frame", frame);
			char c = cvWaitKey(10);
			if(c == ' ')
			{
				cout << "\n Source frame selected\n";
				cvSaveImage("source.jpg", frame);
				cvDestroyWindow("Hold space to select source frame");
				initialized = true;
			}
		}
	}
	if(initialized)
	{				
		//Set ROI (object)
		getROI();
		cvSetImageROI(frame, ROI);
		
		//Make H,S,V planes for object
		IplImage* hsv = cvCreateImage(cvGetSize(frame), 8, 3);
		cvCvtColor(frame, hsv, CV_BGR2HSV);
		IplImage* h_plane = cvCreateImage(cvGetSize(frame), 8, 1);
		IplImage* s_plane = cvCreateImage(cvGetSize(frame), 8, 1);
		IplImage* v_plane = cvCreateImage(cvGetSize(frame), 8, 1);
		IplImage* planes[] = {h_plane, s_plane};
		cvCvtPixToPlane(hsv, h_plane, s_plane, v_plane, 0);
		
		//Calculate histogram for object
		int h_bins = 30, s_bins = 32;
		int hist_size[] = {h_bins, s_bins};
		float h_ranges[] = {0, 180};
		float s_ranges[] = {0, 255};
		float* ranges[] = {h_ranges, s_ranges};
		hist = cvCreateHist(2, hist_size, CV_HIST_ARRAY, ranges, 1);

		cvCalcHist(planes, hist, 0, 0);
		ROI = cvRect(ROI.x - buffer_w, ROI.y - buffer_h, ROI.width + buffer_w * 2, ROI.height + buffer_h * 2);
		cvResetImageROI(frame);		
	}
}

//============================================================
//Analyze frame to find object
//============================================================
void SpaceFinder::locateObject()
{
	cvSetImageROI(frame, ROI);
	cvShowImage("ROI", frame);
	
	cvSetImageROI(back_project, ROI);
	h_plane2 = cvCreateImage(cvGetSize(back_project), 8, 1);
	s_plane2 = cvCreateImage(cvGetSize(back_project), 8, 1);
	v_plane2 = cvCreateImage(cvGetSize(back_project), 8, 1);
	hsv2 = cvCreateImage(cvGetSize(back_project), 8, 3);
	
	CvMemStorage* storage = cvCreateMemStorage();
	first_contour = NULL;
	c = first_contour;

	cvZero(back_project);

	//Create H,S,V planes of image to be analyzed
	cvCvtColor(frame, hsv2, CV_BGR2HSV);
	cvCvtPixToPlane(hsv2, h_plane2, s_plane2, v_plane2, 0);
	cvResetImageROI(frame);
	
	//Calculate back projetion of image
	IplImage* planes2[] = {h_plane2, s_plane2};
	cvCalcBackProject(planes2, back_project, hist);
	
	//Threshold and smooth projection
	cvThreshold(back_project, back_project, 50, 255, CV_THRESH_BINARY);
	cvErode(back_project, back_project);
	cvSmooth(back_project, back_project, CV_BLUR, 15, 15);
	cvThreshold(back_project, back_project, 25, 255, CV_THRESH_BINARY);
	cvShowImage("Back Projection", back_project);
	IplImage* temp = cvCloneImage(back_project);
	
	//Initialize variables to find contours
	cvFindContours(temp, storage, &first_contour, sizeof(CvContour), CV_RETR_LIST);
	double area = 0;
	CvRect boundBox = cvRect(0,0,0,0);

	//Find the first contour that has appropriate area and percent light pixels
	bool goodContour = false;
	double light = 0, percentLight = 0;
	for(c = first_contour; !goodContour && c != NULL; c = c->h_next)
	{
		area = (double)cvContourArea(c, CV_WHOLE_SEQ);
		if(abs(area) > abs(area_thresh)) //Check how many pixels in contour are light
		{
			light = percentLight = 0;	
				
			boundBox = cvBoundingRect(c);
			cvSetImageROI(back_project, boundBox);
			for(int y=0; y < back_project->height; y++)
			{
				uchar* ptr = (uchar*) (back_project->imageData + y * back_project->widthStep);
				for(int x=0; x < back_project->width; x++)
				{
					if(ptr[x] > light_pixel_threshold)
						light++;
				}
			}
			cvResetImageROI(back_project);
			cvSetImageROI(back_project, ROI);
			percentLight = light/(double)(boundBox.width*boundBox.height);
			if(percentLight >= percent_light_threshold)
				goodContour = true;
		}

	}
	cvResetImageROI(back_project);
	boundBox.x += ROI.x;
	boundBox.y += ROI.y;

	CvPoint point1 = cvPoint(boundBox.x, boundBox.y);
	CvPoint point2 = cvPoint(boundBox.x + boundBox.width, boundBox.y + boundBox.height);
	CvPoint center = cvPoint(boundBox.x + boundBox.width / 2, boundBox.y + boundBox.height / 2);
			
	//Draw rectangle around object and center bullseye on back projection
	cvDrawRect(back_project, point1, point2, cvScalar(255,255,255), 4);
	cvCircle(back_project, center, 2, cvScalar(0,0,0), 12);
	cvCircle(back_project, center, 2, cvScalar(255,255,255), 8);
	cvCircle(back_project, center, 2, cvScalar(0,0,0), 2);

	//Draw rectangle around object, center bullseye, and left/right tracking bounds on actual frame
	cvDrawRect(frame, point1, point2, cvScalar(0,255,0), 4);
	cvCircle(frame, center, 2, cvScalar(0,0,255), 14);
	cvCircle(frame, center, 2, cvScalar(255,255,255), 8);
	cvCircle(frame, center, 2, cvScalar(0,0,255), 2);
	cvDrawRect(frame, cvPoint(center_x - center_x * ERR_side, 0), cvPoint(center_x + center_x * ERR_side, frame->height), cvScalar(0,0,255), 6);
	cvShowImage("Video", frame);

	//Update object's location and size
	object.center = cvPoint(boundBox.x + boundBox.width / 2, boundBox.y + boundBox.y / 2);
	object.size = area;
	
	//Update region of frame in which to look for object
	if(!goodContour)
	{
		if(ROI.width == screen_width && ROI.height == screen_height)
			object.center = cvPoint(0, frame->height/2); // force robot to turn right
		else
			ROI = cvRect(0, 0, screen_width, screen_height);
	}
	else
			ROI = cvRect(boundBox.x - buffer_w, boundBox.y - buffer_h, boundBox.width + buffer_w * 2, boundBox.height + buffer_h * 2);
			
	if(ROI.x < 0)
		ROI.x = 0;
	else if(ROI.x > screen_width)
		ROI.x = screen_width;
	if(ROI.x + ROI.width > screen_width)
		ROI.width = screen_width - ROI.x;
	
	if(ROI.y < 0)
		ROI.y = 0;
	else if(ROI.y > screen_height)
		ROI.y = screen_height;
	if(ROI.y + ROI.height > screen_height)
		ROI.height = screen_height - ROI.y;		
	
			
}

//============================================================
//Moves the robot to center the object
//============================================================
void SpaceFinder::moveRobot()
{
	int x_vel = 0;
	int w_vel = 0;
	
	if((object.center.x - center_x)/center_x > ERR_side)
	{
		w_vel = turn_speed;
		x_vel = 0;		
		//cout << "\nTURN LEFT\n";
	}
	else if((center_x - object.center.x)/center_x > ERR_side)
	{
		w_vel = -turn_speed;
		x_vel = 0;
		//cout << "\nTURN RIGHT\n";
	}
	else if((object.size  - object.normSize)/object.normSize > ERR_size)
	{
		w_vel = 0;
		x_vel = -drive_speed;
		//cout << "\nDRIVE BACKWARD " << object.size << endl;
	}
	else if((object.normSize - object.size)/object.normSize > ERR_size)
	{
		w_vel = 0;
		x_vel = drive_speed;
		//cout << "\nDRIVE FORWARD " << object.size << endl;
	}
	else
	{
		w_vel = 0;
		x_vel = 0;
		//cout << "\nSTOP\n";
	}
	//cout << "Size: " << object.size << endl;
	//robot.setVelocity(x_vel,w_vel);
}

//============================================================
//Allow user to select object in chosen frame
//============================================================
void SpaceFinder::getROI()
{
	cout << "\n\n*** Select the object by drawing a box (begin with upper left corner and then drag) ***\n" << flush;
	IplImage* temp = cvCloneImage(frame);
	cvSetMouseCallback("Draw a box around object", mouse_callback, (void*)temp);
	while(waiting)
	{
		temp = cvCloneImage(frame);
		if(g_testPoint1.x != -1)
			cvDrawRect(temp, g_testPoint1, g_testPoint2, cvScalar(0,0,255), 2);
		cvShowImage("Draw a box around object", temp);
		cvWaitKey(50);
	}
	ROI = cvRect(g_testPoint1.x, g_testPoint1.y, g_testPoint2.x - g_testPoint1.x, g_testPoint2.y - g_testPoint1.y);
	cvDestroyWindow("Draw a box around object");
	
	cout << "\n Got object! Now tracking... \n";
	cout << "\n Press 'm' to view menu at any time \n";
	
	if(driving)
	{
		cout << "\n** Manual control has been disabled.  Press 'd' again to rectify this. **\n";
		driving = false;
	}
		
	time(&start);
	
	object.normSize = ROI.width * ROI.height * -1;
	area_thresh = object.normSize / 5;
	cout << "\nleaving getROI\n" << flush;
}

//============================================================
//Draws box on chosen frame
//============================================================
void SpaceFinder::mouse_callback(int event, int x, int y, int flags, void* param)
{
	if(waiting)
	{
		switch(event)
		{
			case CV_EVENT_LBUTTONDOWN:
					g_testPoint1 = cvPoint(x, y);
					g_testPoint2 = cvPoint(x, y);
				break;
			case CV_EVENT_LBUTTONUP:
				waiting = false;
				g_testPoint2 = cvPoint(x, y);
				break;
			case CV_EVENT_MOUSEMOVE:
				g_testPoint2 = cvPoint(x, y);
		}

	}
}

/**
 * Set the current frame with the image message from other packages
 */
void SpaceFinder::setSource(IplImage* source)
{
	//cvFlip(source, source, 1);
	src = cvCloneImage(source);
	frame = cvCloneImage(source);
}

//============================================================
//Allows user to drive robot with arrow keys
//============================================================
void SpaceFinder::driveRobot(char dir)
{
	int x_vel = 0;
	int w_vel = 0;
	
	switch(dir)
	{
		case 82: //Up
			w_vel = 0;
			x_vel = drive_speed;
			//cout << "\nMan_UP\n";
			break;
		case 84: //Down
			w_vel = 0;
			x_vel = -drive_speed;
			//cout << "\nMan_DOWN\n";
			break;
		case 81: //Left
			w_vel = turn_speed;
			x_vel = 0;
			//cout << "\nMan_LEFT\n";
			break;
		case 83: //Right
			w_vel = -turn_speed;
			x_vel = 0;
			//cout << "\nMan_RIGHT\n";
			break;		
		default:
			w_vel = 0;
			x_vel = 0;
			break;			
	}
	
	//robot.setVelocity(x_vel,w_vel);
}

//============================================================
//Stops the robot
//============================================================
void SpaceFinder::stopRobot()
{
	//robot.setVelocity(0,0);
	cout << "\n Robot stopped\n";
}

//============================================================
//Changes the normal size of object by 10%
//============================================================
//void SpaceFinder::adjustDistance(char choice)
//{
//	if(choice == '.')
//		object.normSize *= 1.1;
//	else if(choice == ',')
//		object.normSize *= 0.9;
//
//	cout << "\n** Adjusted target size **\n";
//}
