#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "vNavigation/irobot_create.h"

#include <time.h>

using namespace std;

class SpaceFinder
{
public:
	//==================Functions==================
	SpaceFinder();
	void init();				//Initializes variables and robot
//	void centerObject();			//Moves the robot to center the object (calls locateObject() and moveRobot)
//	void getObjectFromUser();		//Allows user to select source frame and object (calls getROI and uses mouse_callback)
	void setSource(IplImage*); //Sets source frame to be analyzed
	void driveRobot(char);
	void stopRobot();
//	void adjustDistance(char);
	void viewImage();
	void findSpace();
	void CalculateEdgeDensity(IplImage* img);
	void drawCells(int x, int y, int width, int height, int value);
	void findSafeAreaCenter();

	//==================Variables==================
	bool initialized;
	bool driving;
	bool viewFPS;
	bool isFinding;
	IplImage* frame;
	IplImage* src;
	
private:
	//==================Functions==================
	void locateObject();												//Finds position of object in current frame
	void getROI();											//Allows user to select object
	static void mouse_callback(int event, int x, int y, int flags, void* param);	//Draws box on image while user selects object
	void moveRobot();														//Moves robot to center object
	
	//==================Variables==================
	IRobotCreate robot;

	double area_thresh;
	double ERR_side;
	double ERR_size;

	double center_x;
	double screen_width;
	double screen_height;
	int buffer_w;
	int buffer_h;

	double drive_speed;
	double turn_speed;

	IplImage* bwframe;
	CvHistogram* hist;
	IplImage* back_project;
	IplImage* hsv2;
	IplImage* h_plane2;
	IplImage* s_plane2;
	IplImage* v_plane2;
	IplImage* img_edge;

	CvPoint min_loc;
	CvPoint max_loc;
	CvRect ROI;
	CvPoint imageCenter;
	CvRect safeArea;
	
	int light_pixel_threshold;
	double percent_light_threshold;

	CvSeq* first_contour;
	CvSeq* c;

	struct blob
	{
		CvPoint center;
		double size;
		double normSize;
	} object;
	
	time_t start,end;
	double time_diff;
	double frame_rate;
	double frames;
	
	bool loadImage;

	struct _cell{
		unsigned int width;		//unit : pixel
		unsigned int height;		//unit : pixel
		unsigned int edges;		//Number of edges
		unsigned int colorIndex;	//color index to show the level of edges
		bool isSafeArea;
	}cell[10][10];

	int hCells; //= 10;	// the number of horizontal cell
	int vCells; //= 10;	// the number of vertical cell
};
