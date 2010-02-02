/*
 * controller.cpp
 *
 *  Created on: Jan 29, 2010
 *      Author: joseph
 */

#include "TX60L.h"
#include <iostream>
#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include "staubli/command.h"		//command service of staubli

//static TX60L robot;

void print(double e){
	std::cout << " " << e;
}

int GetPosition(staubli::command::Response *res){

	//TX60L robot;
    //robot.Login("http://128.59.20.63:5653/", "default", ""); //ANTIGUA
	//robot.Login("http://128.59.19.61:5653/", "default", ""); //Thomas' computer
	//robot.Power(true);

	//std::vector<double> position;
	//position.resize(6);

	//if(robot.GetRobotCartesianPosition(position)){
	//	for_each(position.begin(), position.end(), print);
		std::cout <<"\n";
		res->x = 1; //position[0];
		res->y = 2; //position[1];
		res->z = 3; //position[2];
		res->theta_x  = 4; //position[3];
		res->theta_y  = 5; //position[4];
		res->theta_z  = 6; //position[5];
	//}else
	//	std::cout <<"wrong\n";

	return 0;
}

bool commandhandler(staubli::command::Request  &req,
					staubli::command::Response &res ){

	ROS_INFO("Command #%d.",req.command_number);
	switch(req.command_number){
		case 1 : GetPosition(&res); break;
		default: break;
	}
	return true;

}

int main(int argc, char** argv){

	//TX60L robot;
    //robot.Login("http://128.59.20.63:5653/", "default", ""); //ANTIGUA
	//robot.Login("http://128.59.19.61:5653/", "default", ""); //Thomas' computer
	//robot.Power(true);

	ros::init(argc, argv, "staubli_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("command", commandhandler);
	ROS_INFO("Ready to access staubli.");
	ros::spin();

	return 0;
}
