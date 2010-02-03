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

	TX60L robot;
    robot.Login("http://128.59.20.63:5653/", "default", ""); //ANTIGUA
	//robot.Login("http://128.59.19.61:5653/", "default", ""); //Thomas' computer
	robot.Power(true);
	ROS_INFO("Staubli is connected !!.");

	std::vector<double> position;
	position.resize(6);

	if(robot.GetRobotCartesianPosition(position)){
		for_each(position.begin(), position.end(), print);
		std::cout <<"\n";
		res->x = position[0];
		res->y = position[1];
		res->z = position[2];
		res->theta_x  = position[3];
		res->theta_y  = position[4];
		res->theta_z  = position[5];
	}else
		std::cout <<"wrong\n";

	robot.Logoff();
	return 0;
}

int MoveJoints(staubli::command::Request  *req,
			   staubli::command::Response *res)
{
	TX60L robot;
    robot.Login("http://128.59.20.63:5653/", "default", ""); //ANTIGUA
	//robot.Login("http://128.59.19.61:5653/", "default", ""); //Thomas' computer
	robot.Power(true);
	ROS_INFO("Staubli is connected !!.");

	//Move joints
	std::vector<double> j;
	j[0]=req->joint1;
	j[1]=req->joint2;
	j[2]=req->joint3;
	j[3]=req->joint4;
	j[4]=req->joint5;
	j[5]=req->joint6;
	robot.ResetMotion();
	robot.MoveJoints(j);

	//Get Current joints
	std::vector<double> current_joints;
	current_joints.resize(6);

	if(robot.GetRobotJoints(current_joints)){
		for_each(current_joints.begin(), current_joints.end(), print);
		std::cout <<"\n";
		res->joint1 = current_joints[0];
		res->joint2 = current_joints[1];
		res->joint3 = current_joints[2];
		res->joint4  = current_joints[3];
		res->joint5  = current_joints[4];
		res->joint6  = current_joints[5];
	}else
		std::cout <<"wrong\n";

	robot.Logoff();
}

bool commandhandler(staubli::command::Request  &req,
					staubli::command::Response &res ){

	ROS_INFO("Command #%d.",req.command_number);
	switch(req.command_number){
		case 1 : GetPosition(&res); break;
		case 2 : MoveJoints(&req,&res);break;
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
