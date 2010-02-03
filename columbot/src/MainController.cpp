/*
 * MainController.cpp
 *
 *  Created on: Jan 29, 2010
 *      Author: joseph
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <staubli/command.h>
#include <command.h>		//use message of staubli package; why didn't this file include remotely?


#define PI 3.14
#define DEGREE_2_RADIAN(X) (X*PI/180)
#define RAIDAN_2_DEGREE(X) (X*180/PI)

void chatterCallback(const std_msgs::StringConstPtr& msg)
{
  ROS_INFO("Received [%s]", msg->data.c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MainController");
  ros::NodeHandle n;
  //ros::Subscriber chatter_sub = n.subscribe("chatter", 100, chatterCallback);
  //ros::spin();

  ros::ServiceClient client = n.serviceClient<staubli::command>("command");
  staubli::command staubli_srv;

  //Get current cartesian postion
  staubli_srv.request.command_number = 1;
  if (client.call(staubli_srv))
  {
    ROS_INFO("Staubli Postion x : %f", staubli_srv.response.x);
    ROS_INFO("Staubli Postion y : %f", staubli_srv.response.y);
    ROS_INFO("Staubli Postion z : %f", staubli_srv.response.z);
    ROS_INFO("Staubli Postion theta_x : %f", staubli_srv.response.theta_x);
    ROS_INFO("Staubli Postion theta_y : %f", staubli_srv.response.theta_y);
    ROS_INFO("Staubli Postion theta_z : %f", staubli_srv.response.theta_z);
  }
  else
  {
    ROS_ERROR("Failed to call service staubli");
    return 1;
  }

  //Debug here !!
  //Move joints
  staubli_srv.request.command_number = 2;
  staubli_srv.request.joint1 = DEGREE_2_RADIAN(54.18);
  staubli_srv.request.joint2 = DEGREE_2_RADIAN(-30.31);
  staubli_srv.request.joint3 = DEGREE_2_RADIAN(56.6);
  staubli_srv.request.joint4 = DEGREE_2_RADIAN(-8.81);
  staubli_srv.request.joint5 = DEGREE_2_RADIAN(-23.6);
  staubli_srv.request.joint6 = DEGREE_2_RADIAN(-62.0);

  if (client.call(staubli_srv))
  {
    ROS_INFO("Staubli joint 1 : %f", staubli_srv.response.joint1);
    ROS_INFO("Staubli joint 2 : %f", staubli_srv.response.joint2);
    ROS_INFO("Staubli joint 3 : %f", staubli_srv.response.joint3);
    ROS_INFO("Staubli joint 4 : %f", staubli_srv.response.joint4);
    ROS_INFO("Staubli joint 5 : %f", staubli_srv.response.joint5);
    ROS_INFO("Staubli joint 6 : %f", staubli_srv.response.joint6);
  }
  else
  {
    ROS_ERROR("Failed to call service staubli");
    return 1;
  }



  //Senario 1
  //Got target position

  //Move robot arm to the target position
	  //send a message
  //Grasp target

  return 0;
}
