/*
 * MainController.cpp
 *
 *  Created on: Jan 29, 2010
 *      Author: joseph
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <command.h>		//use message of staubli package; why didn't this file include remotely?

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

  //Got target position

  //Move robot arm to the target position
	  //send a message
  //Grasp target

  return 0;
}
