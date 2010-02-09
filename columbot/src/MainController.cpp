/*
 * MainController.cpp
 *
 *  Created on: Jan 29, 2010
 *      Author: joseph
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <staubli/command.h>
#include <staubli/command.h>


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
    ROS_INFO("Staubli Postion x : %f [m]", staubli_srv.response.x);
    ROS_INFO("Staubli Postion y : %f [m]", staubli_srv.response.y);
    ROS_INFO("Staubli Postion z : %f [m]", staubli_srv.response.z);
    ROS_INFO("Staubli Postion theta_x : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.theta_x));
    ROS_INFO("Staubli Postion theta_y : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.theta_y));
    ROS_INFO("Staubli Postion theta_z : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.theta_z));
  }
  else
  {
    ROS_ERROR("Failed to call service staubli");
    return 1;
  }

  //Move joints
  staubli_srv.request.command_number = 3;
  staubli_srv.request.joint1 = DEGREE_2_RADIAN(54.18);
  staubli_srv.request.joint2 = DEGREE_2_RADIAN(-30.31);
  staubli_srv.request.joint3 = DEGREE_2_RADIAN(56.6);
  staubli_srv.request.joint4 = DEGREE_2_RADIAN(-8.81);
  staubli_srv.request.joint5 = DEGREE_2_RADIAN(-23.6);
  staubli_srv.request.joint6 = DEGREE_2_RADIAN(-62.0);

  if (client.call(staubli_srv))
  {
    ROS_INFO("Staubli joint 1 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint1));
    ROS_INFO("Staubli joint 2 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint2));
    ROS_INFO("Staubli joint 3 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint3));
    ROS_INFO("Staubli joint 4 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint4));
    ROS_INFO("Staubli joint 5 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint5));
    ROS_INFO("Staubli joint 6 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint6));
  }
  else
  {
    ROS_ERROR("Failed to call service staubli");
    return 1;
  }

  //Get current joints
  staubli_srv.request.command_number = 2;
  if (client.call(staubli_srv))
  {
    ROS_INFO("Staubli joint 1 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint1));
    ROS_INFO("Staubli joint 2 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint2));
    ROS_INFO("Staubli joint 3 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint3));
    ROS_INFO("Staubli joint 4 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint4));
    ROS_INFO("Staubli joint 5 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint5));
    ROS_INFO("Staubli joint 6 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint6));
  }
  else
  {
    ROS_ERROR("Failed to call service staubli");
    return 1;
  }


  //Move line
  staubli_srv.request.command_number = 4;
  staubli_srv.request.x = 0.09929; //unit : meter
  staubli_srv.request.y = 0.54831;
  staubli_srv.request.z = 0.59587;
  staubli_srv.request.theta_x = DEGREE_2_RADIAN(-70.11);
  staubli_srv.request.theta_y = DEGREE_2_RADIAN(14.59);
  staubli_srv.request.theta_z = DEGREE_2_RADIAN(73.62);

  if (client.call(staubli_srv))
  {
      ROS_INFO("Staubli Postion x : %f [m]", staubli_srv.response.x);
      ROS_INFO("Staubli Postion y : %f [m]", staubli_srv.response.y);
      ROS_INFO("Staubli Postion z : %f [m]", staubli_srv.response.z);
      ROS_INFO("Staubli Postion theta_x : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.theta_x));
      ROS_INFO("Staubli Postion theta_y : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.theta_y));
      ROS_INFO("Staubli Postion theta_z : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.theta_z));
  }
  else
  {
    ROS_ERROR("Failed to call service staubli");
    return 1;
  }
  //Senario 1
  //Get target position

  //Move robot arm to the target position
	  //send a message
  //Grasp target

  return 0;
}
