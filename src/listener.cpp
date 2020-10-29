/*
 *  @file    listener.cpp
 *  @author  Santosh Kesani
 *  @brief   Implementation of subscriber
 *  @License BSD 3 license
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 2000, chatterCallback);

  ros::spin();


  return 0;
}

