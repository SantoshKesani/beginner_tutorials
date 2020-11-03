/*
 *  @file    talker.cpp
 *  @author  Santosh Kesani
 *  @brief   Implementing the talker publisher and Service
 *  @License BSD 3 license
 * 
 *  Copyright (c) 2020, SantoshKesani
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 * 
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 * 
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * 
 */

#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/updateService.h"

extern std::string newMsg = "Default Message";

/**
 * @brief      Updating default message
 *
 * @param      req   Service request
 * @param      res   Service response
 *
 * @return     { true on service execution }
 */
bool update(beginner_tutorials::updateService::Request &req,
               beginner_tutorials::updateService::Response &res) {
  
  res.updateString = req.newString;
  ROS_INFO_STREAM("Updating the string to user input");
  newMsg = res.updateString;
  return true;
}


/**
 * @brief      main function to handle ros publisher and service
 *             with all logging levels
 *
 * @param[in]  argc  The count of arguments
 * @param      argv  The arguments array
 *
 * @return     { 0 on success }
 */
int main(int argc, char **argv) {
  // Intializing ROS Node
  ros::init(argc, argv, "talker");
  // Intializing NodeHandle
  ros::NodeHandle n;
  // Creating a publisher node
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  // Creating a Service Node
  ros::ServiceServer server = n.advertiseService("new_string", update);

  // ROS loop rate
  double loopRate; // Variable to store input frequency
  if(argc == 1) {
    ROS_ERROR_STREAM("loopRate argument is missing");
    ROS_WARN_STREAM("Setting loop rate to default value");
    loopRate = 10;
  }
  else {
    n.getParam("/set_freq", loopRate);
    if(loopRate <= 0) {
      ROS_FATAL_STREAM("loopRate cannot be negative or zero");
      ROS_WARN_STREAM("Setting loop rate to default value");
      loopRate = 10;
    }
    else if(loopRate > 200) {
      ROS_ERROR_STREAM("loop rate is too high");
      ROS_WARN_STREAM("Setting loop rate to default value");
      loopRate = 10;
    }
    else {
      ROS_DEBUG_STREAM("Loop Rate is" << loopRate);
    }
  }
  ros::Rate loop_rate(loopRate);

  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;

    std::stringstream ss;
    ss << newMsg << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

