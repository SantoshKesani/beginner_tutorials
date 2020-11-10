/*
 *  @file    talkertest.cpp
 *  @author  Santosh Kesani
 *  @brief   Implementation of tests for talker
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

#include "ros/ros.h"
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/updateService.h"
#include "std_msgs/String.h"
#include <tf/tf.h>


/**
 * @brief A test for the ecistence of service
 */
TEST(TestTalkerNode, checkServiceInitialization) {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::updateService>("new_string");
  EXPECT_TRUE(client.waitForExistence(ros::Duration(10)));
}

/**
 * @brief A test to check the broadcast of tf frame
 */
TEST(tf, setTransformNoInsertWithNoFrameID) { 
  tf::Transformer mTR(true);
  tf::StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10.0), "world", "");
  EXPECT_FALSE(mTR.setTransform(tranStamped));
}

/**
 * @brief      Main function to run all tests
 */
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

