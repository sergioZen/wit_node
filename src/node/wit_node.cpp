/* $Id$ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010 Jack O'Quin
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>
#include <signal.h>
#include "../../include/wit_node/wit_driver.hpp"
#include <pluginlib/class_list_macros.h>
#include <ecl/threads/thread.hpp>
#include "../../include/wit_node/wit_ros.hpp"

/** @file

    @brief ROS driver node for IIDC-compatible IEEE 1394 digital cameras.

*/

void timerCallback(const ros::TimerEvent&)
{
  ROS_INFO("Timer callback called");
}


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wit_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  std::string s("wit_node");
  wit::WitRos wd_ = wit::WitRos(s);

  try {
    wd_.init(node);
  } catch (ecl::StandardException &e) {
    std::cout << e.what();
  }

  ros::Rate rate(1);  // Set the frequency to 1 Hz

  ros::Timer timer = node.createTimer(ros::Duration(1.0), timerCallback);

  ROS_INFO_STREAM("Wit : timer de [1.0]");

  ros::Time start_time = ros::Time::now();
  ros::Duration wait_time = ros::Duration(.1);  // Wait time of 5 seconds

  Rate spin_rate(20);
  while (ok()) {
    wd_.update();
    spin_rate.sleep();
  }  

  /*
  while (ok())
  {
    while ((ros::Time::now() - start_time) > wait_time)
    {
      wd_.update();

      start_time = ros::Time::now();

      // Execute command after waiting for 5 seconds
      ROS_INFO("Waited for 5 seconds! << wait_time");

      ros::spinOnce();
    }

    rate.sleep();
  } 
  */

  return 0;
}