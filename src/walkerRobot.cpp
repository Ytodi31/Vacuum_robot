/**
*BSD 3-Clause License
*
*Copyright (c) 2019, Yashaarth Todi
*All rights reserved.
*
*Redistribution and use in source and binary forms, with or without
*modification, are permitted provided that the following conditions are met:
*1. Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*3. Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * @file walkerRobot.cpp
 * @brief This file contains the definition of member functions of class VacuumRobot
 *
 * This project contains the execution to navigate a turtlebot much like
 * the Roomba vacuum robot while avoiding obstacles
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Yashaarth Todi
 *
 * @date 11-13-2019
 */
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "walkerRobot.h"

void VacuumRobot::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  for (int i =0; i < msg -> ranges.size(); ++i) {
    // Checking if there is an obstacle in range of 1m of robot
    if (msg -> ranges[i] <= 1) {
      // Setting robot state as unsafe to move
      VacuumRobot::robotState = false;
      ROS_INFO("Obstacle detected, changing path");
      return;
    } else {
      // Setting robot state as safe to move
      VacuumRobot::robotState = true;
    }
    }
  }


void VacuumRobot::velocityController() {

  // Creating a publisher to send velcoity to topic to control robot
  auto velocity_pub = n.advertise<geometry_msgs::Twist>
  ("/mobile_base/commands/velocity", 1000);

  // Creating an object of geometry_msgs/twost type to access velocity
  geometry_msgs::Twist velocity;

  // Initialising linear and angular velocity parameters
  velocity.linear.y = 0;
  velocity.linear.z = 0;
  velocity.angular.x = 0;
  velocity.angular.y = 0;

  // Defining a subscriber to read the laser scans from topic /scan
  auto sub = n.subscribe<sensor_msgs::LaserScan>
             ("/scan", 1000, &VacuumRobot::laserCallback, this);

  ros::Rate loop_rate(10);

  // Checking for successful connection to ROS master
  while (ros::ok()) {
  // Checking if it is safe to move, assigns linear velocity to robot
  if (VacuumRobot::robotState == true) {
    velocity.linear.x = 0.5;
    velocity.angular.z = 0;
  } else {
    // If it is unsafe to move, assigns angular velocity to look for new heading direction
    velocity.linear.x = 0;
    velocity.angular.z = 0.5;
  }

  // Publishing the velocity to rostopic
  velocity_pub.publish(velocity);
  ros::spinOnce();
  loop_rate.sleep();
  }
}
