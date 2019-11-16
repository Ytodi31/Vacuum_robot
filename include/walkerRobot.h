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
 * @file walkerRobot.h
 * @brief This file contains the class declaration for class VacuumRobot
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
#ifndef INCLUDE_WALKERROBOT_H_
#define INCLUDE_WALKERRROBOT_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
/**
 * @brief Vacuum class checks for obstacle and commanfs robot to navigate
 */
class VacuumRobot {
public:
  /**
   * @brief The function checks for obstacles in the robot vicinity
   * @param pointer to Laser scan messages being published on rostopic /scan
   * @return none
   */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg);

  /**
   * @brief The function provides control to the robot by giving velocity
   * @param none
   * @return none
   */
  void velocityController();

private:
  /**
   * @brief ros parameter of node handle type to create a local node
   */
    ros::NodeHandle n;

    /**
     * @brief boolean parameter,true if robot can move safely,false otherwise
     */
    bool robotState;
};
#endif
