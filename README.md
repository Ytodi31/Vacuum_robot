# Vacuum Robot using Turtlebot
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview
This project implements a walker algorithm using Turtlebot while avoiding obstacles
in the environment, resembling the action of vacuum cleaning robot like the Roomba.


---
## Dependencies
- The project uses Ubuntu 16.04
- The project uses Kinetic version of ROS. To install, follow the [link]( http://wiki.ros.org/kinetic/Installation/Ubuntu)
- The project uses catkin build system, To install, follow the [link](http://wiki.ros.org/catkin)
- The project uses turtlebot_gazebo package, which is a part of ROS installation. To install, follow the [link](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation).
---
## Building the project
1. Create a catkin workspace \
`mkdir -p ~/catkin_ws/src` (Skip this step if you have an exisitng catkin worksapce)\
`cd ~/catkin_ws/` \
`catkin_make`
2. Source the new setup files \
`source devel/setup.bash`
3. Clone the repository\
`cd src/` \
`git clone https://github.com/Ytodi31/vacuum_robot.git`
4. Build the project \
`cd ..` \
`catkin_make`
---
## Running the project
`source ~/catkin_ws/devel/setup.bash` \
`roslaunch the_walker_robot vacuumTurtlebot.launch`

By default, this will have the bag recording on. To disable the bag recording and running the project, follow the steps below:\
`source ~/catkin_ws/devel/setup.bash` \
`roslaunch the_walker_robot vacuumTurtlebot.launch recordBag:=false`

---
## Bag file
- To inspect the recorded bag file :\
`cd <path to directory>/results`\
`rosbag info record.bag`

- To playback the recorded bag file, ensure that gazebo is not running.
Run the following
  - Terminal 1 - ROS master \
`source ~/catkin_ws/devel/setup.bash` \
`roscore`

 - Terminal 2\
 `cd <path to directory>/results`\
 `source ~/catkin_ws/devel/setup.bash` \
 `rosbag play record.bag`
