cmake_minimum_required(VERSION 2.8.3)
project(the_walker_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs geometry_msgs move_base_msgs)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(DEPENDENCIES std_msgs)

## Declaring a Catkin package
catkin_package()

## Build
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(walker src/walkerRobot.cpp src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(walker ${catkin_LIBRARIES})

## Specify dependencies to link a library or executable target against
#add_dependencies(walker beginner_tutorials_generate_messages_cpp)
