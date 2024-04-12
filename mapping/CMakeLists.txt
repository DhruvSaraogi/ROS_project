cmake_minimum_required(VERSION 3.0.2)
project(mapping)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

install(DIRECTORY launch config rviz world 
  DESTINATION share/${PROJECT_NAME}/
)
