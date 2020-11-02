cmake_minimum_required(VERSION 2.8.3)
project(hippocampus_common)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  tf2
  tf2_geometry_msgs
  tf
  geometry_msgs
  nav_msgs
)

catkin_python_setup()
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
