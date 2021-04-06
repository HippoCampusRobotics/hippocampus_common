cmake_minimum_required(VERSION 3.0.2)
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
catkin_install_python(PROGRAMS
  nodes/quat_to_rpy_node
  nodes/rpy_to_att_target_node
  nodes/tf_publisher_node
  nodes/water_baro_node
  scripts/rqt.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(${catkin_INCLUDE_DIRS})
