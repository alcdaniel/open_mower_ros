cmake_minimum_required(VERSION 3.0.2)
project(mower_ios_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mower_msgs
  rospy
  std_msgs
  xbot_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    mower_msgs
    rospy
    std_msgs
    xbot_msgs
)

catkin_install_python(PROGRAMS
  scripts/mower_ios_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
