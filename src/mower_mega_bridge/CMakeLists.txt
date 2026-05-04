cmake_minimum_required(VERSION 3.0.2)
project(mower_mega_bridge)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  mower_msgs
  serial
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs std_msgs mower_msgs serial
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(mower_mega_bridge_node src/mower_mega_bridge_node.cpp)
target_link_libraries(mower_mega_bridge_node ${catkin_LIBRARIES})

add_executable(calibrate_odom_gain_node src/calibrate_odom_gain_node.cpp)
target_link_libraries(calibrate_odom_gain_node ${catkin_LIBRARIES})
