cmake_minimum_required(VERSION 3.0.2)
project(mower_ios_bridge)

add_compile_options(-std=c++17)

find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mower_map
  mower_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  xbot_rpc
  xbot_msgs
)

add_subdirectory(../lib/json json EXCLUDE_FROM_ALL)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    mower_map
    mower_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    xbot_rpc
    xbot_msgs
)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(mower_ios_bridge_node
  src/mower_ios_bridge_node.cpp
)

add_dependencies(mower_ios_bridge_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mower_ios_bridge_node
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  nlohmann_json::nlohmann_json
  xbot_rpc
)

install(TARGETS mower_ios_bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/mower_ios_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
