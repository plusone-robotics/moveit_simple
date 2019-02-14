cmake_minimum_required(VERSION 2.8.3)
project(moveit_simple_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_action_files(
  DIRECTORY action
  FILES
    LookupTrajectory.action
    LookupWaypoint.action
)

add_message_files(
  FILES
    CombinedJointPoint.msg
)

add_service_files(
  FILES
    LookupTrajectory.srv
    LookupWaypoint.srv
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    actionlib_msgs
    message_runtime
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
)

