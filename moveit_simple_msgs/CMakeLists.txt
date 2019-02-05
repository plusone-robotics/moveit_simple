cmake_minimum_required(VERSION 2.8.3)
project(moveit_simple_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_service_files(
  FILES
  TrajectoryTrainer.srv
)

add_message_files(
  FILES
  CombinedJointPoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

