cmake_minimum_required(VERSION 2.8.3)
project(moveit_simple)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  control_msgs
  dynamic_reconfigure
  eigen_conversions
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_visual_tools
  prettyprint
  roscpp
  rosparam_handler
  sensor_msgs
  tf2
  tf2_ros
  trajectory_msgs
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

generate_ros_parameter_files(cfg/moveit_simple_dynamic_reconfigure_.params)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    moveit_simple
  CATKIN_DEPENDS
    actionlib
    cmake_modules
    control_msgs
    eigen_conversions
    geometry_msgs
    roscpp
    sensor_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    trajectory_msgs
  DEPENDS
    Boost
    Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(moveit_simple src/moveit_simple.cpp)
add_dependencies(moveit_simple ${catkin_EXPORTED_TARGETS})
add_dependencies(moveit_simple ${PROJECT_NAME}_gencfg)
add_dependencies(moveit_simple ${PROJECT_NAME}_genparam)
target_link_libraries(moveit_simple ${catkin_LIBRARIES})
target_compile_options(moveit_simple PUBLIC -std=c++11)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  set(UTEST_SRC_FILES test/utest.cpp
      test/moveit_simple.cpp)

  add_rostest_gtest(${PROJECT_NAME}_utest test/launch/utest.launch ${UTEST_SRC_FILES})
  target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()


