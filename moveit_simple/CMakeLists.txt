cmake_minimum_required(VERSION 2.8.12)
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
  tf2_geometry_msgs
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
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    actionlib
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
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/moveit_simple.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_genparam)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC -std=c++11)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(UTEST_SRC_FILES test/utest.cpp
      test/moveit_simple.cpp)

  add_rostest_gtest(${PROJECT_NAME}_utest test/launch/utest.launch ${UTEST_SRC_FILES})
  target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
