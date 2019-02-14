cmake_minimum_required(VERSION 2.8.12)
project(moveit_simple)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  control_msgs
  dynamic_reconfigure
  eigen_conversions
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_simple_msgs
  moveit_visual_tools
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
    control_msgs
    dynamic_reconfigure
    eigen_conversions
    geometry_msgs
    moveit_simple_msgs
    roscpp
    rosparam_handler
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

add_library(${PROJECT_NAME} 
  src/joint_lock_options.cpp
  src/joint_locker.cpp
  src/online_robot.cpp
  src/point_types.cpp
  src/robot.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_genparam)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  set(KUKA_KR210_SRC_FILES test/kuka_kr210_utest.cpp test/kuka_kr210.cpp)
  add_rostest_gtest(kuka_kr210_utest test/launch/kuka_kr210_utest.launch ${KUKA_KR210_SRC_FILES})
  target_link_libraries(kuka_kr210_utest ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  set(MOTOMAN_MH5_SRC_FILES test/motoman_mh5_utest.cpp test/motoman_mh5.cpp)
  add_rostest_gtest(motoman_mh5_utest test/launch/motoman_mh5_utest.launch ${MOTOMAN_MH5_SRC_FILES})
  target_link_libraries(motoman_mh5_utest ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  set(MOVEIT_SIMPLE_TEST_SRC_FILES test/moveit_simple_utest.cpp test/joint_locker.cpp test/joint_lock_options.cpp)
  catkin_add_gtest(moveit_simple_utest ${MOVEIT_SIMPLE_TEST_SRC_FILES})
  target_link_libraries(moveit_simple_utest ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
