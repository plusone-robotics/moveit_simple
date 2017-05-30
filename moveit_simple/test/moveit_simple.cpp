/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <moveit_simple/moveit_simple.h>
#include <gtest/gtest.h>

using testing::Types;

namespace moveit_simple_test
{

TEST(MoveitSimpleTest, construction)
{
  moveit_simple::Robot robot(ros::NodeHandle(), "robot_description", "manipulator");
}

TEST(MoveitSimpleTest, reachability)
{
  moveit_simple::Robot robot(ros::NodeHandle(), "robot_description", "manipulator");

  ROS_INFO_STREAM("Testing reachability of unknown point, should fail");
  EXPECT_FALSE(robot.isReachable("unknown_name"));

  ros::Duration(2.0).sleep();  //wait for tf tree to populate
  ROS_INFO_STREAM("Testing reach of points");
  ASSERT_TRUE(robot.isReachable("home"));        //stored in the SRDF
  ASSERT_TRUE(robot.isReachable("waypoint1"));   //stored in the URDF
  ASSERT_TRUE(robot.isReachable("waypoint2"));   //stored in the URDF
  ASSERT_TRUE(robot.isReachable("waypoint3"));   //stored in the URDF
  ASSERT_TRUE(robot.isReachable("tf_pub1"));     //stored published externally
  ASSERT_FALSE(robot.isReachable("waypoint4"));  //stored in the URDF
}

TEST(MoveitSimpleTest, add_trajectory)
{
  moveit_simple::Robot robot(ros::NodeHandle(), "robot_description", "manipulator");
  const std::string TRAJECTORY_NAME("traj1");

  ROS_INFO_STREAM("Testing loading of unknown point, should fail");
  EXPECT_FALSE(robot.addTrajPoint("bad_traj", "unknown_name", 1.0));

  ros::Duration(2.0).sleep();  //wait for tf tree to populate
  ROS_INFO_STREAM("Testing trajectory adding of points");
  EXPECT_TRUE(robot.addTrajPoint(TRAJECTORY_NAME, "home",      0.5));
  EXPECT_TRUE(robot.addTrajPoint(TRAJECTORY_NAME, "waypoint1", 1.0));
  EXPECT_TRUE(robot.addTrajPoint(TRAJECTORY_NAME, "tf_pub1",   2.0));
  EXPECT_TRUE(robot.addTrajPoint(TRAJECTORY_NAME, "waypoint2", 3.0));
  EXPECT_TRUE(robot.addTrajPoint(TRAJECTORY_NAME, "waypoint3", 4.0));

  EXPECT_TRUE(robot.execute(TRAJECTORY_NAME));
}

bool checkIfTimeWithinTolerance(double actual_time, double expected_time, 
  double tolerance = 0.15) // Tolerance is a percentage
{

}

TEST(MoveitSimpleTest, speed_reconfiguration)
{
  moveit_simple::Robot robot(ros::NodeHandle(), "robot_description", "manipulator");
  const std::string TRAJECTORY_NAME("traj1");

  ros::Duration(2.0).sleep();  //wait for tf tree to populate

  robot.addTrajPoint(TRAJECTORY_NAME, "home",      0.5);
  robot.addTrajPoint(TRAJECTORY_NAME, "waypoint1", 1.0);
  robot.addTrajPoint(TRAJECTORY_NAME, "tf_pub1",   2.0);
  robot.addTrajPoint(TRAJECTORY_NAME, "waypoint2", 3.0);
  robot.addTrajPoint(TRAJECTORY_NAME, "waypoint3", 4.0);

  EXPECT_TRUE(robot.getSpeedModifier() == 1.0);
  double nominal_time = robot.getTotalExecutionTime(TRAJECTORY_NAME);

  ROS_ERROR_STREAM(nominal_time);

  robot.setSpeedModifier(0.5);
  EXPECT_TRUE(robot.getSpeedModifier() == 0.5);

  double half_time = robot.getTotalExecutionTime(TRAJECTORY_NAME);

  ROS_ERROR_STREAM(half_time);

  robot.setSpeedModifier(2.0);
  EXPECT_TRUE(robot.getSpeedModifier() == 2.0);

  double double_time = robot.getTotalExecutionTime(TRAJECTORY_NAME);
  
  ROS_ERROR_STREAM(double_time);
#if 0
  EXPECT_TRUE(robot.getSpeedModifier() == 1.0);
  double start_regular_execution = ros::Time::now().toSec();
  robot.execute(TRAJECTORY_NAME);
  double end_regular_execution = ros::Time::now().toSec();

  ROS_ERROR_STREAM("Time for single execution at regular speed: " 
    << end_regular_execution - start_regular_execution << " seconds.");

  robot.setSpeedModifier(0.5);
  EXPECT_TRUE(robot.getSpeedModifier() == 0.5);

  double start_half_execution = ros::Time::now().toSec();
  robot.execute(TRAJECTORY_NAME);
  double end_half_execution = ros::Time::now().toSec();

  ROS_ERROR_STREAM("Time for single execution at half speed: "
    << end_half_execution - start_half_execution << " seconds.");

  robot.setSpeedModifier(2.0);
  EXPECT_TRUE(robot.getSpeedModifier() == 2.0);

  double start_double_execution = ros::Time::now().toSec();
  robot.execute(TRAJECTORY_NAME);
  double end_double_execution = ros::Time::now().toSec();

  ROS_ERROR_STREAM("Time for single execution at double speed: "
    << end_double_execution - start_double_execution << " seconds.");

  double regular_time = end_regular_execution - start_regular_execution;
  double half_time = end_half_execution - start_half_execution;
  double double_time = end_double_execution - start_double_execution;
#endif
}

}
