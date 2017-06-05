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

  // Check if a proper plan and execution works.
  moveit_simple::TrajectoryGoal trajectory_goal;
  EXPECT_TRUE(robot.plan(TRAJECTORY_NAME, trajectory_goal));
  EXPECT_TRUE(robot.execute(TRAJECTORY_NAME, trajectory_goal));

  // Check if a trajectory without any trajPoints is passed in plan
  moveit_simple::TrajectoryGoal random_goal;
  EXPECT_FALSE(robot.plan("Random Name", random_goal));

  // Check if a trajectory with an empty goal is passed in to execution
  moveit_simple::TrajectoryGoal empty_goal;
  EXPECT_FALSE(robot.execute(TRAJECTORY_NAME, empty_goal));
}

}
