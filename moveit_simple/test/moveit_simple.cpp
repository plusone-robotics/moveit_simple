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
  const Eigen::Affine3d pose = Eigen::Affine3d::Identity(); 

  ROS_INFO_STREAM("Testing reachability of unknown point, should fail");
  EXPECT_FALSE(robot.isReachable("unknown_name"));
  EXPECT_FALSE(robot.isReachable(pose, "random_link"));
  EXPECT_TRUE(robot.isReachable(pose, "tool0"));


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
  const Eigen::Affine3d pose = Eigen::Affine3d::Identity(); 

  ROS_INFO_STREAM("Testing loading of unknown point, should fail");
  EXPECT_THROW(robot.addTrajPoint("bad_traj", "unknown_name", 1.0),std::invalid_argument);
  EXPECT_THROW(robot.addTrajPoint(TRAJECTORY_NAME, pose, "random_link", 5.0, "unknown_name"), tf2::TransformException);

  ros::Duration(2.0).sleep();  //wait for tf tree to populate
  ROS_INFO_STREAM("Testing trajectory adding of points");
  EXPECT_NO_THROW(robot.addTrajPoint(TRAJECTORY_NAME, "home",      0.5));
  EXPECT_NO_THROW(robot.addTrajPoint(TRAJECTORY_NAME, "home",      0.5));
  EXPECT_NO_THROW(robot.addTrajPoint(TRAJECTORY_NAME, "waypoint1", 1.0));
  EXPECT_NO_THROW(robot.addTrajPoint(TRAJECTORY_NAME, "tf_pub1",   2.0));
  EXPECT_NO_THROW(robot.addTrajPoint(TRAJECTORY_NAME, "waypoint2", 3.0));
  EXPECT_NO_THROW(robot.addTrajPoint(TRAJECTORY_NAME, "waypoint3", 4.0));
  EXPECT_NO_THROW(robot.addTrajPoint(TRAJECTORY_NAME, pose, "tool0", 5.0, "random_name"));
  EXPECT_NO_THROW(robot.execute(TRAJECTORY_NAME));

  EXPECT_NO_THROW(robot.addTrajPoint("traj2", "waypoint4", 4.5));
  EXPECT_THROW(robot.execute("traj2"), moveit_simple::IKFailException);

// Not sure if it is a good test for Execution Failure
  EXPECT_NO_THROW(robot.addTrajPoint("traj3", "waypoint1", 1.5));
  EXPECT_NO_THROW(robot.addTrajPoint("traj3", "home", 0.5));
  EXPECT_THROW(robot.execute("traj3"), moveit_simple::ExecutionFailureException);

  EXPECT_THROW(robot.execute("bad_traj"), std::invalid_argument);

}

}
