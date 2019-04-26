/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2019 Plus One Robotics
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

#include <eigen_conversions/eigen_msg.h>
#include <gtest/gtest.h>
#include <moveit_simple/moveit_simple.h>
#include <moveit_simple/prettyprint.hpp>
#include <random_numbers/random_numbers.h>

using testing::Types;

namespace moveit_simple_test
{

/**
 * @brief UserRobotTest is a fixture for testing public methods.
 * Objects that can be directly used inside the test
 - robot pointer for moveit_simple::OnlineRobot
*/
class UserRobotTest : public ::testing::Test
{
protected:
  std::unique_ptr<moveit_simple::OnlineRobot> robot;
  virtual void SetUp()
  {
  robot = std::unique_ptr<moveit_simple::OnlineRobot> (new moveit_simple::OnlineRobot
                  (ros::NodeHandle(), "robot_description", "manipulator"));
  ros::Duration(2.0).sleep();  //wait for tf tree to populate
  }
  virtual void TearDown()
  {}
};

/**
 * @brief DeveloperRobot is a class inheriting from moveit_simple::OnlineRobot to test protected methods
 * Add the protected methods(that are required to be tested) in the following list
*/
class DeveloperRobot: public moveit_simple::OnlineRobot
{
public:
  using moveit_simple::OnlineRobot::OnlineRobot;
  using moveit_simple::OnlineRobot::addTrajPoint;
  using moveit_simple::OnlineRobot::toJointTrajectory;
  using moveit_simple::OnlineRobot::interpolate;
  using moveit_simple::OnlineRobot::jointInterpolation;
  using moveit_simple::OnlineRobot::cartesianInterpolation;
  using moveit_simple::OnlineRobot::isInCollision;
};

/**
 * @brief DeveloperRobotTest is a fixture for testing protected methods.
 * Objects that can be directly used inside the test
 - robot pointer for DeveloperRobot
*/

class DeveloperRobotTest : public ::testing::Test
{
protected:
  std::unique_ptr<DeveloperRobot> robot2;
  virtual void SetUp()
  {
  robot2 = std::unique_ptr<DeveloperRobot> (new DeveloperRobot
                  (ros::NodeHandle(), "robot_description", "manipulator"));
  ros::Duration(2.0).sleep();  //wait for tf tree to populate
  }
  virtual void TearDown()
  {}
};

TEST(MoveitSimpleTest, construction_robot)
{
  moveit_simple::Robot robot(ros::NodeHandle(), "robot_description", "manipulator");
}

TEST(MoveitSimpleTest, test_limit_ik_windup)
{
  ROS_INFO_STREAM("Testing IK solution of waypoint");
  // initialize robot
  moveit_simple::Robot robot(ros::NodeHandle(), "robot_description", "manipulator");

  // run stepwise rotations and provoke windup
  Eigen::Isometry3d pose(Eigen::Translation3d(Eigen::Vector3d(1.904, 0.000, 2.762)));
  std::vector<double> joints(6,0);
  double angle_step = 0.05;
  size_t steps = 1.5 * M_PI / angle_step + 1;
  for (size_t i = 0; i < steps; ++i)
  {
    pose.rotate(Eigen::AngleAxisd(angle_step, Eigen::Vector3d::UnitX()));
    robot.getJointSolution(pose, "tool0", 0.01, joints, joints);
  }
  EXPECT_TRUE(std::abs(joints.back()) > 1.5 * M_PI);

  // test getter and setter functions for ik_seed_state_fractions_
  std::map<size_t, double> seed_fractions;
  seed_fractions[6] = 0.1;  // invalid joint position
  EXPECT_FALSE(robot.setIKSeedStateFractions(seed_fractions));
  seed_fractions.clear();
  seed_fractions[5] = 1.1;  // invalid fraction value
  EXPECT_FALSE(robot.setIKSeedStateFractions(seed_fractions));
  seed_fractions.clear();
  // set ik seed state fractions to 'pull' wrist joint towards 0
  seed_fractions[5] = 0.3;
  EXPECT_TRUE(robot.setIKSeedStateFractions(seed_fractions));
  EXPECT_TRUE(seed_fractions == robot.getIKSeedStateFractions());
  // enable joint windup limitation
  robot.limitJointWindup(true);

  // run same rotations as before and verify that windup is disabled
  pose.linear().setIdentity();
  for (size_t i = 0; i < steps; ++i)
  {
    pose.rotate(Eigen::AngleAxisd(angle_step, Eigen::Vector3d::UnitX()));
    robot.getJointSolution(pose, "tool0", 0.01, joints, joints);
  }
  EXPECT_FALSE(std::abs(joints.back()) > 1.5 * M_PI);

  // stress test with 1000 consecutive random orientations while using the previous solution seed state
  // this shouldn't produce any windup in the wrist joint at all (abs(value) < pi)
  random_numbers::RandomNumberGenerator rand(7 /* seed */);
  steps = 1000;
  size_t failed_ik_count = 0;
  size_t failed_windup_count = 0;
  for (size_t i = 0; i < steps; ++i)
  {
    pose.rotate(Eigen::AngleAxisd(rand.uniformReal(-M_PI, M_PI), Eigen::Vector3d::UnitX()));
    if (!robot.getJointSolution(pose, "tool0", 0.05, joints, joints))
      ++failed_ik_count;
    else if (std::abs(joints.back() > 1.5 * M_PI))
      ++failed_windup_count;
  }
  EXPECT_TRUE(failed_ik_count == 0);
  EXPECT_TRUE(failed_windup_count == 0);
}
}
