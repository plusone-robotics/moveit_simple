/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser
   Desc: Unit test cases for verifying tool windup limitation is functional
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
  std::unique_ptr<moveit_simple::OnlineRobot> robot_;
  virtual void SetUp()
  {
    robot_ = std::unique_ptr<moveit_simple::OnlineRobot> (new moveit_simple::OnlineRobot
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
  using moveit_simple::OnlineRobot::getSymmetricIK;
};

/**
 * @brief DeveloperRobotTest is a fixture for testing protected methods.
 * Objects that can be directly used inside the test
 - robot pointer for DeveloperRobot
*/

class DeveloperRobotTest : public ::testing::Test
{
protected:
  std::unique_ptr<DeveloperRobot> robot_;
  virtual void SetUp()
  {
    robot_ = std::unique_ptr<DeveloperRobot> (new DeveloperRobot
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

TEST_F(UserRobotTest, test_limit_ik_windup)
{
  ROS_INFO_STREAM("Testing IK solution of waypoint");

  // run stepwise rotations and provoke windup
  Eigen::Isometry3d pose(Eigen::Translation3d(Eigen::Vector3d(2, 0, 2.5)));
  std::vector<double> joints(6,0);
  double angle_step = 0.05;
  size_t steps = 1.5 * M_PI / angle_step + 1;
  for (size_t i = 0; i < steps; ++i)
  {
    pose.rotate(Eigen::AngleAxisd(angle_step, Eigen::Vector3d::UnitX()));
    robot_->getJointSolution(pose, "tool0", 0.01, joints, joints);
  }
  EXPECT_TRUE(std::abs(joints.back()) > 1.5 * M_PI);

  // test getter and setter functions for ik_seed_state_fractions_
  std::map<size_t, double> seed_fractions;
  seed_fractions[6] = 0.1;  // invalid joint position
  EXPECT_FALSE(robot_->setIKSeedStateFractions(seed_fractions));
  seed_fractions.clear();
  seed_fractions[5] = 1.1;  // invalid fraction value
  EXPECT_FALSE(robot_->setIKSeedStateFractions(seed_fractions));
  seed_fractions.clear();
  // set ik seed state fractions to 'pull' wrist joint towards 0
  seed_fractions[5] = 0.3;
  EXPECT_TRUE(robot_->setIKSeedStateFractions(seed_fractions));
  EXPECT_TRUE(seed_fractions == robot_->getIKSeedStateFractions());

  // Test enable and disable joint windup limitation. (this should end with an enabled flag)
  robot_->setLimitJointWindup(false);
  EXPECT_FALSE(robot_->getLimitJointWindup());
  robot_->setLimitJointWindup(true);
  EXPECT_TRUE(robot_->getLimitJointWindup());

  // run same rotations as before and verify that windup is disabled
  pose.linear().setIdentity();
  for (size_t i = 0; i < steps; ++i)
  {
    pose.rotate(Eigen::AngleAxisd(angle_step, Eigen::Vector3d::UnitX()));
    robot_->getJointSolution(pose, "tool0", 0.01, joints, joints);
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
    if (!robot_->getJointSolution(pose, "tool0", 0.05, joints, joints))
      ++failed_ik_count;
    else if (std::abs(joints.back() > 1.5 * M_PI))
      ++failed_windup_count;
  }
  EXPECT_TRUE(failed_ik_count == 0);
  EXPECT_TRUE(failed_windup_count == 0);
}

TEST_F(DeveloperRobotTest, test_symmetric_ik)
{
  // init target pose and a target state IK solution
  Eigen::Isometry3d pose(Eigen::Translation3d(Eigen::Vector3d(2, 0, 2.5)));
  std::vector<double> joint_state = robot_->getJointState();
  ASSERT_TRUE(robot_->getJointSolution(pose, 0.1, joint_state, joint_state));
  std::vector<double> seed = joint_state;  // init target state as seed

  // test free rotation around z (circular end effector)
  robot_->setEndEffectorSymmetry(moveit_simple::EndEffectorSymmetry::Circular);
  double distance_threshold = 0.05; // total joint value distance shouldn't exceed this value
  double max_distance = 0.0;
  constexpr size_t steps = 10;
  for (size_t i = 0; i < steps; ++i)
  {
    pose.rotate(Eigen::AngleAxisd(2 * M_PI / steps, Eigen::Vector3d::UnitX()));
    ASSERT_TRUE(robot_->getJointSolution(pose, 0.1, seed, joint_state, true));
    double distance = 0;
    for (size_t j = 0; j < seed.size(); ++j)
      distance += std::abs(joint_state[j] - seed[j]);
    if (distance > max_distance)
      max_distance = distance;
  }
  EXPECT_TRUE(max_distance < distance_threshold);

  // test 1 symmetry around z (rectangular end effector)
  robot_->setEndEffectorSymmetry(moveit_simple::EndEffectorSymmetry::Rectangular);
  distance_threshold = M_PI / 2; // total joint value distance shouldn't exceed this value
  max_distance = 0.0;
  for (size_t i = 0; i < steps; ++i)
  {
    pose.rotate(Eigen::AngleAxisd(2 * M_PI / steps, Eigen::Vector3d::UnitX()));
    ASSERT_TRUE(robot_->getJointSolution(pose, 0.1, seed, joint_state, true));
    double distance = 0;
    for (size_t j = 0; j < seed.size(); ++j)
      distance += std::abs(joint_state[j] - seed[j]);
    if (distance > max_distance)
      max_distance = distance;
  }
  EXPECT_TRUE(max_distance < distance_threshold);

  // test 2 symmetries around z (square end effector)
  robot_->setEndEffectorSymmetry(moveit_simple::EndEffectorSymmetry::Quadratic);
  distance_threshold =  M_PI / 4; // total joint value distance shouldn't exceed this value
  max_distance = 0.0;
  for (size_t i = 0; i < steps; ++i)
  {
    pose.rotate(Eigen::AngleAxisd(2 * M_PI / steps, Eigen::Vector3d::UnitX()));
    ASSERT_TRUE(robot_->getJointSolution(pose, 0.1, seed, joint_state, true));
    double distance = 0;
    for (size_t j = 0; j < seed.size(); ++j)
      distance += std::abs(joint_state[j] - seed[j]);
    if (distance > max_distance)
      max_distance = distance;
  }
  EXPECT_TRUE(max_distance < distance_threshold);

  // Test symmetric Ik in combination with joint windup reduction
  // The last wrist joint is set to 2*MPI while the seed fraction is set to 0.
  // Random target pose orientations should generate IK solutions that are below
  // the orientation threshold of the corresponding symmetry.
  std::map<size_t, double> seed_fractions;
  seed_fractions[5] = 0.0; // strongly 'pull' wrist joint towards 0
  EXPECT_TRUE(robot_->setIKSeedStateFractions(seed_fractions));
  robot_->setLimitJointWindup(true);
  random_numbers::RandomNumberGenerator rand(7 /* seed */);
  // wrist joint values should be below this threshold since symmetry allows it
  double orientation_threshold = M_PI / 4;
  seed[5] = 2 * M_PI;  // seed wrist joint value is 2 * M_PI
  for (size_t i = 0; i < 1000; ++i)
  {
    pose.rotate(Eigen::AngleAxisd(rand.uniformReal(-M_PI, M_PI), Eigen::Vector3d::UnitX()));
    ASSERT_TRUE(robot_->getJointSolution(pose, 0.1, seed, joint_state, true));
    ASSERT_TRUE(std::abs(joint_state[5]) < orientation_threshold);
  }
}
}
