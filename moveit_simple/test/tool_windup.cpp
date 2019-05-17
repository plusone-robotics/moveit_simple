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
  // Test enable and disable joint windup limitation. (this should end with an enabled flag)
  robot.setLimitJointWindup(false);
  EXPECT_FALSE(robot.getLimitJointWindup());
  robot.setLimitJointWindup(true);
  EXPECT_TRUE(robot.getLimitJointWindup());

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
