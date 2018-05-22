/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016 Shaun Edwards
 * Copyright (c) 2018 Plus One Robotics
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

#include <prettyprint/prettyprint.hpp>
#include <ros/ros.h>

#include <moveit_simple/point_types.h>
#include <moveit_simple/robot.h>

namespace moveit_simple
{
std::unique_ptr<JointTrajectoryPoint> JointTrajectoryPoint::toJointTrajPoint(
  const Robot &robot, double timeout, const std::vector<double> &seed) const
{
  ROS_DEBUG_STREAM("JointTrajectoryPoint: passing through joint trajectory point");
  return std::unique_ptr<JointTrajectoryPoint>(new JointTrajectoryPoint(*this));
}

std::unique_ptr<CartTrajectoryPoint> JointTrajectoryPoint::toCartTrajPoint(const Robot &robot) const
{
  Eigen::Affine3d pose;

  ROS_DEBUG_STREAM("JointTrajectoryPoint: Calculating FK for Cartesian trajectory point");
  if (robot.getPose(joint_point_, pose))
  {
    return std::unique_ptr<CartTrajectoryPoint>(new CartTrajectoryPoint(pose, time(), name()));
  }
  else
  {
    ROS_WARN_STREAM("Failed to find FK for point: " << name_);
    return std::unique_ptr<CartTrajectoryPoint>(nullptr);
  }
}

std::unique_ptr<JointTrajectoryPoint> CartTrajectoryPoint::toJointTrajPoint(
  const Robot &robot, double timeout, const std::vector<double> &seed) const
{
  std::vector<double> joints;

  ROS_DEBUG_STREAM("CartTrajectoryPoint: Calculating IK for joint trajectory point");
  if (robot.getJointSolution(pose_, timeout, seed, joints))
  {
    return std::unique_ptr<JointTrajectoryPoint>(new JointTrajectoryPoint(joints, time(), name()));
  }
  else
  {
    ROS_WARN_STREAM("Failed to find joint solution for point: " << name_);
    return std::unique_ptr<JointTrajectoryPoint>(nullptr);
  }
}

std::unique_ptr<CartTrajectoryPoint> CartTrajectoryPoint::toCartTrajPoint(const Robot &robot) const
{
  ROS_DEBUG_STREAM("CartTrajectoryPoint: passing through cartesian trajectory point");
  return std::unique_ptr<CartTrajectoryPoint>(new CartTrajectoryPoint(*this));
}
} // namespace moveit_simple