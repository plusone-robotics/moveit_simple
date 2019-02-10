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

#include <moveit_simple/prettyprint.hpp>
#include <ros/ros.h>

#include <moveit_simple/point_types.h>
#include <moveit_simple/robot.h>

namespace moveit_simple
{
void TrajectoryPoint::setJointLockOptions(const JointLockOptions &options)
{
  joint_lock_options_ = options;
}

JointLockOptions TrajectoryPoint::getJointLockOptions()
{
  return joint_lock_options_;
}

std::unique_ptr<JointTrajectoryPoint> JointTrajectoryPoint::toJointTrajPoint(const Robot &robot, double timeout,
                                                                             const std::vector<double> &seed,
                                                                             JointLockOptions options) const
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

std::unique_ptr<JointTrajectoryPoint> CartTrajectoryPoint::toJointTrajPoint(const Robot &robot, double timeout,
                                                                            const std::vector<double> &seed,
                                                                            JointLockOptions options) const
{
  std::vector<double> joints;

  ROS_DEBUG_STREAM("CartTrajectoryPoint: Calculating IK for joint trajectory point");
  if (robot.getJointSolution(pose_, timeout, seed, joints))
  {
    return std::unique_ptr<JointTrajectoryPoint>(new JointTrajectoryPoint(joints, time(), name(), options));
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

std::unique_ptr<JointTrajectoryPoint> CombinedTrajectoryPoint::toJointTrajPoint(const Robot &robot, double timeout,
                                                                                const std::vector<double> &seed,
                                                                                JointLockOptions options) const
{
  std::vector<double> joints;
  std::copy(joint_point_.begin(), joint_point_.end(), std::back_inserter(joints));
  if (compareJointAndCart(robot, timeout))
  {
    return std::unique_ptr<JointTrajectoryPoint>(new JointTrajectoryPoint(joints, time(), name(), options));
  }
  else if(this->type() == PointType::JOINT)
  {
    return std::unique_ptr<JointTrajectoryPoint>(new JointTrajectoryPoint(joints, time(), name(), options));
  }
  else
  {
    auto point = std::unique_ptr<CartTrajectoryPoint>(new CartTrajectoryPoint(pose(), time(), name(), jointLockOptions()));
    return point->toJointTrajPoint(robot, timeout, joints, options);
  }
}

std::unique_ptr<CartTrajectoryPoint> CombinedTrajectoryPoint::toCartTrajPoint(const Robot &robot) const
{
  if(compareJointAndCart(robot, this->timeout()))
  {
    return std::unique_ptr<CartTrajectoryPoint>(new CartTrajectoryPoint(pose(), time(), name(), jointLockOptions()));
  }
  else if(this->type() == PointType::JOINT)
  {
    std::vector<double> joints;
    std::copy(joint_point_.begin(), joint_point_.end(), std::back_inserter(joints));
    auto point = std::unique_ptr<JointTrajectoryPoint>(new JointTrajectoryPoint(joints, time(), name(), jointLockOptions()));
    return point->toCartTrajPoint(robot);
  }
  else
  {
    return std::unique_ptr<CartTrajectoryPoint>(new CartTrajectoryPoint(pose(), time(), name(), jointLockOptions()));
  }
}

std::string CombinedTrajectoryPoint::pointVecToString(const std::vector<double> &vec) const
{
  std::stringstream ss;
  ss << "[ ";
  for_each(vec.begin(), vec.end(), [&ss](const double &point){ss << point << " ";});
  ss << " ]";
  return ss.str();
}

bool CombinedTrajectoryPoint::compareJointAndCart(const Robot &robot, double timeout) const
{
  std::vector<double> cart_point;
  bool in_tol;

  if (robot.getJointSolution(pose_, timeout, joint_point_, cart_point))
  {
    std::vector<double>::const_iterator joint_it = joint_point_.begin();
    std::vector<double>::const_iterator cart_it = cart_point.begin();

    while (in_tol && joint_it != joint_point_.end())
    {
      in_tol = std::abs(*joint_it - *cart_it) <= tol_;
      joint_it++;
      cart_it++;
    }
    if (!in_tol)
    {
      std::stringstream ss;
      ss << "CombinedTrajectoryPoint: Cartesian and Joint representations are out of tolerance. " << std::endl;
      ss << "Using " << ((this->type() == PointType::JOINT) ? "joint" : "cartesian") << " representation, per preference." << std::endl;
      ss << "Joint Representation Joints: " << pointVecToString(joint_point_) << std::endl;
      ss << "Cartesian Representation Joints: " << pointVecToString(cart_point) << std::endl;
      ROS_WARN_STREAM(ss.str());
    }
  }
  else
  {
    ROS_WARN_STREAM("Failed to find joint solution for point: " << name_);
    in_tol = false;
  }

  return in_tol;
}

}  // namespace moveit_simple
