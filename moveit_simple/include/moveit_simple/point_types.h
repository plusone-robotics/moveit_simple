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

#ifndef POINT_TYPES_H
#define POINT_TYPES_H

#include <memory>
#include <string>
#include <vector>

#include <moveit_simple/joint_lock_options.h>
#include <eigen3/Eigen/Geometry>

namespace moveit_simple
{
class Robot;
class TrajectoryPoint;
class JointTrajectoryPoint;
class CartTrajectoryPoint;
class CombinedTrajectoryPoint;

namespace interpolation_type
{
  enum InterpolationType
  {
    UNKNOWN = 0,
    JOINT = 1,
    CARTESIAN = 2
  };
} // namespace interpolation_type

typedef interpolation_type::InterpolationType InterpolationType;

struct TrajectoryPointInfo
{
  std::unique_ptr<TrajectoryPoint> point;
  InterpolationType type;
  unsigned int num_steps;
};

typedef std::vector<TrajectoryPointInfo> TrajectoryInfo;

class TrajectoryPoint
{
public:
  friend class Robot;

  virtual ~TrajectoryPoint() { }

  const double time() const
  {
    return t_;
  }

  const std::string &name() const
  {
    return name_;
  }

  void setJointLockOptions(const JointLockOptions &options);

  JointLockOptions getJointLockOptions();

protected:
  enum PointType
  {
    UNKNOWN = 0,
    JOINT = 1,
    CARTESIAN = 2
  };

  TrajectoryPoint(std::string name, double t, PointType type, JointLockOptions options = JointLockOptions::LOCK_NONE)
    : name_(name), t_(t), type_(type), joint_lock_options_(options) { }

  const PointType &type() const
  {
    return type_;
  }

  virtual std::unique_ptr<JointTrajectoryPoint> toJointTrajPoint(const Robot &robot,
    double timeout, const std::vector<double> &seed, JointLockOptions options = JointLockOptions::LOCK_NONE) const = 0;

  virtual std::unique_ptr<CartTrajectoryPoint> toCartTrajPoint(const Robot &robot) const = 0;

  double t_;
  std::string name_;
  PointType type_;
  JointLockOptions joint_lock_options_ = JointLockOptions::LOCK_NONE;
};




class JointTrajectoryPoint : public TrajectoryPoint
{
public:
  friend class CombinedTrajectoryPoint;
  JointTrajectoryPoint() : TrajectoryPoint("", 0.0, PointType::JOINT) { }

  virtual ~JointTrajectoryPoint() { }

  JointTrajectoryPoint(std::vector<double> &joint_point, double t, std::string name = std::string(),
                       JointLockOptions options = JointLockOptions::LOCK_NONE)
                       : TrajectoryPoint(name, t, PointType::JOINT, options), joint_point_(joint_point) { }

  const std::vector<double> &jointPoint() const
  {
    return joint_point_;
  }

protected:
  virtual std::unique_ptr<JointTrajectoryPoint> toJointTrajPoint(const Robot &robot,
    double timeout, const std::vector<double> &seed, JointLockOptions options = JointLockOptions::LOCK_NONE) const;

  virtual std::unique_ptr<CartTrajectoryPoint> toCartTrajPoint(const Robot &robot) const;

private:
  std::vector<double> joint_point_;
};

class CartTrajectoryPoint : public TrajectoryPoint
{
public:
  friend class CombinedTrajectoryPoint;
  CartTrajectoryPoint() : TrajectoryPoint("", 0.0, PointType::CARTESIAN) { }

  CartTrajectoryPoint(const Eigen::Affine3d pose, const double t, std::string name = std::string(),
                      JointLockOptions options = JointLockOptions::LOCK_NONE)
    : TrajectoryPoint(name, t, PointType::CARTESIAN, options), pose_(pose) { }

  virtual ~CartTrajectoryPoint() { }

  const Eigen::Affine3d &pose() const
  {
    return pose_;
  }

protected:
  virtual std::unique_ptr<JointTrajectoryPoint> toJointTrajPoint(const Robot &robot,
    double timeout, const std::vector<double> &seed, JointLockOptions options = JointLockOptions::LOCK_NONE) const;

  virtual std::unique_ptr<CartTrajectoryPoint> toCartTrajPoint(const Robot &robot) const;

private:
  Eigen::Affine3d pose_;
};

class CombinedTrajectoryPoint : public TrajectoryPoint
{
public:
  enum PointPreference
  {
    CARTESIAN = 0,
    JOINT = 1
  };

  CombinedTrajectoryPoint() : TrajectoryPoint("", 0.0, PointType::JOINT)
  {
  }

  CombinedTrajectoryPoint(std::vector<double> &joint_point, const Eigen::Affine3d pose, double t, double tol,
                          std::string name = std::string(), PointPreference type = PointPreference::JOINT,
                          JointLockOptions options = JointLockOptions::LOCK_NONE, double timeout = 5.0)
    : TrajectoryPoint(name, t, (type == PointPreference::JOINT) ? PointType::JOINT : PointType::CARTESIAN, options)
    , options_(options)
    , joint_point_(joint_point)
    , pose_(pose)
    , tol_(tol)
  {
  }

  virtual ~CombinedTrajectoryPoint() { }

  const JointLockOptions &jointLockOptions() const
  {
    return options_;
  }

  const Eigen::Affine3d &pose() const
  {
    return pose_;
  }

  const std::vector<double> &jointPoint() const
  {
    return joint_point_;
  }

  const double &timeout() const
  {
    return timeout_;
  }

  std::string pointVecToString(const std::vector<double> &vec) const;

protected:

  virtual std::unique_ptr<JointTrajectoryPoint>
  toJointTrajPoint(const Robot &robot, double timeout, const std::vector<double> &seed,
                   JointLockOptions options = JointLockOptions::LOCK_NONE) const;

  virtual std::unique_ptr<CartTrajectoryPoint> toCartTrajPoint(const Robot &robot) const;

  bool compareJointAndCart(const Robot &robot, double timeout) const;

private:

  JointLockOptions options_;
  std::vector<double> joint_point_;
  Eigen::Affine3d pose_;
  double tol_;
  double timeout_;
};
} // namespace moveit_simple
#endif // POINT_TYPES_H
