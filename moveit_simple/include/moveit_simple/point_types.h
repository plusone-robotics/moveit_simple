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

#include <eigen3/Eigen/Geometry>

namespace moveit_simple
{
class Robot;
class TrajectoryPoint;
class JointTrajectoryPoint;
class CartTrajectoryPoint;

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

protected:
  enum PointType
  {
    UNKNOWN = 0,
    JOINT = 1,
    CARTESIAN = 2
  };

  TrajectoryPoint(std::string name, double t, PointType type) : name_(name), t_(t), type_(type) { }

  const PointType &type() const
  {
    return type_;
  }

  virtual std::unique_ptr<JointTrajectoryPoint> toJointTrajPoint(const Robot &robot,
    double timeout, const std::vector<double> &seed) const = 0;

  virtual std::unique_ptr<CartTrajectoryPoint> toCartTrajPoint(const Robot &robot) const = 0;

  double t_;
  std::string name_;
  PointType type_;
};

class JointTrajectoryPoint : public TrajectoryPoint
{
public:
  JointTrajectoryPoint() : TrajectoryPoint("", 0.0, PointType::JOINT) { }
  
  virtual ~JointTrajectoryPoint() { }

  JointTrajectoryPoint(std::vector<double> &joint_point, double t, std::string name = std::string())
    : TrajectoryPoint(name, t, PointType::JOINT), joint_point_(joint_point) { }

  const std::vector<double> &jointPoint() const
  {
    return joint_point_;
  }

protected:
  virtual std::unique_ptr<JointTrajectoryPoint> toJointTrajPoint(const Robot &robot,
    double timeout, const std::vector<double> &seed) const;

  virtual std::unique_ptr<CartTrajectoryPoint> toCartTrajPoint(const Robot &robot) const;

private:
  std::vector<double> joint_point_;
};

class CartTrajectoryPoint : public TrajectoryPoint
{
public:
  CartTrajectoryPoint() : TrajectoryPoint("", 0.0, PointType::CARTESIAN) { }

  CartTrajectoryPoint(const Eigen::Affine3d pose, const double t, std::string name = std::string())
    : TrajectoryPoint(name, t, PointType::CARTESIAN), pose_(pose) { }

  virtual ~CartTrajectoryPoint() { }

  const Eigen::Affine3d &pose() const
  {
    return pose_;
  }

protected:
  virtual std::unique_ptr<JointTrajectoryPoint> toJointTrajPoint(const Robot &robot,
    double timeout, const std::vector<double> &seed) const;

  virtual std::unique_ptr<CartTrajectoryPoint> toCartTrajPoint(const Robot &robot) const;

private:
  Eigen::Affine3d pose_;
};
} // namespace moveit_simple
#endif // POINT_TYPES_H