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


#include <moveit_simple/trajectory_planner.h>
#include <moveit_simple/conversions.h>
#include <moveit_simple/robot.h>
#include <moveit_simple/joint_locker.h>

namespace moveit_simple {

void TrajectoryPlanner::addTrajPoint(const std::string& traj_name, std::unique_ptr<TrajectoryPoint>& point,
                         const InterpolationType& type, const unsigned int num_steps)
{
  std::lock_guard<std::recursive_mutex> guard(trajectory_info_map_mutex_);
  traj_info_map_[traj_name].push_back({ std::move(point), type, num_steps });
}

void TrajectoryPlanner::clearTrajectory(const std::string& traj_name)
{
  std::lock_guard<std::recursive_mutex> guard(trajectory_info_map_mutex_);
  traj_info_map_.erase(traj_name);
}

std::vector<moveit_simple::JointTrajectoryPoint> TrajectoryPlanner::plan(Robot& robot, const std::string traj_name, bool collision_check)
{
  std::vector<moveit_simple::JointTrajectoryPoint> plan;

  if (traj_info_map_.count(traj_name))
  {
    std::vector<trajectory_msgs::JointTrajectoryPoint> ROS_trajectory_points;

    try
    {
      if (toJointTrajectory(robot, traj_name, ROS_trajectory_points, collision_check))
      {
        // Modify the speed of execution for the trajectory based off of the speed_modifier
        for (std::size_t i = 0; i < ROS_trajectory_points.size(); i++)
        {
          ROS_trajectory_points[i].time_from_start *= (1.0 / robot.getSpeedModifier());
        }

        std::vector<moveit_simple::JointTrajectoryPoint> goal = toJointTrajectoryPoint(ROS_trajectory_points);

        ROS_INFO_STREAM("Successfully planned out trajectory: [" << traj_name << "]");

        return goal;
      }
      else
      {
        ROS_ERROR_STREAM("Failed to convert [" << traj_name << "] to joint trajectory");
        throw IKFailException("Conversion to joint trajectory failed for " + traj_name);
      }
    }
    catch (CollisionDetected& cd)
    {
      ROS_ERROR_STREAM("Collision detected in [" << traj_name << "]");
      throw cd;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Trajectory [" << traj_name << "] not found");
    throw std::invalid_argument("No trajectory found named " + traj_name);
  }
 return plan;
}

bool TrajectoryPlanner::toJointTrajectory(Robot& robot, const std::string traj_name,
                                          std::vector<trajectory_msgs::JointTrajectoryPoint>& points,
                                          bool collision_check)
{
  const TrajectoryInfo& traj_info = traj_info_map_[traj_name];

  // The first point in any trajectory is the current pose
  std::vector<double> current_joint_position = robot.getJointState();
  points.clear();
  points.push_back(toJointTrajPtMsg(current_joint_position, 0.0));

  for (size_t i = 0; i < traj_info.size(); ++i)
  {
    const std::unique_ptr<TrajectoryPoint>& traj_point = traj_info[i].point;
    if (traj_info[i].type == interpolation_type::JOINT)
    {
      if (jointInterpolation(robot, traj_point, points, traj_info[i].num_steps, collision_check))
      {
        ROS_INFO_STREAM("Trajectory successfully added till " << traj_point->name());
      }
      else
      {
        ROS_ERROR_STREAM("Conversion to joint trajectory failed for " << traj_name << " due to IK failure of "
                                                                      << traj_point->name());
        return false;
      }
    }
    else if (traj_info[i].type == interpolation_type::CARTESIAN)
    {
      if (robot.cartesianInterpolation(traj_point, points, traj_info[i].num_steps, collision_check))
      {
        ROS_INFO_STREAM("Trajectory successfully added till " << traj_point->name());
      }
      else
      {
        ROS_ERROR_STREAM("Conversion to joint trajectory failed for " << traj_name << " before adding "
                                                                      << traj_point->name());
        return false;
      }
    }
    else
    {
      ROS_ERROR_STREAM("Unknown interpolation call " << traj_info[i].type);
      return false;
    }
  }

  return true;
}

bool TrajectoryPlanner::jointInterpolation(Robot& robot, const std::unique_ptr<TrajectoryPoint>& traj_point,
                               std::vector<trajectory_msgs::JointTrajectoryPoint>& points, unsigned int num_steps,
                               bool collision_check)
{
  const double IK_TIMEOUT = 0.250;  // 250 ms for IK solving

  auto options = traj_point->getJointLockOptions();

  // Check to make sure the points vector is not empty. At minimum, it should plan from the current state.
  if (!points.size())
  {
    ROS_ERROR_STREAM("jointInterpolation was given an invalid value for `points`. This vector may not be empty");
    return false;
  }

  // Get the last point in the points trajectory to connect it to this new trajectory
  trajectory_msgs::JointTrajectoryPoint prev_point_info = points.back();
  std::vector<double> prev_point = prev_point_info.positions;
  double prev_time = prev_point_info.time_from_start.toSec();

  // Convert the previous point stored in points to Joint Trajectory Point
  std::unique_ptr<JointTrajectoryPoint> prev_traj_point =
      std::unique_ptr<JointTrajectoryPoint>(new JointTrajectoryPoint(prev_point, prev_time, ""));

  std::unique_ptr<JointTrajectoryPoint> target_point;
  if (traj_point->type() != TrajectoryPoint::JOINT)
  {
    const size_t MAX_IK_ATTEMPTS = 2;
    size_t num_attempts = 0;

    while (true)
    {
      num_attempts++;
      target_point = traj_point->toJointTrajPoint(robot, IK_TIMEOUT, prev_point);

      if (target_point)
      {
        if (robot.isConfigChange(prev_point, target_point->jointPoint()))
        {
          ROS_WARN_STREAM("Configuration change detected in move to/from joint state: ("
                          << traj_point->name() << "), of type: " << traj_point->type() << " to joint trajectory");
        }
        else
        {
          ROS_INFO_STREAM("Found proper IK (no config change) in " << num_attempts << " attempts");
          break;
        }
      }
      else
      {
        ROS_WARN_STREAM("Failed to convert trajectory point:  ("
                        << traj_point->name() << "), of type: " << traj_point->type() << " to joint trajectory");
        return false;
      }
      if (num_attempts >= MAX_IK_ATTEMPTS)
      {
        ROS_ERROR_STREAM("Failed to find proper IK (no config change) in " << num_attempts << " attempts");
        return false;
      }
    }
  }
  else
  {
    target_point = traj_point->toJointTrajPoint(robot, IK_TIMEOUT, prev_point);
    if (!target_point)
    {
      ROS_WARN_STREAM("Failed to convert joint point - this shouldn't happen");
      return false;
    }
  }

  // Create a local vector for storing interpolated points
  std::vector<trajectory_msgs::JointTrajectoryPoint> points_local;
  unsigned int points_added = 0;
  // TODO (mlautman): Explain why we are adding num_steps + 1 instead of just num_steps.
  for (std::size_t i = 1; i <= num_steps + 1; ++i)
  {
    double t = (double)i / (double)(num_steps + 1);
    std::unique_ptr<JointTrajectoryPoint> new_point;
    robot.interpolate(prev_traj_point, target_point, t, new_point);
    if (new_point)
    {
      if ((collision_check) && (robot.isInCollision(new_point->jointPoint())))
      {
        ROS_WARN_STREAM("Collision detected at " << points_added << " among " << (num_steps + 1) << " points for "
                                                 << traj_point->name());
        // TODO (mlautman): preserve points_local for debugging
        points_local.clear();
        throw CollisionDetected("Collision detected while interpolating " + traj_point->name());
      }
      else
      {
        // Lock the joints
        auto new_point_joints = new_point->jointPoint();
        JointLocker::lockJoints(prev_point, new_point_joints, options);
        auto locked_new_point =
            std::unique_ptr<JointTrajectoryPoint>(new JointTrajectoryPoint(new_point_joints, new_point->time(), ""));

        points_local.push_back(toJointTrajPtMsg(*locked_new_point));
        points_added++;
        ROS_INFO_STREAM(points_added << " points among " << (num_steps + 1) << " successfully interpolated for "
                                     << traj_point->name());
      }
    }
    else
    {
      ROS_WARN_STREAM("Conversion to joint trajectory failed at " << points_added << " among " << (num_steps + 1)
                                                                  << "points for " << traj_point->name()
                                                                  << ". Exiting without interpolation");
      points_local.clear();
      return false;
    }
  }
  // Append the global points with local_points
  points.insert(points.end(), points_local.begin(), points_local.end());
  ROS_INFO_STREAM("Appending trajectory point, size: " << points.size());
  points_local.clear();
  return true;
}

bool TrajectoryPlanner::cartesianInterpolation(Robot& robot,
                                   const std::unique_ptr<TrajectoryPoint>& traj_point,
                                   std::vector<trajectory_msgs::JointTrajectoryPoint>& points, unsigned int num_steps,
                                   bool collision_check)
{
  // Create a local vector for storing interpolated points and append
  // it with last element of global points to be used for interpolation
  std::vector<trajectory_msgs::JointTrajectoryPoint> points_local;
  points_local.clear();
  points_local.push_back(points.back());

  trajectory_msgs::JointTrajectoryPoint prev_point_info = points.back();

  // Convert the previous point stored in points to Cartesian Trajectory Point
  std::unique_ptr<TrajectoryPoint> prev_point = std::unique_ptr<TrajectoryPoint>(
      new JointTrajectoryPoint(prev_point_info.positions, prev_point_info.time_from_start.toSec(), ""));
  std::unique_ptr<CartTrajectoryPoint> prev_traj_point = prev_point->toCartTrajPoint(robot);
  std::unique_ptr<CartTrajectoryPoint> target_point;

  if (prev_traj_point)
  {
    target_point = traj_point->toCartTrajPoint(robot);
    if (!target_point)
    {
      ROS_ERROR_STREAM("Failed to find FK for " << traj_point->name());
      return false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Failed to find FK for already added point. This is unexpected.");
    return false;
  }

  unsigned int points_added = 0;
  for (std::size_t i = 1; i <= num_steps + 1; ++i)
  {
    double t = (double)i / (double)(num_steps + 1);
    std::unique_ptr<CartTrajectoryPoint> new_point_cart;
    robot.interpolate(prev_traj_point, target_point, t, new_point_cart);

    std::unique_ptr<TrajectoryPoint> new_point =
        std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(new_point_cart->pose(), new_point_cart->time(), ""));

    // Add new point at the end of Joint Trajectory (named points_local)
    if (jointInterpolation(robot, new_point, points_local, (unsigned int)0, collision_check))
    {
      points_added++;
      ROS_INFO_STREAM(points_added << " points among " << (num_steps + 1) << " added successfullyfor "
                                   << traj_point->name());
    }
    else
    {
      ROS_WARN_STREAM("Conversion to joint trajectory failed at " << points_added << " among " << (num_steps + 1)
                                                                  << "points for " << traj_point->name()
                                                                  << ". Exiting without interpolation");
      points_local.clear();
      return false;
    }
  }

  // Append the global points with local_points
  points.insert(points.end(), points_local.begin() + 1, points_local.end());
  ROS_INFO_STREAM("Appending trajectory point, size: " << points.size());
  points_local.clear();
  return true;
}


} // namespace moveit_simple
