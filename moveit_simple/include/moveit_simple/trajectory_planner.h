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

#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <mutex>

#include <ros/ros.h>

#include <moveit_simple/point_types.h>
#include <moveit_simple/exceptions.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>


namespace moveit_simple {
class Robot;

/**
 * @brief TrajectoryPlanner manages the generation, lookup and planning of trajectories.
 * Assumptions are:
 *  - single arm manipulator (one joint group)
 *  - all cartesian poses are of the tool of the robot (could be fixed in the future)
 *  - point names are stored as joint positions in the SRDF or frame names in the
 * URDF
 * - frame transformations outside the URDF are provided by TF
 */

class TrajectoryPlanner {
public:
  /**
  * @brief Constructor
  */
  TrajectoryPlanner()
  {
  }

  /**
  * @brief Destructor
  */
  ~TrajectoryPlanner(){}

  void addTrajPoint(const std::string &traj_name, std::unique_ptr<TrajectoryPoint> &point,
    const InterpolationType &type = interpolation_type::JOINT,
    const unsigned int num_steps = 0);

  /**
   * @brief clearTrajectory - clears stored trajectory
   * @param traj_name - trajectory to clear
   */
  void clearTrajectory(const ::std::string& traj_name);

  /**
   * @brief plan out a given trajectory
   * @param robot - Robot to be used for planning
   * @param traj_name - name of trajectory to be executed (must be filled with
   * prior calls to "addTrajPoint".
   * @param collision_check - bool to turn check for collision on\off
   * @throws <moveit_simple::IKFailException> (Conversion to joint trajectory failed)
   * @throws <std::invalid_argument> (Trajectory "traj_name" not found)
   * @throws <moveit_simple::CollisionDetected> (One of interpolated point is
   * in Collision with itself or environment)
   */
  std::vector<moveit_simple::JointTrajectoryPoint> plan(
    Robot& robot,
    const std::string traj_name, bool collision_check = false);

  /**
   * @brief  jointInterpolation - joint Interpolation from last added point to
   * current trajectory point(traj_point).
   * @param robot - Robot to be used for planning
   * @param traj_point: target traj_point for joint interpolation
   * @param points: Vector of Joint Trajectory Point to be executed. We append new points in place
   *                This Vector must not be empty and it should have the current pose at index 0.
   * @param num_steps: number of steps to be interpolated between current point and traj_point
   * @param collision_check - bool to turn check for collision on\off
   * @return true if all the points including traj_point are added to the points.
   */
  bool jointInterpolation(
    Robot& robot,
    const std::unique_ptr<TrajectoryPoint> &traj_point,
    std::vector<trajectory_msgs::JointTrajectoryPoint> &points, const unsigned int num_steps,
    bool collision_check = false);

  /**
   * @brief  cartesianInterpolation - Cartesian Interpolation from last added point to
   * current trajectory point(traj_point).
   * @param robot - Robot to be used for planning
   * @param traj_point: target traj_point for cartesian interpolation
   * @param points: Vector of Joint Trajectory Point to be executed
   * @param num_steps: number of steps to be interpolated between current point and traj_point
   * @param collision_check - bool to turn check for collision on\off
   * @return true if all the points including traj_point are added to the points.
   */
  bool cartesianInterpolation(Robot& robot,
    const std::unique_ptr<TrajectoryPoint> &traj_point,
    std::vector<trajectory_msgs::JointTrajectoryPoint> &points, const unsigned int num_steps,
    bool collision_check = false);


  bool toJointTrajectory(
    Robot& robot,
    const std::string traj_name,
    std::vector<trajectory_msgs::JointTrajectoryPoint> &points,
    bool collision_check = false);


  // Trajectory storage
  std::map<std::string, TrajectoryInfo> traj_info_map_;
  mutable std::recursive_mutex trajectory_info_map_mutex_;
private:

};
} // namespace moveit_simple
#endif // TRAJECTORY_PLANNER_H
