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

#include <moveit_simple/exceptions.h>

#ifndef TRAJECTORY_PROCESSING_H
#define TRAJECTORY_PROCESSING_H
namespace moveit_simple
{
namespace trajectory_processing
{
struct TrajectoryValidationResult
{
  enum { Success,
         InvalidPosition,
         InvalidVelocity,
         InvalidAcceleration,
         InvalidTimestamp} value;
  std::string error_message;
};
TrajectoryValidationResult validateTrajectory(robot_model::RobotModelConstPtr robot_model,
                                              const trajectory_msgs::JointTrajectory& trajectory)
{
  TrajectoryValidationResult result;
  const std::vector<std::string>& joint_names = trajectory.joint_names;
  trajectory_msgs::JointTrajectoryPoint previous_point;
  for (size_t i = 0; i < trajectory.points.size(); ++i)
  {
    // validate waypoint bounds
    const auto& point = trajectory.points[i];
    for (size_t j = 0; j < joint_names.size(); ++j)
    {
      const auto& joint_model = robot_model->getJointModel(joint_names[j]);
      const auto& bounds = joint_model->getVariableBounds()[0];
      // validate position bounds and throw exception since this can't be fixed easily
      if (bounds.position_bounded_ && !joint_model->satisfiesPositionBounds(&point.positions[j]))
      {
        result.value = TrajectoryValidationResult::InvalidPosition;
        result.error_message = "Trajectory contains waypoints that don't satisfy position bounds";
        return result;
      }
      // validate velocity/acceleration bounds and pair-wise time difference
      double dt = (point.time_from_start - previous_point.time_from_start).toSec();
      if (bounds.velocity_bounded_)
      {
        result.value = TrajectoryValidationResult::InvalidVelocity;
        if (bounds.max_velocity_ < std::abs(point.velocities[j]))
        {
          result.error_message = "Trajectory waypoint does not satisfy velocity bounds";
          return result;
        }
        if (i > 0 && bounds.max_velocity_ * dt < std::abs(point.positions[j] - previous_point.positions[j]))
        {
          result.error_message = "Trajectory waypoint position not reachable within target time and velocity limits";
          return result;
        }
      }
      // validate acceleration bounds and pair-wise velocty difference
      if (bounds.acceleration_bounded_)
      {
        result.value = TrajectoryValidationResult::InvalidAcceleration;
        if (bounds.max_acceleration_ < std::abs(point.accelerations[j]))
        {
          result.error_message = "Trajectory waypoint does not satisfy acceleration bounds";
          return result;
        }
        if (i > 0 && bounds.max_acceleration_ * dt < std::abs(point.velocities[j] - previous_point.velocities[j]))
        {
          result.error_message = "Trajectory waypoint velocity not reachable within target time and acceleration limits";
          return result;
        }
      }
    }
    previous_point = point;
  }
  result.value = TrajectoryValidationResult::Success;
  return result;
}

bool fixTrajectory(robot_model::RobotModelConstPtr robot_model, const trajectory_msgs::JointTrajectory& trajectory)
{
  // TODO(henningkayser): compute timestamps
  return true;
}

void validateTrajectory(robot_model::RobotModelConstPtr robot_model,
                        const trajectory_msgs::JointTrajectory& trajectory,
                        bool fix_trajectory)
{
  TrajectoryValidationResult result = validateTrajectory(robot_model, trajectory);
  if (result.value != TrajectoryValidationResult::Success)
  {
    // we can't fix invalid positions by trajectory parameterization
    if (fix_trajectory && result.value != TrajectoryValidationResult::InvalidPosition)
      fixTrajectory(robot_model, trajectory);
    else
      throw InvalidTrajectoryException(result.error_message);
  }
}
}  // namespace trajectory_processing
}  // namespace moveit_simple
#endif  // TRAJECTORY_PROCESSING_H
