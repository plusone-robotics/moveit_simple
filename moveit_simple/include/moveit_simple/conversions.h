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

#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <moveit_simple/point_types.h>


namespace moveit_simple
{
  /**
   * @brief toJointTrajPtMsg - Converts native joint point (vector + time) to ROS joint
   * trajectory point message type.
   * @param joint_point
   * @param time
   * @return
   */
  trajectory_msgs::JointTrajectoryPoint toJointTrajPtMsg(const std::vector<double> &joint_point,
    double time);

  trajectory_msgs::JointTrajectoryPoint toJointTrajPtMsg(const JointTrajectoryPoint &joint_point);

  std::vector<moveit_simple::JointTrajectoryPoint>
  toJointTrajectoryPoint(std::vector<trajectory_msgs::JointTrajectoryPoint>& ros_joint_trajectory_points);

} // namespace moveit_simple
#endif // CONVERSIONS_H
