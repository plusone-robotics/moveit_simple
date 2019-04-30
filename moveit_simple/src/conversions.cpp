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

#include <moveit_simple/conversions.h>


namespace moveit_simple
{
  trajectory_msgs::JointTrajectoryPoint toJointTrajPtMsg(const std::vector<double> &joint_point,
    double time)
  {
    trajectory_msgs::JointTrajectoryPoint rtn;
    std::vector<double> empty(joint_point.size(), 0.0);

    rtn.positions = joint_point;
    rtn.velocities = empty;

    rtn.accelerations = empty;
    rtn.effort = empty;
    rtn.time_from_start = ros::Duration(time);

    return rtn;
  }

  trajectory_msgs::JointTrajectoryPoint toJointTrajPtMsg(const JointTrajectoryPoint &joint_point)
  {
    return toJointTrajPtMsg(joint_point.jointPoint(), joint_point.time());
  }

  std::vector<moveit_simple::JointTrajectoryPoint>
  toJointTrajectoryPoint(std::vector<trajectory_msgs::JointTrajectoryPoint>& ros_joint_trajectory_points)
  {
    std::vector<moveit_simple::JointTrajectoryPoint> goal;
    goal.reserve(ros_joint_trajectory_points.size());

    for (std::size_t i = 0; i < ros_joint_trajectory_points.size(); i++)
    {
      JointTrajectoryPoint point(ros_joint_trajectory_points[i].positions,
                                 ros_joint_trajectory_points[i].time_from_start.toSec(), "");
      goal.push_back(point);
    }

    return goal;
  }

} // namespace moveit_simple
