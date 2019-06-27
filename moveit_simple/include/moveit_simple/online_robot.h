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

#ifndef ONLINE_ROBOT_H
#define ONLINE_ROBOT_H

#include <atomic>
#include <thread>

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

#include <moveit_simple/robot.h>

namespace moveit_simple
{
/**
 * @brief OnlineRobot is a wrapper around standard MoveIt objects.
 * It inherits from Robot. Added assumption:
 * motion is executed via a "JointTrajectoryAction" interface
*/
class OnlineRobot : public Robot
{
public:
  /**
  * @brief Constructor
  */
  OnlineRobot(const ros::NodeHandle &nh, const std::string &robot_description,
    const std::string &group_name);

  /**
  * @brief Constructor for the case where the IK implementation does not match the
  * SRDF. For example: IKFast solutions are generally solved for the base_link to
  * tool0 of a robot, and the robot_description is defined for some world_frame to
  * some tcp frame.
  */
  OnlineRobot(const ros::NodeHandle &nh, const std::string &robot_description,
    const std::string &group_name, const std::string &ik_base_frame,
    const std::string &ik_tip_frame);

  /**
   * @brief execute a given trajectory
   * @param traj_name - name of trajectory to be executed (must be filled with
   * prior calls to "addTrajPoint".
   * @param collision_check - bool to turn check for collision on\off
   * @param retime_trajectory - bool to fix trajectory timesteps if necessary on\off
   * @throws <moveit_simple::ExecutionFailureException> (Execution failure)
   * @throws <moveit_simple::IKFailException> (Conversion to joint trajectory failed)
   * @throws <std::invalid_argument> (Trajectory "traj_name" not found)
   * @throws <moveit_simple::CollisionDetected> (One of interpolated point is
   * in Collision with itself or environment)
   */
  void execute(const std::string traj_name, bool collision_check = false, bool retime_trajectory = false);

  /**
   * @brief execute a given planned joint trajectory
   * @param goal - Joint Trajectory goal which is a known 'Plan'
   * @param collision_check - bool to turn check for collision on\off
   * @param retime_trajectory - bool to fix trajectory timesteps if necessary on\off
   * @throws <moveit_simple::ExecutionFailureException> (Execution failure)
   * @throws <moveit_simple::CollisionDetected> (One of interpolated point is
   * in Collision with itself or environment)
   */
  void execute(std::vector<moveit_simple::JointTrajectoryPoint> &goal, bool collision_check = false,
               bool retime_trajectory = false);

  /**
   * @brief Starts execution for a given trajectory, non blocking
   * @param traj_name - name of trajectory to be executed (must be filled with
   * prior calls to "addTrajPoint".
   * @param collision_check - bool to turn check for collision on\off
   * @throws <moveit_simple::ExecutionFailureException> (Execution failure)
   * @throws <moveit_simple::IKFailException> (Conversion to joint trajectory failed)
   * @throws <std::invalid_argument> (Trajectory "traj_name" not found)
   * @throws <moveit_simple::CollisionDetected> (One of interpolated point is
   * in Collision with itself or environment)
   */
  void startExecution(const std::string &traj_name, bool collision_check = false);

  /**
  * @brief Checks if the non-blocking execute is still executing
  * @return bool - true if it is, false if not
  */
  bool isExecuting();

  /**
  * @brief Stops the non-blocking execution
  */
  void stopExecution();

  /**
  * @brief getJointState - Returns a vector<double> of the
  * current joint positions of the robot from current_robot_state_.
  *
  * @return std::vector<double> current_joint_position
  */
  virtual std::vector<double> getJointState(void) const;

protected:
  void executing(const ros::Time &timeout);

  void updateCurrentState(const sensor_msgs::JointStateConstPtr &msg);

protected:
  mutable moveit::core::RobotStatePtr current_robot_state_;
  moveit_visual_tools::MoveItVisualToolsPtr online_visual_tools_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_;
  ros::Subscriber j_state_sub_;
  std::atomic<bool> is_timed_out_;
};
} // namespace moveit_simple
#endif // ONLINE_ROBOT_H
