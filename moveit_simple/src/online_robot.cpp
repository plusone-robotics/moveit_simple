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

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <moveit_simple/exceptions.h>
#include <moveit_simple/online_robot.h>
#include <moveit_simple/point_types.h>
#include <moveit_simple/trajectory_processing.h>

namespace moveit_simple
{
OnlineRobot::OnlineRobot(const ros::NodeHandle &nh, const std::string &robot_description,
  const std::string &group_name) : Robot(nh, robot_description, group_name),
  action_("joint_trajectory_action", true)
{
  current_robot_state_.reset(new moveit::core::RobotState(robot_model_ptr_));
  current_robot_state_->setToDefaultValues();

  online_visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(robot_model_ptr_->getRootLinkName(),
    nh_.getNamespace() + "/rviz_visual_tools", robot_model_ptr_));
  online_visual_tools_->loadRobotStatePub(nh_.getNamespace() + "/display_robot_state");

  ROS_INFO_STREAM("Waiting for action servers");
  action_.waitForServer(ros::Duration(30.0));
  ROS_INFO_STREAM("Done waiting for action servers");

  ROS_INFO_STREAM("Loading ROS pubs/subs");
  j_state_sub_ = nh_.subscribe("joint_states", 1, &OnlineRobot::updateCurrentState, this);

  // TODO: How to handle action server and other failures in the constructor
  // Perhaps move any items that can fail our of the constructor into an init
  // function with a proper return
  if (!action_.isServerConnected())
  {
    ROS_ERROR_STREAM("Failed to connect to joint trajectory action server: ");
  }

  return;
}

OnlineRobot::OnlineRobot(const ros::NodeHandle &nh, const std::string &robot_description,
  const std::string &group_name, const std::string &ik_base_frame, const std::string &ik_tip_frame)
  : OnlineRobot(nh, robot_description, group_name)
{
  try
  {
    ik_base_frame_ = ik_base_frame;
    ik_tip_frame_ = ik_tip_frame;
    this->computeIKSolverTransforms();
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR_STREAM("Failed to compute transforms between the base/tip frames defined"
      << " in the SRDF and the base/tip frames defined for the IK solver");
    throw IKSolverTransformException("Failed to compute transforms between the base/tip"
      " frame defined in the SRDF and the base/tip frames defined for the IK solver");
  }
}

void OnlineRobot::execute(const std::string traj_name, bool collision_check, bool retime_trajectory)
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  try
  {
    std::vector<moveit_simple::JointTrajectoryPoint> goal;

    goal = plan(traj_name, collision_check);
    execute(goal, false, retime_trajectory);
  }
  catch (IKFailException &ik)
  {
    ROS_ERROR_STREAM("IK Failed for trajectory: [" << traj_name << "]");
    throw ik;
  }
  catch (CollisionDetected &cd)
  {
    ROS_ERROR_STREAM("Collision detected in trajectory");
    throw cd;
  }
  catch (InvalidTrajectoryException &ite)
  {
    ROS_ERROR_STREAM("Invalid trajectory [" << traj_name << "]: " << ite.what());
    throw ite;
  }
  catch (std::invalid_argument &ia)
  {
    ROS_ERROR_STREAM("Invalid trajectory name: [" << traj_name << "]");
    throw ia;
  }
  catch (ExecutionFailureException &ef)
  {
    ROS_ERROR_STREAM("Trajectory [" << traj_name << "] failed to execute: " << ef.what());
    throw ef;
  }
}

void OnlineRobot::execute(std::vector<moveit_simple::JointTrajectoryPoint> &joint_trajectory_points,
                          bool collision_check, bool retime_trajectory)
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  const double TIMEOUT_SCALE = 1.25;  // scales time to wait for action timeout.

  control_msgs::FollowJointTrajectoryGoal goal = toFollowJointTrajectoryGoal(joint_trajectory_points);

  // validate trajectory waypoint bounds and timestamps
  // if invalid, attempt to fix it or throw InvalidTrajectoryException
  trajectory_processing::validateTrajectory(robot_model_ptr_, joint_group_->getName(), goal.trajectory, retime_trajectory);

  int collision_points = trajCollisionCheck(goal, collision_check);

  if (collision_points == 0)
  {
    ros::Duration traj_time =
      goal.trajectory.points[goal.trajectory.points.size() - 1].time_from_start;

    ros::Duration timeout(TIMEOUT_SCALE * traj_time.toSec());

    if (action_.sendGoalAndWait(goal, timeout) == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO_STREAM("Successfully executed Joint Trajectory ");
    }
    else
    {
      ROS_ERROR_STREAM("Joint Trajectory failed to execute.. check input Plan");
      throw ExecutionFailureException("Execution failed for Joint Trajectory");
    }
  }
  else
  {
    ROS_ERROR_STREAM("Collision detected at " << collision_points
      << " points for Joint Trajectory");
    throw CollisionDetected("Collision detected while interpolating ");
  }
}

void OnlineRobot::startExecution(const std::string &traj_name, bool collision_check)
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  try
  {
    std::vector<moveit_simple::JointTrajectoryPoint> joint_trajectory_points;
    joint_trajectory_points = plan(traj_name, collision_check);

    const double TIMEOUT_SCALE = 1.25;
    control_msgs::FollowJointTrajectoryGoal goal = this->toFollowJointTrajectoryGoal(joint_trajectory_points);

    auto collision_points = this->trajCollisionCheck(goal, collision_check);

    if (collision_points == 0)
    {
      auto traj_time = goal.trajectory.points[goal.trajectory.points.size() - 1].time_from_start;
      auto timeout = ros::Time::now() + ros::Duration(TIMEOUT_SCALE*traj_time.toSec());

      action_.sendGoal(goal);
      is_timed_out_ = false;
      std::thread execution_thread(&OnlineRobot::executing, this, timeout);
      execution_thread.detach();
    }
    else
    {
      ROS_ERROR_STREAM("Collision detected in trajectory: " << traj_name);
      throw CollisionDetected("Collision detected while interpolating ");
    }
  }
  catch (IKFailException &ik)
  {
    ROS_ERROR_STREAM("IK Failed for trajectory: [" << traj_name << "]");
    throw ik;
  }
  catch (std::invalid_argument &ia)
  {
    ROS_ERROR_STREAM("Invalid trajectory name: [" << traj_name << "]");
    throw ia;
  }
  catch (ExecutionFailureException &ef)
  {
    ROS_ERROR_STREAM("Trajectory [" << traj_name << "] failed to execute");
    throw ef;
  }
}

bool OnlineRobot::isExecuting()
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  if (!is_timed_out_)
  {
    auto action_state = action_.getState();
    if (action_state == actionlib::SimpleClientGoalState::PENDING || action_state == actionlib::SimpleClientGoalState::ACTIVE)
    {
      return true;
    }
    else
    {
      this->stopExecution();
      return false;
    }
  }
  else
  {
    this->stopExecution();
    ROS_ERROR_STREAM("Execution timed out");
    return false;
  }
}

void OnlineRobot::stopExecution()
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  action_.cancelGoal();
  auto action_state = action_.getState();
  while (action_state != actionlib::SimpleClientGoalState::RECALLED
         && action_state != actionlib::SimpleClientGoalState::PREEMPTED
         && action_state != actionlib::SimpleClientGoalState::ABORTED
         && action_state != actionlib::SimpleClientGoalState::SUCCEEDED
         && action_state != actionlib::SimpleClientGoalState::REJECTED)
  {
    action_state = action_.getState();
  }
}

std::vector<double> OnlineRobot::getJointState(void) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  ros::spinOnce();
  std::vector<double> current_joint_positions;

  current_robot_state_->update();
  current_robot_state_->copyJointGroupPositions(joint_group_->getName(), current_joint_positions);

  return current_joint_positions;
}

void OnlineRobot::executing(const ros::Time &timeout)
{
  auto action_state = action_.getState();
  while (ros::Time::now() <= timeout && action_state == actionlib::SimpleClientGoalState::ACTIVE)
  {
    is_timed_out_ = false;
    action_state = action_.getState();
  }

  if (action_state == actionlib::SimpleClientGoalState::ACTIVE)
    is_timed_out_ = true;
}

void OnlineRobot::updateCurrentState(const sensor_msgs::JointStateConstPtr &msg)
{
  current_robot_state_->setVariablePositions(msg->name, msg->position);
}
} // namespace moveit_simple
