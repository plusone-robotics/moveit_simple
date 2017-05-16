/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Shaun Edwards
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

#include "moveit_simple/moveit_simple.h"
#include "cxx-prettyprint/prettyprint.hpp"
#include "ros/console.h"
#include "eigen_conversions/eigen_msg.h"

namespace moveit_simple
{



Robot::Robot(const ros::NodeHandle & nh, const std::string &robot_description,
             const std::string &group_name):
  action_("joint_trajectory_action", true),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  nh_(nh),
  speed_modifier_(1.0)
{
  ROS_INFO_STREAM("Loading MoveIt objects based on, robot description: " << robot_description
                  << ", group name: " << group_name);
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader
                            (robot_description));
  robot_model_ptr_ = robot_model_loader_->getModel();
  robot_state_.reset(new moveit::core::RobotState(robot_model_ptr_));
  robot_state_->setToDefaultValues();
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_ptr_));
  joint_group_ = robot_model_ptr_->getJointModelGroup(group_name);

  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(
                        robot_model_ptr_->getRootLinkName(),
                        nh_.getNamespace() + "/rviz_visual_tools",
                        robot_model_ptr_));
  visual_tools_->loadRobotStatePub(nh_.getNamespace() + "/display_robot_state");

  ROS_INFO_STREAM("Calculating all positions assuming, root: " << robot_model_ptr_->getRootLinkName()
                  << ", and tool: " << robot_model_ptr_->getLinkModelNames());
  ROS_INFO_STREAM("MoveIt object loaded");

  ROS_INFO_STREAM("Waiting for action servers");
  action_.waitForServer(ros::Duration(30.0));
  ROS_INFO_STREAM("Done waiting for action servers");

  ROS_INFO_STREAM("Loading ROS pubs/subs");
  j_state_sub_ = nh_.subscribe("joint_states", 1, &Robot::updateState, this);

  // Dynamic ReConfigure 
  dynamic_reconfig_callback_type_ = boost::bind(&Robot::dynamic_reconfig_callback, this, _1, _2);
  dynamic_reconfig_server_.setCallback(dynamic_reconfig_callback_type_);

  //TODO: How to handle action server and other failures in the constructor
  // Perhaps move any items that can fail our of the constructor into an init
  // function with a proper return
  if( !action_.isServerConnected() )
  {
    ROS_ERROR_STREAM("Failed to connect to joint trajectory action server: ");
  }

  return;
}



bool Robot::addTrajPoint(const std::string & traj_name, const Eigen::Affine3d pose,
                  const std::string & frame, double time,
                  const std::string & point_name)
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  bool success = false;
  ROS_INFO_STREAM("Attempting to add " << point_name << " to " << traj_name << "relative to"
                  << frame << " at time " << time);
  try
  {
    Eigen::Affine3d pose_rel_robot = transformToBase(pose, frame);
    std::unique_ptr<TrajectoryPoint> point =
      std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(pose_rel_robot, time, point_name));
    success = addTrajPoint(traj_name, point);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM("Add to trajectory failed for arbitrary pose point: " << ex.what());
    success = false;
  }
  return success;

}




bool Robot::addTrajPoint(const std::string & traj_name, const std::string & point_name,
                         double time)
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  ROS_INFO_STREAM("Attempting to add " << point_name << " to " << traj_name
                 << " at time " << time);
  std::unique_ptr<TrajectoryPoint> point = lookupTrajectoryPoint(point_name, time);
  return addTrajPoint(traj_name, point);
}




bool Robot::addTrajPoint(const std::string & traj_name,
                         std::unique_ptr<TrajectoryPoint> & point)
{
  bool success = false;
  if( point )
  {
    traj_map_[traj_name].push_back(std::move(point));
    success = true;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to add point for trajectory " << traj_name);
    success = false;
  }
  return success;
}




std::unique_ptr<TrajectoryPoint> Robot::lookupTrajectoryPoint(const std::string & name,
                                                            double time) const
{
  ROS_INFO_STREAM("Looking up trajectory point: " << name);

  if ( robot_state_->setToDefaultValues(joint_group_, name) )
  {
    ROS_INFO_STREAM("Looked up named joint target from robot model: " << name);
    std::vector<double> joint_point;
    robot_state_->copyJointGroupPositions(joint_group_->getName(), joint_point);
    return std::unique_ptr<TrajectoryPoint>(new JointTrajectoryPoint(joint_point, time, name));
  }

  else if (robot_model_ptr_->hasLinkModel(name) )
  {
    ROS_INFO_STREAM("Looked up named cart target from robot model: " << name);
    robot_state_->update();  //Updating state for frame tansform below
    Eigen::Affine3d pose = robot_state_->getFrameTransform(name);
    ROS_INFO_STREAM("Getting urdf/robot_state target: " << name << " frame: " << std::endl << pose.matrix());
    return std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(pose, time, name));
  }

  else if( tf_buffer_.canTransform(name, robot_model_ptr_->getRootLinkName(), ros::Time::now(), ros::Duration(0.1)) )
  {
    try
    {
      ROS_INFO_STREAM("Looked up tf named frame: " << name);
      geometry_msgs::TransformStamped trans_msg;
      Eigen::Affine3d pose;
      trans_msg = tf_buffer_.lookupTransform(robot_model_ptr_->getRootLinkName(), name,
                                           ros::Time::now(), ros::Duration(5.0));
      tf::transformMsgToEigen(trans_msg.transform,pose);
      ROS_INFO_STREAM("Using TF to lookup transform " << name << " frame: " << std::endl << pose.matrix());
      return std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(pose, time, name));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM("TF transform lookup failed: " << ex.what());
      return std::unique_ptr<TrajectoryPoint>(nullptr);
    }
  }

  else
  {
    ROS_ERROR_STREAM("Failed to find point " << name << ", consider implementing more look ups");
    return std::unique_ptr<TrajectoryPoint>(nullptr);
  }
}




bool Robot::getJointSolution(const Eigen::Affine3d &pose, double timeout,
                             const std::vector<double> & seed,
                             std::vector<double> & joint_point) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  std::vector<double> local_seed = seed;
  if (seed.empty())
  {
    ROS_INFO_STREAM("Empty seed passed to getJointSolution, using current state");
    robot_state_->copyJointGroupPositions(joint_group_->getName(), local_seed);
  }
  return getIK(pose, local_seed, joint_point, timeout);
}


bool Robot::isInCollision(const Eigen::Affine3d pose, const std::string & frame,
                   double timeout, std::vector<double> joint_seed) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  bool inCollision = true;
  try
  {
    Eigen::Affine3d pose_rel_robot = transformToBase(pose, frame);
    std::unique_ptr<TrajectoryPoint> point =
      std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(pose_rel_robot, 0.0));
    robot_state_->copyJointGroupPositions(joint_group_->getName(), joint_seed);
    inCollision = planning_scene_->isStateColliding(*robot_state_, joint_group_->getName());
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM("IsInCollision failed for for arbitrary pose point: " << ex.what());
    inCollision = true;
  }
  return inCollision;
}


bool Robot::isReachable(const std::string & name, double timeout,
                        std::vector<double> joint_seed) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  std::unique_ptr<TrajectoryPoint> point = lookupTrajectoryPoint(name, 0.0);
  return isReachable(point, timeout, joint_seed );
}




bool Robot::isReachable(const Eigen::Affine3d & pose, const std::string & frame,
                        double timeout, std::vector<double> joint_seed) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  bool reacheable = false;
  try
  {
    Eigen::Affine3d pose_rel_robot = transformToBase(pose, frame);
    std::unique_ptr<TrajectoryPoint> point =
      std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(pose_rel_robot, 0.0));
    reacheable = isReachable(point, timeout, joint_seed );
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM("Reacheabilioty failed for for arbitrary pose point: " << ex.what());
    reacheable = false;
  }
  return reacheable;
}




bool Robot::isReachable(std::unique_ptr<TrajectoryPoint> & point, double timeout,
                 std::vector<double> joint_seed) const
{
  bool reacheable = false;

  if( point )
  {
    if (joint_seed.empty())
    {
      ROS_DEBUG_STREAM("Empty seed passed to reach check, using current state");
      robot_state_->copyJointGroupPositions(joint_group_->getName(), joint_seed);

    }
    std::unique_ptr<JointTrajectoryPoint> dummy =
        point->toJointTrajPoint(*this, timeout, joint_seed);
    if( dummy )
    {
      reacheable = true;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Invalid point for reach check");
    reacheable = false;
  }
  return reacheable;
}



void Robot::clearTrajectory(const::std::string traj_name)
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  traj_map_.erase(traj_name);
}



bool Robot::execute(const std::string traj_name)
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  const double TIMEOUT_SCALE = 1.25;  //scales time to wait for action timeout.
  bool success = false;
  if ( traj_map_.count(traj_name) )
  {
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = joint_group_->getVariableNames();
    if ( toJointTrajectory(traj_name, goal.trajectory.points) )
    {
      // Modifies the speed of execution for the trajectory based off of the speed_modifier_
      for (std::size_t i = 0; i < goal.trajectory.points.size(); i++)
      {
        goal.trajectory.points[i].time_from_start *= speed_modifier_;
      }

      ros::Duration traj_time =
          goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start;
      ros::Duration timeout(TIMEOUT_SCALE * traj_time.toSec());
      if (action_.sendGoalAndWait(goal, timeout) == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO_STREAM("Successfully executed trajectory: " << traj_name);
        success = true;
      }
      else
      {
        ROS_WARN_STREAM("Trajectory " << traj_name << " failed to exectue");
        success = false;
      }
    }
    else
    {
      ROS_ERROR_STREAM("Failed to convert " << traj_name << " to joint trajectory");
      success = false;
    }

  }
  else
  {
    ROS_ERROR_STREAM("Trajectoy " << traj_name << " not found");
    success = false;
  }
  return success;
}

bool Robot::toJointTrajectory(const std::string traj_name,
                       std::vector<trajectory_msgs::JointTrajectoryPoint> & points)
{
  const double IK_TIMEOUT = 0.250;   //250 ms for IK solving
  const Trajectory & traj = traj_map_[traj_name];

  // The first point in any trajectory is the current pose
  std::vector<double> current_joint_position;
  robot_state_->copyJointGroupPositions(joint_group_->getName(), current_joint_position);
  points.push_back(toJointTrajPtMsg(current_joint_position, 0.0));

  for(size_t i = 0; i < traj.size(); ++i)
  {
    const std::unique_ptr<TrajectoryPoint> & traj_point = traj[i];
    std::vector<double> prev_point = points[i].positions;
    std::unique_ptr<JointTrajectoryPoint> current_point;

    if( traj_point->type() != TrajectoryPoint::JOINT )
    {
      const size_t MAX_IK_ATTEMPTS = 2;
      size_t num_attempts = 0;

      while(true)
      {
        num_attempts++;
        current_point = traj_point->toJointTrajPoint(*this, IK_TIMEOUT, prev_point);

        if (current_point)
        {
          if( isConfigChange(prev_point, current_point->jointPoint()) )
          {
            ROS_WARN_STREAM("Configuration change detected in move to/from cart point: " << i << "("
                             << traj_point->name() << "), of type: "
                             << traj_point->type() << " from: " << traj_name << " to joint trajectory");
          }
          else
          {
            ROS_INFO_STREAM("Found proper IK (no config change) in " << num_attempts
                            << " attempts");
            break;
          }
        }
        else
        {
          ROS_WARN_STREAM("Failed to convert trajectory point: " << i << "("
                         << traj_point->name() << "), of type: "
                         << traj_point->type() << " from: " << traj_name << " to joint trajectory");
          return false;
        }
        if (num_attempts >= MAX_IK_ATTEMPTS)
        {
          ROS_ERROR_STREAM("Failed to find proper IK (no config change) in " << num_attempts
                           << " attempts");
          return false;
        }
      }
    }
    else
    {
      current_point = traj_point->toJointTrajPoint(*this, IK_TIMEOUT, prev_point);
      if (!current_point)
      {
        ROS_WARN_STREAM("Failed to convernt joint point - this shouldn't happen");
        return false;
      }
    }

    points.push_back(toJointTrajPtMsg(*current_point));
    ROS_INFO_STREAM("Appending trajectory point, size: " << points.size());
  }
  return true;
}


bool Robot::isConfigChange(const std::vector<double> jp1,
                    const std::vector<double> jp2) const
{
  // The maximum allowed change in joint value
  const double MAX_JOINT_CHANGE = 90.0 * M_PI/180.0;
  // The number of joints allowed to exeed the maximum joint change.
  const int MAX_JOINT_CHANGE_COUNT = 1;

  if( jp1.size() != jp2.size() )
  {
    ROS_ERROR_STREAM("Cannot check for config change, size mismatch");
    return true;
  }

  size_t j_change_count = 0;
  for( size_t ii = 0; ii < jp1.size(); ii++ )
  {
    double joint_change = std::abs(jp1[ii] - jp2[ii]);
    if ( joint_change > MAX_JOINT_CHANGE )
    {
      ROS_INFO_STREAM("Joint[" << ii << "] change of " << joint_change << " exceeds limit: "
                      << MAX_JOINT_CHANGE);
      j_change_count++;
    }
    if ( j_change_count > MAX_JOINT_CHANGE_COUNT)
    {
      ROS_WARN_STREAM("Possible config change detected, with " << j_change_count
                      << " joints exceeding max chage: " << MAX_JOINT_CHANGE);
      return true;
    }
  }
  return false;
}




Eigen::Affine3d Robot::transformToBase(const Eigen::Affine3d &in,
                                       const std::string &in_frame) const
{
  //tf2::Stamped<Eigen::Affine3d> out;
  Eigen::Affine3d out;
  bool success = false;
  try
  {
    geometry_msgs::TransformStamped frame_rel_robot_msg =
        tf_buffer_.lookupTransform(in_frame, robot_model_ptr_->getRootLinkName(),
                                 ros::Time::now(), ros::Duration(5.0));
    Eigen::Affine3d frame_rel_robot;
    tf::transformMsgToEigen(frame_rel_robot_msg.transform, frame_rel_robot);
    out = frame_rel_robot.inverse() * in;
    success = true;
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM("Transform lookup from: " << in_frame << " into robot base: "
                     << robot_model_ptr_->getRootLinkName() << "::" << ex.what());
    throw ex;
  }
  return out;
}




bool Robot::getIK(const Eigen::Affine3d pose, const std::vector<double> & seed,
                  std::vector<double> & joint_point,
                  double timeout, unsigned int attempts) const
{
  robot_state_->setJointGroupPositions(joint_group_, seed);
  return getIK(pose, joint_point, timeout, attempts);
}




bool Robot::getIK(const Eigen::Affine3d pose, std::vector<double> & joint_point,
                  double timeout, unsigned int attempts) const
{
  if ( robot_state_->setFromIK(joint_group_, pose, attempts, timeout) )
  {
    robot_state_->copyJointGroupPositions(joint_group_->getName(), joint_point);
    visual_tools_->deleteAllMarkers();
    robot_state_->update();
    visual_tools_->publishRobotState(robot_state_, rviz_visual_tools::PURPLE);
    visual_tools_->publishContactPoints(*robot_state_, &(*planning_scene_));
    ros::spinOnce();
    return true;
  }
  return false;
}




trajectory_msgs::JointTrajectoryPoint Robot::toJointTrajPtMsg(
    const JointTrajectoryPoint & joint_point) const
{
  return toJointTrajPtMsg(joint_point.jointPoint(), joint_point.time());
}




trajectory_msgs::JointTrajectoryPoint Robot::toJointTrajPtMsg(
    const std::vector<double> & joint_point, double time)
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




void Robot::updateState(const sensor_msgs::JointStateConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> guard(m_);
  robot_state_->setVariablePositions(msg->name, msg->position);
}

void 
Robot::dynamic_reconfig_callback(moveit_simple::moveit_simple_dynamic_reConfig &config,
                                 uint32_t level)
{
  /// Hard coded these checks in for now.
  if (config.speed_modifier <= 10.0 && config.speed_modifier > 0.0)
    speed_modifier_ = config.speed_modifier;
}

}
