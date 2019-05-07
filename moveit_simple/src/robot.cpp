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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "moveit_simple_msgs/CombinedJointPoint.h"
#include "moveit_simple_msgs/LookupTrajectory.h"
#include "moveit_simple_msgs/LookupWaypoint.h"

#include <moveit_simple/exceptions.h>
#include <moveit_simple/joint_locker.h>
#include <moveit_simple/point_types.h>
#include <moveit_simple/prettyprint.hpp>
#include <moveit_simple/conversions.h>
#include <moveit_simple/robot.h>

namespace moveit_simple
{
Robot::Robot(const ros::NodeHandle& nh, const std::string& robot_description, const std::string& group_name)
  : tf_buffer_()
  , tf_listener_(tf_buffer_)
  , nh_(nh)
  , speed_modifier_(1.0)
  , params_(ros::NodeHandle("~/moveit_simple"))
  , planning_group_(group_name)
  , robot_description_(robot_description)
{
  this->refreshRobot();

  // Dynamic Reconfigure Parameters with rosparam_handler
  dynamic_reconfig_server_ = std::make_shared<dynamic_reconfigure::Server<moveit_simple_dynamic_reconfigure_Config>>(
      ros::NodeHandle("~/moveit_simple"));
  params_.fromParamServer();
  dynamic_reconfig_server_->setCallback(boost::bind(&Robot::reconfigureRequest, this, _1, _2));

  try
  {
    ik_base_frame_ = joint_group_->getSolverInstance()->getBaseFrame();
    ik_tip_frame_ = joint_group_->getSolverInstance()->getTipFrame();
    this->computeIKSolverTransforms();
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM("Failed to compute transforms between the base/tip frames defined"
                     << " in the SRDF and the base/tip frames defined for the IK solver");
    throw IKSolverTransformException("Failed to compute transforms between the base/tip"
                                     " frame defined in the SRDF and the base/tip frames defined for the IK solver");
  }

  lookup_wp_client_ = nh_.serviceClient<moveit_simple_msgs::LookupWaypoint>("lookup_waypoint");
}

Robot::Robot(const ros::NodeHandle& nh, const std::string& robot_description, const std::string& group_name,
             const std::string& ik_base_frame, const std::string& ik_tip_frame)
  : Robot(nh, robot_description, group_name)
{
  try
  {
    ik_base_frame_ = ik_base_frame;
    ik_tip_frame_ = ik_tip_frame;
    this->computeIKSolverTransforms();
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM("Failed to compute transforms between the base/tip frames defined"
                     << " in the SRDF and the base/tip frames defined for the IK solver");
    throw IKSolverTransformException("Failed to compute transforms between the base/tip"
                                     " frame defined in the SRDF and the base/tip frames defined for the IK solver");
  }
}

Robot::~Robot()
{
}

void Robot::refreshRobot()
{
  ROS_INFO_STREAM("Loading MoveIt objects based on, robot description: " << robot_description_
                                                                         << ", group name: " << planning_group_);

  std::lock_guard<std::recursive_mutex> guard(m_);
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(robot_description_));
  robot_model_ptr_ = robot_model_loader_->getModel();

  virtual_robot_state_.reset(new moveit::core::RobotState(robot_model_ptr_));
  virtual_robot_state_->setToDefaultValues();

  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_ptr_));

  joint_group_ = robot_model_ptr_->getJointModelGroup(planning_group_);

  ROS_INFO_STREAM("Calculating all positions assuming, root: " << robot_model_ptr_->getRootLinkName() << ", and tool: "
                                                               << robot_model_ptr_->getLinkModelNames());
  ROS_INFO_STREAM("MoveIt object loaded");

  virtual_visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(
      robot_model_ptr_->getRootLinkName(), nh_.getNamespace() + "/rviz_visual_tools", robot_model_ptr_));
  virtual_visual_tools_->loadRobotStatePub(nh_.getNamespace() + "/display_robot_state");
  virtual_visual_tools_->publishRobotState(virtual_robot_state_, rviz_visual_tools::PURPLE);
  virtual_visual_tools_->trigger();
}

void Robot::addTrajPoint(const std::string& traj_name, const Eigen::Isometry3d pose, const std::string& frame,
                         double time, const InterpolationType& type, const unsigned int num_steps,
                         const std::string& point_name)
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  ROS_INFO_STREAM("Attempting to add " << point_name << " to " << traj_name << "relative to" << frame << " at time "
                                       << time);

  try
  {
    Eigen::Isometry3d pose_rel_robot = transformToBase(pose, frame);
    std::unique_ptr<TrajectoryPoint> point =
        std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(pose_rel_robot, time, point_name));
    addTrajPoint(traj_name, point, type, num_steps);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM("Add to trajectory failed for arbitrary pose point: " << ex.what());
    throw ex;
  }
}

void Robot::addTrajPoint(const std::string& traj_name, const Eigen::Isometry3d pose, const std::string& pose_frame,
                         const std::string& tool_name, double time, const InterpolationType& type,
                         const unsigned int num_steps, const std::string& point_name)
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  auto moveit_tool_link = joint_group_->getSolverInstance()->getTipFrame();

  ROS_INFO_STREAM("Attempting to add " << point_name << " to " << traj_name << " relative to " << pose_frame
                                       << " at time " << time << " and custom_tool_frame [" << tool_name << "]");

  try
  {
    ROS_INFO_STREAM("Transforming Pose to custom_tool_frame frame [" << tool_name << "] from moveit_end_link ["
                                                                     << moveit_tool_link << "]");

    auto pose_rel_robot = this->transformToBase(pose, pose_frame);
    auto custom_tool_to_moveit_tool = this->lookupTransformMoveitToolAndCustomTool(tool_name);
    Eigen::Isometry3d custom_tool_to_moveit_tool_eigen;
    tf::transformMsgToEigen(custom_tool_to_moveit_tool.transform, custom_tool_to_moveit_tool_eigen);
    pose_rel_robot = pose_rel_robot * custom_tool_to_moveit_tool_eigen;

    std::unique_ptr<TrajectoryPoint> point =
        std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(pose_rel_robot, time, point_name));

    addTrajPoint(traj_name, point, type, num_steps);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_STREAM("Add to trajectory failed for arbitrary pose point in Frame[" << tool_name << "]: " << ex.what());
    throw ex;
  }
}

void Robot::addTrajPoint(const std::string& traj_name, const std::string& point_name, double time,
                         const InterpolationType& type, const unsigned int num_steps)
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  ROS_INFO_STREAM("Attempting to add " << point_name << " to " << traj_name << " at time " << time);

  try
  {
    std::unique_ptr<TrajectoryPoint> point = lookupTrajectoryPoint(point_name, time);
    addTrajPoint(traj_name, point, type, num_steps);
  }
  catch (std::invalid_argument& ia)
  {
    ROS_ERROR_STREAM("Invalid point " << point_name << " to add to " << traj_name);
    throw ia;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM(" TF transform failed for " << point_name << " to add to " << traj_name);
    throw ex;
  }
}

void Robot::addTrajPoint(const std::string& traj_name, const std::string& point_name, const std::string& tool_name,
                         double time, const InterpolationType& type, const unsigned int num_steps)
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  ROS_INFO_STREAM("Attempting to add " << point_name << " to " << traj_name << " at time " << time
                                       << " and custom_tool_frame [" << tool_name << "]");

  // Check to see if the point under consideration is a Joint point/state
  if (virtual_robot_state_->setToDefaultValues(joint_group_, point_name))
  {
    ROS_INFO_STREAM("Attempting to add joint point/state [" << point_name << "] to trajectory");

    try
    {
      std::unique_ptr<TrajectoryPoint> point = lookupTrajectoryPoint(point_name, time);
      addTrajPoint(traj_name, point, type, num_steps);
    }
    catch (std::invalid_argument& ia)
    {
      ROS_ERROR_STREAM("Invalid point " << point_name << " to add to " << traj_name);
      throw ia;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR_STREAM(" TF transform failed for " << point_name << " to add to " << traj_name);
      throw ex;
    }
  }
  else
  {
    try
    {
      ROS_INFO_STREAM("Looked up cartesian target [" << point_name << "] from robot model: ");

      // We take the Origin of our reference frame in consideration as the "known pose"
      // In this case our reference frame resolves to the variable "point_name"
      Eigen::Isometry3d pose_eigen_buffer;
      geometry_msgs::Pose pose_buffer;

      pose_buffer.position.x = 0.000;
      pose_buffer.position.y = 0.000;
      pose_buffer.position.z = 0.000;

      pose_buffer.orientation.w = 1.000;
      pose_buffer.orientation.x = 0.000;
      pose_buffer.orientation.y = 0.000;
      pose_buffer.orientation.z = 0.000;

      tf::poseMsgToEigen(pose_buffer, pose_eigen_buffer);
      const Eigen::Isometry3d pose_eigen = pose_eigen_buffer;

      addTrajPoint(traj_name, pose_eigen, point_name, tool_name, time, type, num_steps, point_name);
    }
    catch (std::invalid_argument& ia)
    {
      ROS_ERROR_STREAM("Invalid point " << point_name << " to add to " << traj_name);
      throw ia;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR_STREAM(" TF transform failed for " << point_name << " to add to " << traj_name);
      throw ex;
    }
  }
}

void Robot::addTrajPoint(const std::string& traj_name, std::unique_ptr<TrajectoryPoint>& point,
                         const InterpolationType& type, const unsigned int num_steps)
{
  trajectory_planner_.addTrajPoint(traj_name, point, type, num_steps);
}

void Robot::addTrajPointJointLock(const std::string& traj_name, const std::string& point_name, double time,
                                  const InterpolationType& type, const unsigned int num_steps, JointLockOptions options)
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  ROS_INFO_STREAM("Attempting to add " << point_name << " to " << traj_name << " at time " << time);

  try
  {
    std::unique_ptr<TrajectoryPoint> point = lookupTrajectoryPoint(point_name, time);
    point->setJointLockOptions(options);
    addTrajPoint(traj_name, point, type, num_steps);
  }
  catch (std::invalid_argument& ia)
  {
    ROS_ERROR_STREAM("Invalid point " << point_name << " to add to " << traj_name);
    throw ia;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM(" TF transform failed for " << point_name << " to add to " << traj_name);
    throw ex;
  }
}

void Robot::addTrajPointJointLock(const std::string& traj_name, const Eigen::Isometry3d pose,
                                  const std::string& pose_frame, const std::string& tool_name, double time,
                                  const InterpolationType& type, const unsigned int num_steps,
                                  const std::string& point_name, JointLockOptions options)
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  auto moveit_tool_link = joint_group_->getSolverInstance()->getTipFrame();

  ROS_INFO_STREAM("Attempting to add " << point_name << " to " << traj_name << " relative to " << pose_frame
                                       << " at time " << time << " and custom_tool_frame [" << tool_name << "]");

  try
  {
    ROS_INFO_STREAM("Transforming Pose to custom_tool_frame frame [" << tool_name << "] from moveit_end_link ["
                                                                     << moveit_tool_link << "]");

    auto pose_rel_robot = this->transformToBase(pose, pose_frame);
    auto custom_tool_to_moveit_tool = this->lookupTransformMoveitToolAndCustomTool(tool_name);
    Eigen::Isometry3d custom_tool_to_moveit_tool_eigen;
    tf::transformMsgToEigen(custom_tool_to_moveit_tool.transform, custom_tool_to_moveit_tool_eigen);
    pose_rel_robot = pose_rel_robot * custom_tool_to_moveit_tool_eigen;

    std::unique_ptr<TrajectoryPoint> point =
        std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(pose_rel_robot, time, point_name, options));

    addTrajPoint(traj_name, point, type, num_steps);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_STREAM("Add to trajectory failed for arbitrary pose point in Frame[" << tool_name << "]: " << ex.what());
    throw ex;
  }
}

std::unique_ptr<TrajectoryPoint> Robot::lookupTrajectoryPoint(const std::string& name, double time) const
{
  ROS_INFO_STREAM("Looking up trajectory point: " << name);

  if (virtual_robot_state_->setToDefaultValues(joint_group_, name))
  {
    ROS_INFO_STREAM("Looked up named joint target from robot model: " << name);

    std::vector<double> joint_point;
    virtual_robot_state_->copyJointGroupPositions(joint_group_->getName(), joint_point);

    return std::unique_ptr<TrajectoryPoint>(new JointTrajectoryPoint(joint_point, time, name));
  }
  else if (robot_model_ptr_->hasLinkModel(name))
  {
    ROS_INFO_STREAM("Looked up named cart target from robot model: " << name);

    virtual_robot_state_->update();  // Updating state for frame tansform below
    Eigen::Isometry3d ik_base_to_ik_tip = virtual_robot_state_->getFrameTransform(name);
    Eigen::Isometry3d pose = srdf_base_to_ik_base_ * ik_base_to_ik_tip;

    ROS_INFO_STREAM("Getting urdf/robot_state target: " << name << " frame: " << std::endl << pose.matrix());

    return std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(pose, time, name));
  }
  else if (tf_buffer_.canTransform(name, ik_base_frame_, ros::Time::now(), ros::Duration(0.1)))
  {
    try
    {
      ROS_INFO_STREAM("Looked up tf named frame: " << name);

      geometry_msgs::TransformStamped trans_msg;
      Eigen::Isometry3d pose;
      trans_msg = tf_buffer_.lookupTransform(ik_base_frame_, name, ros::Time::now(), ros::Duration(5.0));
      tf::transformMsgToEigen(trans_msg.transform, pose);

      ROS_INFO_STREAM("Using TF to lookup transform " << name << " frame: " << std::endl << pose.matrix());

      return std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(pose, time, name));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR_STREAM("TF transform lookup failed: " << ex.what());
      throw ex;
    }
  }
  else
  {
    ROS_INFO_STREAM("Requesting trajectory training service info for waypoint: " << name);

    moveit_simple_msgs::LookupWaypoint srv;
    srv.request.waypoint_name = name;

    if (lookup_wp_client_.exists() && lookup_wp_client_.call(srv) && srv.response.success)
    {
      auto waypoint = srv.response.waypoint;
      Eigen::Isometry3d pose;
      tf::transformMsgToEigen(waypoint.transform_stamped.transform, pose);

      return std::unique_ptr<TrajectoryPoint>(
          new CombinedTrajectoryPoint(waypoint.joint_point, pose, time, joint_equality_tolerance_, name,
                                      CombinedTrajectoryPoint::PointPreference::CARTESIAN));
    }
    else
    {
      ROS_ERROR_STREAM("Failed to find point " << name << ", consider implementing more look ups");
      throw std::invalid_argument("Failed to find point: " + name);
    }
  }
}

bool Robot::getJointSolution(const Eigen::Isometry3d& pose, double timeout, const std::vector<double>& seed,
                             std::vector<double>& joint_point) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  std::vector<double> local_seed = seed;
  if (seed.empty())
  {
    ROS_INFO_STREAM("Empty seed passed to getJointSolution, using current state");
    local_seed = getJointState();
  }

  return getIK(pose, local_seed, joint_point, timeout);
}

bool Robot::getJointSolution(const Eigen::Isometry3d& pose, const std::string& tool_name, double timeout,
                             const std::vector<double>& seed, std::vector<double>& joint_point) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  bool get_joints = false;
  std::string moveit_tool_link = joint_group_->getSolverInstance()->getTipFrame();

  try
  {
    ROS_INFO_STREAM("Transforming Pose from custom_tool_frame frame [" << tool_name << "] to moveit_end_link ["
                                                                       << moveit_tool_link << "] before performing IK");

    // Transform Target/Goal Point from custom tool frame to moveit_end_link
    const Eigen::Isometry3d custom_frame_goal_pose = transformPoseBetweenFrames(pose, tool_name, moveit_tool_link);

    std::vector<double> local_seed = seed;
    if (seed.empty())
    {
      ROS_INFO_STREAM("Empty seed passed to getJointSolution, using current state");
      local_seed = getJointState();
    }

    get_joints = getIK(custom_frame_goal_pose, local_seed, joint_point, timeout);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_STREAM("getJointSolution failed for arbitrary pose in Frame[" << tool_name << "]: " << ex.what());

    get_joints = false;
  }

  return get_joints;
}

Eigen::Isometry3d Robot::transformPoseBetweenFrames(const Eigen::Isometry3d& target_pose, const std::string& frame_in,
                                                  const std::string& frame_out) const
{
  if (frame_in.compare(frame_out) == 0 && robot_model_ptr_->hasLinkModel(frame_in) &&
      robot_model_ptr_->hasLinkModel(frame_out))
  {
    ROS_INFO_STREAM("Returning same target_pose as input_frame [" << frame_in << "] and target_frame [" << frame_out
                                                                  << "] both exist in Robot Model"
                                                                  << " and are equal");

    return target_pose;
  }

  try
  {
    ROS_INFO_STREAM("Looked up tf named frame: " << frame_in << " because it doesn't exist in Robot Model");

    geometry_msgs::PoseStamped pose_stamped_buffer;
    geometry_msgs::PoseStamped transformed_pose;
    geometry_msgs::TransformStamped trans_msg;

    Eigen::Isometry3d target_pose_buffer = target_pose;
    trans_msg = tf_buffer_.lookupTransform(frame_out, frame_in, ros::Time(0), ros::Duration(5.0));

    pose_stamped_buffer.header = trans_msg.header;
    tf::poseEigenToMsg(target_pose, pose_stamped_buffer.pose);

    tf2::doTransform(pose_stamped_buffer, transformed_pose, trans_msg);

    tf::poseMsgToEigen(transformed_pose.pose, target_pose_buffer);

    ROS_INFO_STREAM("Using TF to lookup transform " << frame_out << " frame: " << std::endl
                                                    << target_pose_buffer.matrix());

    return target_pose_buffer;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM("TF transform lookup failed from: " << frame_in << " into " << frame_out << "::" << ex.what());
    throw ex;
  }
}

bool Robot::getPose(const std::vector<double>& joint_point, Eigen::Isometry3d& pose) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);
  return getFK(joint_point, pose);
}

bool Robot::getPose(const std::vector<double>& joint_point, const std::string& tool_name, Eigen::Isometry3d& pose) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  bool get_pose = false;
  Eigen::Isometry3d pose_buffer;
  std::string moveit_tool_link = joint_group_->getSolverInstance()->getTipFrame();

  if (getFK(joint_point, pose_buffer))
  {
    try
    {
      ROS_INFO_STREAM("Transforming Pose from moveit_end_link frame [" << moveit_tool_link << "] to custom_tool_frame ["
                                                                       << tool_name << "]");

      // Transform Target/Goal Point from moveit_end_link to custom tool frame
      pose = transformPoseBetweenFrames(pose_buffer, moveit_tool_link, tool_name);

      get_pose = true;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_STREAM("getPose failed for arbitrary Joint Point in Frame[" << tool_name << "]: " << ex.what());

      get_pose = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Robot State failed to find FK for given joint point");
    get_pose = false;
  }

  return get_pose;
}

bool Robot::isInCollision(const Eigen::Isometry3d& pose, const std::string& frame, const std::string& joint_seed,
                          double timeout) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  std::map<std::string, double> m;
  if (!joint_group_->getVariableDefaultPositions(joint_seed, m))
  {
    throw JointSeedException(joint_seed + " is not a named state defined in the SRDF / URDF");
  }

  std::vector<double> joints;
  for (auto it = m.begin(); it != m.end(); ++it)
  {
    joints.push_back(it->second);
  }

  return this->isInCollision(pose, frame, timeout, joints);
}

bool Robot::isInCollision(const Eigen::Isometry3d& pose, const geometry_msgs::TransformStamped& frame_to_robot_base,
                          const std::string& joint_seed, double timeout) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  std::map<std::string, double> m;
  if (!joint_group_->getVariableDefaultPositions(joint_seed, m))
  {
    throw JointSeedException(joint_seed + " is not a named state defined in the SRDF / URDF");
  }

  std::vector<double> joints;
  for (auto it = m.begin(); it != m.end(); ++it)
  {
    joints.push_back(it->second);
  }

  return this->isInCollision(pose, frame_to_robot_base, timeout, joints);
}

bool Robot::isInCollision(const Eigen::Isometry3d& pose, const geometry_msgs::TransformStamped& frame_to_robot_base,
                          const geometry_msgs::TransformStamped& custom_tool_to_moveit_tool,
                          const std::string& joint_seed, double timeout) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  // Note(gChiou): This is a weird case where we need to do some calculations in a different order from the rest
  std::map<std::string, double> m;
  if (!joint_group_->getVariableDefaultPositions(joint_seed, m))
  {
    throw JointSeedException(joint_seed + " is not a named state defined in the SRDF / URDF");
  }

  std::vector<double> joints;
  for (auto it = m.begin(); it != m.end(); ++it)
  {
    joints.push_back(it->second);
  }

  auto pose_rel_robot = this->transformToBase(pose, frame_to_robot_base);
  Eigen::Isometry3d custom_tool_to_moveit_tool_eigen;
  tf::transformMsgToEigen(custom_tool_to_moveit_tool.transform, custom_tool_to_moveit_tool_eigen);
  pose_rel_robot = pose_rel_robot * custom_tool_to_moveit_tool_eigen;

  auto point = std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(pose_rel_robot, 0.0));

  if (point)
  {
    if (joint_seed.empty())
    {
      ROS_DEBUG_STREAM("Empty seed passed to collision check, using current state");
      virtual_robot_state_->copyJointGroupPositions(joint_group_->getName(), joints);
    }

    auto joint_traj_point = point->toJointTrajPoint(*this, timeout, joints);
    if (joint_traj_point)
    {
      return this->isInCollision(joint_traj_point->jointPoint());
    }
  }

  return true;
}

bool Robot::isInCollision(const std::vector<double>& joint_point) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  std::vector<double> local_joint_point = joint_point;

  if (joint_point.empty())
  {
    ROS_DEBUG_STREAM("Empty joint point passed to isIncollision, using current state");
    local_joint_point = getJointState();
  }

  virtual_robot_state_->setJointGroupPositions(joint_group_, local_joint_point);
  bool inCollision = planning_scene_->isStateColliding(*virtual_robot_state_, joint_group_->getName());

  return inCollision;
}

bool Robot::isInCollision(const Eigen::Isometry3d& pose, const std::string& frame, double timeout,
                          std::vector<double> joint_seed) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  bool inCollision = true;
  try
  {
    auto pose_rel_robot = this->lookupTransformToBase(frame);
    inCollision = isInCollision(pose, pose_rel_robot, timeout, joint_seed);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_STREAM("IsInCollision failed for arbitrary pose point: " << ex.what());
    inCollision = true;
  }

  return inCollision;
}

bool Robot::isInCollision(const Eigen::Isometry3d& pose, const geometry_msgs::TransformStamped& frame_to_robot_base,
                          double timeout, std::vector<double> joint_seed) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  auto pose_rel_robot = this->transformToBase(pose, frame_to_robot_base);
  auto point = std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(pose_rel_robot, 0.0));

  if (point)
  {
    if (joint_seed.empty())
    {
      ROS_DEBUG_STREAM("Empty seed passed to collision check, using current state");
      virtual_robot_state_->copyJointGroupPositions(joint_group_->getName(), joint_seed);
    }

    auto joint_traj_point = point->toJointTrajPoint(*this, timeout, joint_seed);
    if (joint_traj_point)
    {
      return this->isInCollision(joint_traj_point->jointPoint());
    }
  }

  return true;
}

bool Robot::isReachable(const std::string& name, double timeout, std::vector<double> joint_seed) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  bool reacheable = false;
  try
  {
    std::unique_ptr<TrajectoryPoint> point = lookupTrajectoryPoint(name, 0.0);
    return isReachable(point, timeout, joint_seed);
  }
  catch (...)
  {
    ROS_ERROR_STREAM("Invalid point for reach check");
    return false;
  }
}

bool Robot::isReachable(const Eigen::Isometry3d& pose, const std::string& frame, const std::string& joint_seed,
                        double timeout) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  std::map<std::string, double> m;
  if (!joint_group_->getVariableDefaultPositions(joint_seed, m))
  {
    throw JointSeedException(joint_seed + " is not a named state defined in the SRDF / URDF");
  }

  std::vector<double> joints;
  for (auto it = m.begin(); it != m.end(); ++it)
  {
    joints.push_back(it->second);
  }

  return this->isReachable(pose, frame, timeout, joints);
}

bool Robot::isReachable(const Eigen::Isometry3d& pose, const geometry_msgs::TransformStamped& frame_to_robot_base,
                        const std::string& joint_seed, double timeout) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  std::map<std::string, double> m;
  if (!joint_group_->getVariableDefaultPositions(joint_seed, m))
  {
    throw JointSeedException(joint_seed + " is not a named state defined in the SRDF / URDF");
  }

  std::vector<double> joints;
  for (auto it = m.begin(); it != m.end(); ++it)
  {
    joints.push_back(it->second);
  }

  return this->isReachable(pose, frame_to_robot_base, timeout, joints);
}

bool Robot::isReachable(const Eigen::Isometry3d& pose, const geometry_msgs::TransformStamped& frame_to_robot_base,
                        const geometry_msgs::TransformStamped& custom_tool_to_moveit_tool,
                        const std::string& joint_seed, double timeout) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  // Note(gChiou): This is a weird case where we need to do some calculations in a different order from the rest
  std::map<std::string, double> m;
  if (!joint_group_->getVariableDefaultPositions(joint_seed, m))
  {
    throw JointSeedException(joint_seed + " is not a named state defined in the SRDF / URDF");
  }

  std::vector<double> joints;
  for (auto it = m.begin(); it != m.end(); ++it)
  {
    joints.push_back(it->second);
  }

  auto pose_rel_robot = this->transformToBase(pose, frame_to_robot_base);
  Eigen::Isometry3d custom_tool_to_moveit_tool_eigen;
  tf::transformMsgToEigen(custom_tool_to_moveit_tool.transform, custom_tool_to_moveit_tool_eigen);
  pose_rel_robot = pose_rel_robot * custom_tool_to_moveit_tool_eigen;

  std::unique_ptr<TrajectoryPoint> point =
      std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(pose_rel_robot, 0.0));

  return this->isReachable(point, timeout, joints);
}

bool Robot::isReachable(const Eigen::Isometry3d& pose, const std::string& frame, double timeout,
                        std::vector<double> joint_seed) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  bool reachable = false;
  try
  {
    auto frame_rel_robot = this->lookupTransformToBase(frame);
    reachable = isReachable(pose, frame_rel_robot, timeout, joint_seed);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_STREAM("Reacheability failed for arbitrary pose point: " << ex.what());
    reachable = false;
  }

  return reachable;
}

bool Robot::isReachable(const Eigen::Isometry3d& pose, const geometry_msgs::TransformStamped& frame_to_robot_base,
                        double timeout, std::vector<double> joint_seed) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  auto pose_rel_robot = this->transformToBase(pose, frame_to_robot_base);

  std::unique_ptr<TrajectoryPoint> point =
      std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(pose_rel_robot, 0.0));

  return this->isReachable(point, timeout, joint_seed);
}

bool Robot::isReachable(std::unique_ptr<TrajectoryPoint>& point, double timeout, std::vector<double> joint_seed) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  bool reachable = false;

  if (point)
  {
    if (joint_seed.empty())
    {
      ROS_DEBUG_STREAM("Empty seed passed to reach check, using current state");
      virtual_robot_state_->copyJointGroupPositions(joint_group_->getName(), joint_seed);
    }

    std::unique_ptr<JointTrajectoryPoint> dummy = point->toJointTrajPoint(*this, timeout, joint_seed);
    if (dummy)
    {
      reachable = true;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Invalid point for reach check");
    reachable = false;
  }

  return reachable;
}

void Robot::clearTrajectory(const std::string& traj_name)
{
  trajectory_planner_.clearTrajectory(traj_name);
}

std::vector<moveit_simple::JointTrajectoryPoint> Robot::plan(const std::string traj_name, bool collision_check)
{
  return trajectory_planner_.plan(*this, traj_name, collision_check);
}

control_msgs::FollowJointTrajectoryGoal Robot::toFollowJointTrajectoryGoal(
    const std::vector<moveit_simple::JointTrajectoryPoint>& joint_trajectory_points) const
{
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = joint_group_->getVariableNames();

  std::vector<double> empty(goal.trajectory.joint_names.size(), 0.0);

  for (auto& trajectory_point : joint_trajectory_points)
  {
    trajectory_msgs::JointTrajectoryPoint goal_buffer;
    goal_buffer.positions = trajectory_point.jointPoint();
    goal_buffer.accelerations = empty;
    goal_buffer.effort = empty;
    goal_buffer.velocities = empty;

    ros::Duration ros_time(trajectory_point.time());
    goal_buffer.time_from_start = ros_time;

    goal.trajectory.points.push_back(goal_buffer);
  }

  return goal;
}

int Robot::trajCollisionCheck(control_msgs::FollowJointTrajectoryGoal& goal, bool collision_check)
{
  int collision_count = 0;

  if (collision_check)
  {
    for (std::size_t i = 0; i < goal.trajectory.points.size(); i++)
    {
      if (isInCollision(goal.trajectory.points[i].positions))
      {
        collision_count++;
      }
    }
  }

  return collision_count;
}

void Robot::reconfigureRequest(moveit_simple_dynamic_reconfigure_Config& config, uint32_t level)
{
  params_.fromConfig(config);
  if (params_.speed_modifier > 0.0)
  {
    setSpeedModifier(params_.speed_modifier);
  }
  else
  {
    ROS_WARN_STREAM("Speed modifier should be a positive nunber but it is: " << params_.speed_modifier);
  }
  joint_equality_tolerance_ = params_.joint_equality_tolerance;
}

void Robot::setSpeedModifier(const double speed_modifier)
{
  if (speed_modifier <= 1.0 && speed_modifier > 0.0)
  {
    speed_modifier_ = speed_modifier;
  }
  else if ((speed_modifier > 1.0))
  {
    speed_modifier_ = 1.0;  // MAX allowed parameter value
    ROS_WARN_STREAM("Clamping Speed from " << speed_modifier_ << " to max_speed: [1.0]");
  }
  else
  {
    speed_modifier_ = 0.01;  // MIN allowed parameter value
    ROS_WARN_STREAM("Clamping Speed from " << speed_modifier_ << " to min_speed: [0.1]");
  }
}

double Robot::getSpeedModifier(void) const
{
  return speed_modifier_;
}

bool Robot::toJointTrajectory(const std::string traj_name, std::vector<trajectory_msgs::JointTrajectoryPoint>& points,
                              bool collision_check)
{
  return trajectory_planner_.toJointTrajectory(*this, traj_name, points, collision_check);
}

void Robot::computeIKSolverTransforms()
{
  ROS_INFO_STREAM("Computing transforms between the base/tip frames defined in the SRDF"
                  " and the base/tip frames defined for the IK solver");

  // Check if transform is available first
  tf_buffer_.canTransform(joint_group_->getSolverInstance()->getBaseFrame(), ik_base_frame_, ros::Time::now(),
                          ros::Duration(15.0));

  try
  {
    ROS_INFO_STREAM("Looking up transform from: " << joint_group_->getSolverInstance()->getBaseFrame()
                                                  << " to: " << ik_base_frame_);
    geometry_msgs::TransformStamped transform_msg;
    transform_msg = tf_buffer_.lookupTransform(ik_base_frame_, joint_group_->getSolverInstance()->getBaseFrame(),
                                               ros::Time::now(), ros::Duration(5.0));
    tf::transformMsgToEigen(transform_msg.transform, srdf_base_to_ik_base_);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM("Failed to calculate transform from: " << joint_group_->getSolverInstance()->getBaseFrame()
                                                            << " to: " << ik_base_frame_);
    throw ex;
  }

  // Check if transform is available first
  tf_buffer_.canTransform(ik_tip_frame_, joint_group_->getSolverInstance()->getTipFrame(), ros::Time::now(),
                          ros::Duration(15.0));

  try
  {
    ROS_INFO_STREAM("Looking up transform from: " << ik_tip_frame_
                                                  << " to: " << joint_group_->getSolverInstance()->getTipFrame());
    geometry_msgs::TransformStamped transform_msg;
    transform_msg = tf_buffer_.lookupTransform(joint_group_->getSolverInstance()->getTipFrame(), ik_tip_frame_,
                                               ros::Time::now(), ros::Duration(5.0));
    tf::transformMsgToEigen(transform_msg.transform, ik_tip_to_srdf_tip_);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM("Failed to calculate transfrom from: " << ik_tip_frame_ << " to: "
                                                            << joint_group_->getSolverInstance()->getTipFrame());
    throw ex;
  }

  ROS_INFO_STREAM("Transforms between the base/tip frames defined in the SRDF"
                  " and the base/tip frames defined for the IK solver have been computed");
}

bool Robot::jointInterpolation(const std::unique_ptr<TrajectoryPoint>& traj_point,
                               std::vector<trajectory_msgs::JointTrajectoryPoint>& points, unsigned int num_steps,
                               bool collision_check)
{
  return trajectory_planner_.jointInterpolation(*this, traj_point, points, num_steps, collision_check);
}

bool Robot::cartesianInterpolation(const std::unique_ptr<TrajectoryPoint>& traj_point,
                                   std::vector<trajectory_msgs::JointTrajectoryPoint>& points, unsigned int num_steps,
                                   bool collision_check)
{
  return trajectory_planner_.cartesianInterpolation(*this, traj_point, points, num_steps, collision_check);
}

void Robot::interpolate(const std::unique_ptr<JointTrajectoryPoint>& from,
                        const std::unique_ptr<JointTrajectoryPoint>& to, double t,
                        std::unique_ptr<JointTrajectoryPoint>& point) const
{
  std::vector<double> start_joint_point = from->jointPoint();
  double start_time = from->time();

  std::vector<double> target_joint_point = to->jointPoint();
  double target_time = to->time();

  if (start_joint_point.size() == target_joint_point.size())
  {
    std::vector<double> joint_point = interpolate(start_joint_point, target_joint_point, t);
    double time = t * target_time + (1 - t) * start_time;
    point = std::unique_ptr<JointTrajectoryPoint>(new JointTrajectoryPoint(joint_point, time, ""));
  }
  else
  {
    ROS_ERROR_STREAM("Interpolation between these two joint points is not possible"
                     << "as start and target points have different sizes with start: " << start_joint_point.size()
                     << " joints and target: " << target_joint_point.size() << "joints respectively");

    point = std::unique_ptr<JointTrajectoryPoint>(nullptr);
  }
}

std::vector<double> Robot::interpolate(const std::vector<double>& from, const std::vector<double>& to, double t) const
{
  std::vector<double> joint_point(from.size());
  for (std::size_t i = 0; i < from.size(); ++i)
  {
    joint_point[i] = t * to[i] + (1 - t) * from[i];
  }

  return joint_point;
}

void Robot::interpolate(const std::unique_ptr<CartTrajectoryPoint>& from,
                        const std::unique_ptr<CartTrajectoryPoint>& to, double t,
                        std::unique_ptr<CartTrajectoryPoint>& point) const
{
  Eigen::Isometry3d start_pose = from->pose();
  double start_time = from->time();

  Eigen::Isometry3d target_pose = to->pose();
  double target_time = to->time();

  Eigen::Isometry3d pose = interpolate(start_pose, target_pose, t);
  double time = t * target_time + (1 - t) * start_time;

  point = std::unique_ptr<CartTrajectoryPoint>(new CartTrajectoryPoint(pose, time, ""));
}

Eigen::Isometry3d Robot::interpolate(const Eigen::Isometry3d& from, const Eigen::Isometry3d& to, double t) const
{
  Eigen::Quaterniond from_quaternion(from.rotation());
  Eigen::Quaterniond to_quaternion(to.rotation());
  Eigen::Isometry3d pose(from_quaternion.slerp(t, to_quaternion));
  pose.translation() = t * to.translation() + (1 - t) * from.translation();
  return pose;
}

bool Robot::isConfigChange(const std::vector<double> jp1, const std::vector<double> jp2) const
{
  // The maximum allowed change in joint value
  const double MAX_JOINT_CHANGE = 90.0 * M_PI / 180.0;
  // The number of joints allowed to exeed the maximum joint change.
  const int MAX_JOINT_CHANGE_COUNT = 1;
  if (jp1.size() != jp2.size())
  {
    ROS_ERROR_STREAM("Cannot check for config change, size mismatch");
    return true;
  }
  size_t j_change_count = 0;
  for (size_t ii = 0; ii < jp1.size(); ii++)
  {
    double joint_change = std::abs(jp1[ii] - jp2[ii]);
    if (joint_change > MAX_JOINT_CHANGE)
    {
      ROS_INFO_STREAM("Joint[" << ii << "] change of " << joint_change << " exceeds limit: " << MAX_JOINT_CHANGE);
      j_change_count++;
    }
    if (j_change_count > MAX_JOINT_CHANGE_COUNT)
    {
      ROS_WARN_STREAM("Possible config change detected, with " << j_change_count
                                                               << " joints exceeding max chage: " << MAX_JOINT_CHANGE);
      return true;
    }
  }
  return false;
}

void Robot::updateRvizRobotState(const Eigen::Isometry3d& pose, const std::string& in_frame,
                                 const std::string& joint_seed, double timeout) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  std::map<std::string, double> m;
  if (!joint_group_->getVariableDefaultPositions(joint_seed, m))
  {
    throw JointSeedException(joint_seed + " is not a named state defined in the SRDF / URDF");
  }

  std::vector<double> joints;
  for (auto it = m.begin(); it != m.end(); ++it)
  {
    joints.push_back(it->second);
  }

  this->updateRvizRobotState(pose, in_frame, joints, timeout);
}

void Robot::updateRvizRobotState(const Eigen::Isometry3d& pose, const std::string& in_frame,
                                 std::vector<double> joint_seed, double timeout) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  try
  {
    auto frame_rel_robot = this->lookupTransformToBase(in_frame);
    this->updateRvizRobotState(pose, frame_rel_robot, joint_seed, timeout);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_STREAM("Unable to find transform to: " << ik_base_frame_ << " for frame: " << in_frame
                                                    << " exception: " << ex.what());
  }
}

void Robot::updateRvizRobotState(const Eigen::Isometry3d& pose,
                                 const geometry_msgs::TransformStamped& frame_to_robot_base,
                                 const std::string& joint_seed, double timeout) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  std::map<std::string, double> m;
  if (!joint_group_->getVariableDefaultPositions(joint_seed, m))
  {
    throw JointSeedException(joint_seed + " is not a named state defined in the SRDF / URDF");
  }

  std::vector<double> joints;
  for (auto it = m.begin(); it != m.end(); ++it)
  {
    joints.push_back(it->second);
  }

  this->updateRvizRobotState(pose, frame_to_robot_base, joints, timeout);
}

void Robot::updateRvizRobotState(const Eigen::Isometry3d& pose,
                                 const geometry_msgs::TransformStamped& frame_to_robot_base,
                                 std::vector<double> joint_seed, double timeout) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);
  auto pose_rel_robot = this->transformToBase(pose, frame_to_robot_base);
  auto point = std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(pose_rel_robot, 0.0));
  auto update_rviz = point->toJointTrajPoint(*this, timeout, joint_seed);
}

geometry_msgs::TransformStamped Robot::lookupTransformMoveitToolAndCustomTool(const std::string tool_frame)
{
  std::lock_guard<std::recursive_mutex> guard(m_);
  auto moveit_tool = joint_group_->getSolverInstance()->getTipFrame();

  return this->lookupTransformBetweenFrames(tool_frame, moveit_tool);
}

geometry_msgs::TransformStamped Robot::lookupTransformBetweenFrames(const std::string& target_frame,
                                                                    const std::string& source_frame) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);
  try
  {
    auto transform = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(5.0));
    return transform;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM("TF transform lookup between target frame: " << target_frame << " and source frame: "
                                                                  << source_frame << " failed");
    throw ex;
  }
}

geometry_msgs::TransformStamped Robot::lookupTransformToBase(const std::string& in_frame) const
{
  geometry_msgs::TransformStamped frame_rel_robot_msg;

  try
  {
    frame_rel_robot_msg = tf_buffer_.lookupTransform(in_frame, ik_base_frame_, ros::Time::now(), ros::Duration(5.0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_STREAM("Transform lookup from: " << in_frame << " into robot base: " << ik_base_frame_
                                              << "::" << ex.what());
    throw ex;
  }

  return frame_rel_robot_msg;
}

Eigen::Isometry3d Robot::transformToBase(const Eigen::Isometry3d& in, const std::string& in_frame) const
{
  geometry_msgs::TransformStamped frame_rel_robot_msg;

  try
  {
    frame_rel_robot_msg = this->lookupTransformToBase(in_frame);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_STREAM("Transform lookup from: " << in_frame << " into robot base: " << ik_base_frame_
                                              << "::" << ex.what());
    throw ex;
  }

  return this->transformToBase(in, frame_rel_robot_msg);
}

Eigen::Isometry3d Robot::transformToBase(const Eigen::Isometry3d& in,
                                       const geometry_msgs::TransformStamped& transform_msg) const
{
  Eigen::Isometry3d out;

  Eigen::Isometry3d frame_rel_robot;
  tf::transformMsgToEigen(transform_msg.transform, frame_rel_robot);
  out = frame_rel_robot.inverse() * in;

  return out;
}

bool Robot::getFK(const std::vector<double>& joint_point, Eigen::Isometry3d& pose) const
{
  virtual_robot_state_->setJointGroupPositions(joint_group_, joint_point);
  Eigen::Isometry3d ik_base_to_ik_tip = virtual_robot_state_->getFrameTransform(ik_tip_frame_);
  pose = srdf_base_to_ik_base_ * ik_base_to_ik_tip * ik_tip_to_srdf_tip_.inverse();
  return true;
}

bool Robot::getIK(const Eigen::Isometry3d pose, const std::vector<double>& seed, std::vector<double>& joint_point,
                  double timeout) const
{
  virtual_robot_state_->setJointGroupPositions(joint_group_, seed);
  return getIK(pose, joint_point, timeout);
}

bool Robot::getIK(const Eigen::Isometry3d pose, std::vector<double>& joint_point, double timeout) const
{
  Eigen::Isometry3d ik_tip_pose = pose * ik_tip_to_srdf_tip_;
  if (virtual_robot_state_->setFromIK(joint_group_, ik_tip_pose, timeout))
  {
    virtual_robot_state_->copyJointGroupPositions(joint_group_->getName(), joint_point);
    virtual_robot_state_->update();

    virtual_visual_tools_->publishRobotState(virtual_robot_state_, rviz_visual_tools::PURPLE);
    virtual_visual_tools_->publishContactPoints(*virtual_robot_state_, planning_scene_.get());
    virtual_visual_tools_->trigger();
    return true;
  }

  return false;
}

bool Robot::isNearSingular(const std::vector<double>& joint_point) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  // Threshold determined by keeping axes 4 and 6 at 3 degrees of being aligned
  double threshold = 5e-2;
  std::vector<double> local_joint_point = joint_point;

  // Reference point for Jacobian
  Eigen::Vector3d reference_point(0, 0, 0);
  if (joint_point.empty())
  {
    ROS_DEBUG_STREAM("Empty joint point passed to isNearSingular, using current state");
    local_joint_point = getJointState();
  }

  virtual_robot_state_->setJointGroupPositions(joint_group_, local_joint_point);

  Eigen::MatrixXd jacobian;
  if (virtual_robot_state_->getJacobian(joint_group_, joint_group_->getLinkModels().back(), reference_point, jacobian,
                                        false))
  {
    ROS_INFO_STREAM("Jacobian" << jacobian.matrix());
    ROS_INFO_STREAM("Determinant" << fabs((jacobian * jacobian.transpose()).determinant()));

    // Check for determinant of J*J'
    if (fabs((jacobian * jacobian.transpose()).determinant()) <= threshold)
    {
      ROS_WARN_STREAM("Given configuration " << local_joint_point << " is near Singularity");
      return true;
    }
    else
    {
      ROS_INFO_STREAM("Given configuration " << local_joint_point << " is away from any singularity");
      return false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Jacobian not found for " << local_joint_point << "joint group " << joint_group_->getName()
                                               << " is not a chain");
    return false;
  }
}

std::vector<double> Robot::getJointState(void) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  ros::spinOnce();
  std::vector<double> current_joint_positions;

  virtual_robot_state_->update();
  virtual_robot_state_->copyJointGroupPositions(joint_group_->getName(), current_joint_positions);

  return current_joint_positions;
}
}  // namespace moveit_simple
