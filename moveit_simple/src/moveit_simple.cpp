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
#include "prettyprint/prettyprint.hpp"
#include "ros/console.h"
#include "eigen_conversions/eigen_msg.h"

namespace moveit_simple
{



Robot::Robot(const ros::NodeHandle & nh, const std::string &robot_description,
             const std::string &group_name):
  action_("joint_trajectory_action", true),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  nh_(nh)
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

  //TODO: How to handle action server and other failures in the constructor
  // Perhaps move any items that can fail our of the constructor into an init
  // function with a proper return
  if( !action_.isServerConnected() )
  {
    ROS_ERROR_STREAM("Failed to connect to joint trajectory action server: ");
  }


  return;
}



void Robot::addTrajPoint(const std::string & traj_name, const Eigen::Affine3d pose,
                         const std::string & frame, double time,
                         const InterpolationType & type, const unsigned int num_steps,
                         const std::string & point_name)
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  ROS_INFO_STREAM("Attempting to add " << point_name << " to " << traj_name << "relative to"
                  << frame << " at time " << time);
  try
  {
    Eigen::Affine3d pose_rel_robot = transformToBase(pose, frame);
    std::unique_ptr<TrajectoryPoint> point =
      std::unique_ptr<TrajectoryPoint>(new CartTrajectoryPoint(pose_rel_robot, time, point_name));
    traj_info_map_[traj_name].push_back({std::move(point), type, num_steps});
  }
  catch(tf2::TransformException &ex)
  {
    ROS_ERROR_STREAM("Add to trajectory failed for arbitrary pose point: " << ex.what());
    throw ex;
  }
}




void Robot::addTrajPoint(const std::string & traj_name, const std::string & point_name,
                         double time, const InterpolationType & type,
                         const unsigned int num_steps)
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  ROS_INFO_STREAM("Attempting to add " << point_name << " to " << traj_name
                 << " at time " << time);
  try
  {
    std::unique_ptr<TrajectoryPoint> point = lookupTrajectoryPoint(point_name, time);
    traj_info_map_[traj_name].push_back({std::move(point), type, num_steps});
  }
  catch ( std::invalid_argument &ia )
  {
    ROS_ERROR_STREAM("Invalid point " << point_name << " to add to " << traj_name);
    throw ia;
  }
  catch ( tf2::TransformException &ex )
  {
    ROS_ERROR_STREAM(" TF transform failed for " << point_name << " to add to " << traj_name);
    throw ex;
  }
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
      throw ex;
    }
  }

  else
  {
    ROS_ERROR_STREAM("Failed to find point " << name << ", consider implementing more look ups");
    throw std::invalid_argument("Failed to find point: " + name);
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


bool Robot::getPose(const std::vector<double> & joint_point,
                                  Eigen::Affine3d & pose) const
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  return getFK(joint_point, pose);
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
    ROS_WARN_STREAM("IsInCollision failed for arbitrary pose point: " << ex.what());
    inCollision = true;
  }
  return inCollision;
}


bool Robot::isReachable(const std::string & name, double timeout,
                        std::vector<double> joint_seed) const
{
  bool reacheable = false;
  std::lock_guard<std::recursive_mutex> guard(m_);
  try
  {
  std::unique_ptr<TrajectoryPoint> point = lookupTrajectoryPoint(name, 0.0);
  return isReachable(point, timeout, joint_seed );
  }
  catch( ... )
  {
  ROS_ERROR_STREAM("Invalid point for reach check");
  return false;
  }
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
    ROS_WARN_STREAM("Reacheability failed for arbitrary pose point: " << ex.what());
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

  traj_info_map_.erase(traj_name);
}



void Robot::execute(const std::string traj_name)
{
  std::lock_guard<std::recursive_mutex> guard(m_);

  const double TIMEOUT_SCALE = 1.25;  //scales time to wait for action timeout.
  bool success = false;
  if ( traj_info_map_.count(traj_name) )
  {
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = joint_group_->getVariableNames();
    if ( toJointTrajectory(traj_name, goal.trajectory.points) )
    {
      ros::Duration traj_time =
          goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start;
      ros::Duration timeout(TIMEOUT_SCALE * traj_time.toSec());
      if (action_.sendGoalAndWait(goal, timeout) == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO_STREAM("Successfully executed trajectory: " << traj_name);
      }
      else
      {
        ROS_ERROR_STREAM("Trajectory " << traj_name << " failed to exectue");
        throw ExecutionFailureException("Execution failed for "+ traj_name);
      }
    }
    else
    {
      ROS_ERROR_STREAM("Failed to convert " << traj_name << " to joint trajectory");
      throw IKFailException("Conversion to joint trajectory failed for " + traj_name);
    }

  }
  else
  {
    ROS_ERROR_STREAM("Trajectoy " << traj_name << " not found");
    throw std::invalid_argument("No trajectory found named " + traj_name);
  }
}

bool Robot::toJointTrajectory(const std::string traj_name,
                       std::vector<trajectory_msgs::JointTrajectoryPoint> & points)
{
  const TrajectoryInfo & traj_info = traj_info_map_[traj_name];

  // The first point in any trajectory is the current pose
  std::vector<double> current_joint_position;
  robot_state_->copyJointGroupPositions(joint_group_->getName(), current_joint_position);
  points.push_back(toJointTrajPtMsg(current_joint_position, 0.0));

  for(size_t i = 0; i < traj_info.size(); ++i)
  {
    const std::unique_ptr<TrajectoryPoint> & traj_point = traj_info[i].point;
    if (traj_info[i].type == interpolation_type::JOINT)
    {
      if(jointInterpolation(traj_point, points, traj_info[i].num_steps))
      {
        ROS_INFO_STREAM("Trajectory successfully added till " << traj_point->name());
      }else{
        ROS_ERROR_STREAM("Conversion to joint trajectory failed for " << traj_name <<
                        " due to IK failure of " << traj_point->name());
        return false;
      }
    }
    else if (traj_info[i].type == interpolation_type::CARTESIAN)
    {
      if (cartesianInterpolation(traj_point, points, traj_info[i].num_steps))
      {
        ROS_INFO_STREAM("Trajectory successfully added till " << traj_point->name());
      }else{
        ROS_ERROR_STREAM("Conversion to joint trajectory failed for " << traj_name <<
                        " before adding " << traj_point->name());
        return false;
      }
    }else{
      ROS_ERROR_STREAM("Unknown interpolation call " << traj_info[i].type);
      return false;
    }
  }
  return true;
}

bool Robot::jointInterpolation(const std::unique_ptr<TrajectoryPoint> & traj_point,
           std::vector<trajectory_msgs::JointTrajectoryPoint> & points,
           unsigned int num_steps)
{
  const double IK_TIMEOUT = 0.250;   //250 ms for IK solving
  trajectory_msgs::JointTrajectoryPoint prev_point_info = points.back();
   std::vector<double> prev_point = prev_point_info.positions;
   double prev_time = prev_point_info.time_from_start.toSec();
   // Convert the previous point stored in points to Joint Trajectory Point
  std::unique_ptr<JointTrajectoryPoint>prev_traj_point =
                               std::unique_ptr<JointTrajectoryPoint>
                               (new JointTrajectoryPoint(prev_point, prev_time, ""));

  std::unique_ptr<JointTrajectoryPoint> target_point;
  if( traj_point->type() != TrajectoryPoint::JOINT )
  {
    const size_t MAX_IK_ATTEMPTS = 2;
    size_t num_attempts = 0;

    while(true)
    {
      num_attempts++;
      target_point = traj_point->toJointTrajPoint(*this, IK_TIMEOUT, prev_point);

      if (target_point)
      {
        if( isConfigChange(prev_point, target_point->jointPoint()) )
        {
          ROS_WARN_STREAM("Configuration change detected in move to/from cart point: ("
                           << traj_point->name() << "), of type: "
                           << traj_point->type() << " to joint trajectory");
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
        ROS_WARN_STREAM("Failed to convert trajectory point:  ("
                       << traj_point->name() << "), of type: "
                       << traj_point->type() << " to joint trajectory");
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
    target_point = traj_point->toJointTrajPoint(*this, IK_TIMEOUT, prev_point);
    if (!target_point)
    {
      ROS_WARN_STREAM("Failed to convernt joint point - this shouldn't happen");
      return false;
    }
  }

  unsigned int points_added = 0;
  for (std::size_t i=1; i<=num_steps+1; ++i)
  {
    double t = (double)i / (double)(num_steps+1);
    std::unique_ptr<JointTrajectoryPoint> new_point;
    interpolate(prev_traj_point,target_point,t,new_point);
    if(new_point)
    {
      points.push_back(toJointTrajPtMsg(*new_point));
      ROS_INFO_STREAM("Appending trajectory point, size: " << points.size());
      points_added++;
      ROS_INFO_STREAM( points_added << " points among " << (num_steps+1) <<
                      " successfully interpolated for " << traj_point->name());
    }else{
       ROS_WARN_STREAM("Conversion to joint trajectory failed at " << points_added << " among "
                        << (num_steps+1) << "points for " << traj_point->name()
                        << ". Clearing points added in unsuccessful attempt");
      for (std::size_t j=0; j<points_added; ++j)
      {
        points.pop_back();
      }
      return false;
    }
  }
  return true;
}


bool Robot::cartesianInterpolation(const std::unique_ptr<TrajectoryPoint> & traj_point,
           std::vector<trajectory_msgs::JointTrajectoryPoint> & points,
           unsigned int num_steps)
{
  trajectory_msgs::JointTrajectoryPoint prev_point_info = points.back();
  // Convert the previous point stored in points to Cartesian Trajectory Point
  std::unique_ptr<TrajectoryPoint>prev_point =  std::unique_ptr<TrajectoryPoint>
(new JointTrajectoryPoint(prev_point_info.positions, prev_point_info.time_from_start.toSec(), ""));
  std::unique_ptr<CartTrajectoryPoint>prev_traj_point =
  prev_point->toCartTrajPoint(*this);
  std::unique_ptr<CartTrajectoryPoint> target_point;
  if (prev_traj_point)
  {
    target_point = traj_point->toCartTrajPoint(*this);
    if (!target_point)
    {
      ROS_ERROR_STREAM("Failed to find FK for " << traj_point->name());
      return false;
    }
  }else{
    ROS_ERROR_STREAM("Failed to find FK for already added point. This is unexpected." );
    return false;
  }
  unsigned int points_added = 0;
  for (std::size_t i=1; i<=num_steps+1; ++i)
  {
    double t = (double)i / (double)(num_steps+1);
    std::unique_ptr<CartTrajectoryPoint> new_point_cart;
    interpolate(prev_traj_point,target_point,t,new_point_cart);

    std::unique_ptr<TrajectoryPoint> new_point =std::unique_ptr<TrajectoryPoint>
               ( new CartTrajectoryPoint(new_point_cart->pose(), new_point_cart->time(), ""));

    // Add new point at the end of Joint Trajectory (named points)
    if(jointInterpolation(new_point, points,(unsigned int) 0))
    {
      points_added++;
      ROS_INFO_STREAM( points_added << " points among " << (num_steps+1) << " added successfullyfor " << traj_point->name());
    }else{
      ROS_WARN_STREAM("Conversion to joint trajectory failed at " << points_added << " among "
                        << (num_steps+1) << "points for " << traj_point->name()
                        << ". Clearing points added in unsuccessful attempt");
      for (std::size_t j=0; j<points_added; ++j)
      {
        points.pop_back();
      }
      return false;
    }
  }
    return true;
}


void Robot::interpolate( const std::unique_ptr<JointTrajectoryPoint>& from,
                         const std::unique_ptr<JointTrajectoryPoint>& to,
                         double t, std::unique_ptr<JointTrajectoryPoint> & point)
{
  std::vector<double> start_joint_point = from->jointPoint();
  double start_time = from->time();

  std::vector<double> target_joint_point = to->jointPoint();
  double target_time = to->time();

  if (start_joint_point.size() == target_joint_point.size())
  {
  std::vector<double> joint_point =
   interpolate(start_joint_point, target_joint_point, t);
  double time = t* target_time + (1-t)*start_time;

  point = std::unique_ptr<JointTrajectoryPoint>(new JointTrajectoryPoint(joint_point, time, ""));
  }else{
  ROS_ERROR_STREAM("Interpolation between these two joint points is not possible" <<
                  "as start and target points have different sizes with start: "
                   << start_joint_point.size() << " joints and target: "
                   << target_joint_point.size() << "joints respectively");
  point = std::unique_ptr<JointTrajectoryPoint>(nullptr);
  }
}


std::vector<double> Robot::interpolate( const std::vector<double> & from,
                                        const std::vector<double> & to,
                                        double t)
{
  std::vector<double> joint_point(from.size());
  for (std::size_t i=0; i< from.size(); ++i)
  {
    joint_point[i] = t*to[i] + (1-t)*from[i];
  }

  return joint_point;
}


void Robot::interpolate( const std::unique_ptr<CartTrajectoryPoint>& from,
                         const std::unique_ptr<CartTrajectoryPoint>& to,
                         double t, std::unique_ptr<CartTrajectoryPoint> & point)
{
  Eigen::Affine3d start_pose = from->pose();
  double start_time = from->time();

  Eigen::Affine3d target_pose = to->pose();
  double target_time = to->time();

  Eigen::Affine3d pose =
   interpolate(start_pose, target_pose, t);
  double time = t* target_time + (1-t)*start_time;

  point = std::unique_ptr<CartTrajectoryPoint>(new CartTrajectoryPoint(pose, time, ""));
}

Eigen::Affine3d Robot::interpolate( const Eigen::Affine3d & from,
                                    const Eigen::Affine3d & to,
                                    double t)
{

  Eigen::Quaterniond from_quaternion(from.rotation());
  Eigen::Quaterniond to_quaternion(to.rotation());
  Eigen::Affine3d pose(from_quaternion.slerp(t, to_quaternion));
  pose.translation() = t * to.translation() + (1 - t) * from.translation();
  return pose;
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




bool Robot::getFK(const std::vector<double> & joint_point,
                  Eigen::Affine3d &pose) const
{
  robot_state_->setJointGroupPositions(joint_group_, joint_point);
  const std::vector<std::string> link_names = joint_group_->getLinkModelNames();
  const std::vector<std::string> active_joints = joint_group_->getActiveJointModelNames();
  const int vc =  (int)robot_state_->getVariableCount();
  if ( active_joints.size() == vc)
  {
    pose = robot_state_->getFrameTransform(link_names.back());
    return true;
  }else{
    return false;
  }
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


std::unique_ptr<JointTrajectoryPoint> JointTrajectoryPoint::toJointTrajPoint(
        const Robot & robot,  double timeout, const std::vector<double> & seed) const
{
  ROS_DEBUG_STREAM("JointTrajectoryPoint: passing through joint trajectory point");
  return std::unique_ptr<JointTrajectoryPoint>(new JointTrajectoryPoint(*this));
}

std::unique_ptr<CartTrajectoryPoint> JointTrajectoryPoint::toCartTrajPoint(
                                   const Robot & robot) const
{
  Eigen::Affine3d pose;

  ROS_DEBUG_STREAM("JointTrajectoryPoint: Calculating FK for Cartesian trajectory point");
  if(robot.getPose(joint_point_, pose))
  {
    return std::unique_ptr<CartTrajectoryPoint>( new CartTrajectoryPoint(pose, time(), name()));
  }
  else
  {
    ROS_WARN_STREAM("Failed to find FK for point: " << name_);
    return std::unique_ptr<CartTrajectoryPoint>(nullptr);
  }
}


std::unique_ptr<JointTrajectoryPoint> CartTrajectoryPoint::toJointTrajPoint(
       const Robot & robot,  double timeout, const std::vector<double> & seed) const
{
  std::vector<double> joints;

  ROS_DEBUG_STREAM("CartTrajectoryPoint: Calculating IK for joint trajectory point");
  if( robot.getJointSolution(pose_, timeout, seed, joints) )
  {
    return std::unique_ptr<JointTrajectoryPoint>( new JointTrajectoryPoint(joints, time(), name()));
  }
  else
  {
    ROS_WARN_STREAM("Failed to find joint solution for point: " << name_);
    return std::unique_ptr<JointTrajectoryPoint>(nullptr);
  }
}

std::unique_ptr<CartTrajectoryPoint> CartTrajectoryPoint::toCartTrajPoint(
    const Robot & robot) const
{
  ROS_DEBUG_STREAM("CartTrajectoryPoint: passing through cartesian trajectory point");
  return std::unique_ptr<CartTrajectoryPoint>(new CartTrajectoryPoint(*this));
}

}
