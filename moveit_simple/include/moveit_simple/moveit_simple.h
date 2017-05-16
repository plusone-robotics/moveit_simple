/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016 Shaun Edwards
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


#pragma once

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <mutex>

#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_ros/transform_listener.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

// Dynamic Reconfigure Parameters
#include <dynamic_reconfigure/server.h>
#include <moveit_simple/moveit_simple_Parameters.h>

namespace moveit_simple
{

class TrajectoryPoint;
class JointTrajectoryPoint;
class CartTrajectoryPoint;
class Robot;
typedef std::vector<std::unique_ptr<TrajectoryPoint> > Trajectory;


/**
 * @brief Robot is a wrapper around standard MoveIt objects.  It makes multiple
 assumptions about the type and complexity of the robot.  Assumptions are:
 - single arm manipulator (one joint group)
 - all cartesian poses are of the tool of the robot (could be fixed in the future)
 - point names are stored as joint positions in the SRDF or frame names in the URDF
 - motion is executed via a "JointTrajectoryAction" interface
 - frame transformations outside the URDF are provided by TF
*/
class Robot
{
public:
  Robot(const ros::NodeHandle & nh, const std::string &robot_description,
        const std::string &group_name);

  /**
   * @brief isInCollision  returns true if pose results in robot config that is
   * in collision with the environment as defined by the URDF.
   * @param pose - cartesian pose of the tool to check
   * @param frame - tool pose relative to frame
   * @param timeout - (optional) timeout for IK
   * @param joint_seed (optional) - seed to use
   * @return
   */
  bool isInCollision(const Eigen::Affine3d pose, const std::string & frame,
                     double timeout = 10.0,
                     std::vector<double> joint_seed = std::vector<double>() ) const;
  /**
   * @brief isReachable - check if point is reacheable
   * @param name - name of point to check
   * @param joint_seed - (optional) tries to find joint solutions closest to seed
   * @param timeout - (optional) timeout for IK
   * @return
   */
  bool isReachable(const std::string & name, double timeout = 10.0,
                   std::vector<double> joint_seed = std::vector<double>() ) const;


  /**
   * @brief isReachable - check if pose (relative to named frame) is reacheable
   * @param pose - pose to check
   * @param frame - frame in which pose is expressed
   * @param timeout - (optional) timeout for IK
   * @param joint_seed (optional) - seed to use
   * @return
   */
  bool isReachable(const Eigen::Affine3d & pose, const std::string & frame,
                   double timeout = 10.0,
                   std::vector<double> joint_seed = std::vector<double>() ) const;


  /**
   * @brief addTrajPoint - add point to trajectory
   * @param traj_name - name of trajectory buffer to add point to
   * @param point_name - name of point to add
   * @param time - time from start of trajectory to reach point
   * @return true if point successfullly added
   */
  bool addTrajPoint(const std::string & traj_name, const std::string & point_name,
                    double time);
  /**
   * @brief Add trajectory point to motion buffer
   *
   * @param traj_name - name of trajectory buffer to add point to
   * @param pose - pose of point to add
   * @param frame - frame (must be a TF accessible frame) in which pose is defined
   * @param time - time from start of trajectory to reach point
   * @param point_name - (optional) name of point (used in log messages)
   *
   * @return true if point successfullly added
  */
  bool addTrajPoint(const std::string & traj_name, const Eigen::Affine3d pose,
                    const std::string & frame, double time,
                    const std::string & point_name = std::string());

  /**
   * @brief getJointSolution returns joint solution for cartesian pose.
   * @param pose - desired pose
   * @param timeout - ik solver timeout
   * @param attempts - number of IK solve attem
   * @param seed - ik seed
   * @param joint_point - joint solution for pose
   * @return true if joint solution found
   */
  bool getJointSolution(const Eigen::Affine3d & pose, double timeout,
                        const std::vector<double> & seed,
                        std::vector<double> & joint_point) const;

  /**
   * @brief execute a given trajectory
   * @param traj_name - name of trajectory to be executed (must be filled with
   * prior calls to "addTrajPoint".
   * @return - true if desired trajectory was executed.
   */
  bool execute(const std::string traj_name);
  /**
   * @brief clearTrajectory - clears stored trajectory
   * @param traj_name - trajectory to clear
   */
  void clearTrajectory(const::std::string traj_name);

  /**
   * @brief toJointTrajPtMsg - Converts native joint point (vector + time) to ROS joint
   * trajectory point message type.
   * @param joint_point
   * @param time
   * @return
   */
  static trajectory_msgs::JointTrajectoryPoint toJointTrajPtMsg(
      const std::vector<double> & joint_point, double time);

  /**
   * @brief setSpeedModifier - Setter method for the execution speed modifier of the 
   * execute method.
   * @param speed_modifier
   * @return
   */
  void setSpeedModifier(double speed_modifier);

  /**
   * @brief setSpeedModifier - Getter method for the execution speed modifier of the 
   * execute method.
   * @return speed_modifier_
   */
  double getSpeedModifier(void);

protected:
  Robot();

  Eigen::Affine3d transformToBase(const Eigen::Affine3d &in,
                                         const std::string &in_frame) const;

  bool toJointTrajectory(const std::string traj_name,
                         std::vector<trajectory_msgs::JointTrajectoryPoint> & points);

  bool addTrajPoint(const std::string & traj_name,
                    std::unique_ptr<TrajectoryPoint> &point);

  bool isReachable(std::unique_ptr<TrajectoryPoint> & point, double timeout,
                   std::vector<double> joint_seed = std::vector<double>() ) const;


  bool getIK(const Eigen::Affine3d pose, const std::vector<double> & seed,
             std::vector<double> & joint_point, double timeout=1,
              unsigned int attempts=1) const;
  bool getIK(const Eigen::Affine3d pose, std::vector<double> & joint_point,
             double timeout=1, unsigned int attempts=1) const;

  std::unique_ptr<TrajectoryPoint> lookupTrajectoryPoint(const std::string & name,
                                                              double time) const;

  bool isConfigChange(const std::vector<double> jp1,
                      const std::vector<double> jp2) const;

  trajectory_msgs::JointTrajectoryPoint toJointTrajPtMsg(
      const JointTrajectoryPoint & joint_point) const;

  void updateState(const sensor_msgs::JointStateConstPtr& msg);

  void reconfigureRequest(moveit_simple_Config &config, uint32_t level);

  // Robot internal objects
  std::map<std::string, Trajectory> traj_map_;

  // MoveIt objects
  mutable moveit::core::RobotStatePtr robot_state_;
  planning_scene::PlanningScenePtr planning_scene_;
  const moveit::core::JointModelGroup *joint_group_;
  robot_model::RobotModelConstPtr robot_model_ptr_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;

  // Visualizations
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Move robot action
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_;

  // TF for looking up waypoints
  // TODO: Consider swapping out the listener for an action based lookup - much
  // less overhead
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ROS objects
  ros::NodeHandle nh_;
  ros::Subscriber j_state_sub_;
  mutable std::recursive_mutex m_;

  // Dynamic Reconfigure
  double speed_modifier_;

  moveit_simple::moveit_simple_Parameters params_;
  dynamic_reconfigure::Server
       <moveit_simple::moveit_simple_Config> dynamic_reconfig_server_; 
};




class TrajectoryPoint
{
public:
  friend class Robot;
  virtual ~TrajectoryPoint() {}

  const double time() const
  {
    return t_;
  }
  const std::string & name() const
  {
    return name_;
  }




protected:
  enum PointType
     {
        UNKNOWN = 0,
        JOINT = 1,
        CARTESIAN = 2
     };


  TrajectoryPoint(std::string name, double t, PointType type): name_(name),t_(t), type_(type){}

  const PointType & type() const
  {
    return type_;
  }

  virtual std::unique_ptr<JointTrajectoryPoint> toJointTrajPoint(
      const Robot & robot,  double timeout, const std::vector<double> & seed) const=0;

  double t_;
  std::string name_;
  PointType type_;


};




class JointTrajectoryPoint : public TrajectoryPoint
{
public:
  JointTrajectoryPoint() : TrajectoryPoint("", 0.0, PointType::JOINT) {}
  virtual ~JointTrajectoryPoint() {}

  JointTrajectoryPoint(std::vector<double> & joint_point, double t,
                       std::string name = std::string()) :
    TrajectoryPoint(name, t, PointType::JOINT),
    joint_point_(joint_point)
  {
  }

  const std::vector<double> & jointPoint() const
  {
    return joint_point_;
  }

protected:

  virtual std::unique_ptr<JointTrajectoryPoint> toJointTrajPoint(
      const Robot & robot,  double timeout, const std::vector<double> & seed) const
  {
    ROS_DEBUG_STREAM("JointTrajectoryPoint: passing through joint trajectory point");
    return std::unique_ptr<JointTrajectoryPoint>(new JointTrajectoryPoint(*this));
  }

private:
  std::vector<double> joint_point_;
};




class CartTrajectoryPoint : public TrajectoryPoint
{
public:
  CartTrajectoryPoint() : TrajectoryPoint("", 0.0, PointType::CARTESIAN) {}
  CartTrajectoryPoint(const Eigen::Affine3d pose, const double t,
                      std::string name = std::string()):
    TrajectoryPoint(name, t, PointType::CARTESIAN),
    pose_(pose)
  {
  }

  virtual ~CartTrajectoryPoint() {}

  const Eigen::Affine3d & pose() const
  {
    return pose_;
  }

protected:

  virtual std::unique_ptr<JointTrajectoryPoint> toJointTrajPoint(
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

private:
  Eigen::Affine3d pose_;
};

}
