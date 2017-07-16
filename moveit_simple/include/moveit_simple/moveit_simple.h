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



namespace moveit_simple
{

class TrajectoryPoint;
class JointTrajectoryPoint;
class CartTrajectoryPoint;
class Robot;
typedef std::vector<std::unique_ptr<TrajectoryPoint>> Trajectory;
typedef std::pair<std::vector<std::string>, std::vector<unsigned int>> Trajectory_info;
class ExecutionFailureException;
class IKFailException;


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
   * @param traj_type - Type of Trajectory from last point to this point
   * By deafult, it is set to JOINT. Can be set to "CARTESIAN" for cartesian Interpolation
   * @throws <std::invalid_argument> (point_name is not found)
   * @throws <tf2::TransformException> (transform of TF named point_name fails)
   */
  void addTrajPoint(const std::string & traj_name, const std::string & point_name,
                    double time, const std::string & traj_type = "JOINT",
                    const unsigned int num_steps = 0);
  /**
   * @brief Add trajectory point to motion buffer
   *
   * @param traj_name - name of trajectory buffer to add point to
   * @param pose - pose of point to add
   * @param frame - frame (must be a TF accessible frame) in which pose is defined
   * @param time - time from start of trajectory to reach point
   * @param traj_type - Type of Trajectory from last point to this point
   * By deafult, it is set to JOINT. Can be set to "CARTESIAN" for cartesian Interpolation
   * @param point_name - (optional) name of point (used in log messages)
   * @throws <tf2::TransformException> (Transform from frame to robot base failed)
  */
  void addTrajPoint(const std::string & traj_name, const Eigen::Affine3d pose,
                    const std::string & frame, double time,
                    const std::string & traj_type = "JOINT",
                    const unsigned int num_steps = 0,
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
   * @brief getPosition finds cartesian pose for the given joint positions.
   * @param joint_point - joint positions
   * @param pose - pose corresponding to joint_pose
   * @return true if pose is found
   */
  bool getPosition(const std::vector<double> & joint_point,
             Eigen::Affine3d & pose) const;

  /**
   * @brief execute a given trajectory
   * @param traj_name - name of trajectory to be executed (must be filled with
   * prior calls to "addTrajPoint".
   * @throws <moveit_simple::ExecutionFailureException> (Execution failure)
   * @throws <moveit_simple::IKFailException> (Conversion to joint trajectory failed)
   * @throws <std::invalid_argument> (Trajectory "traj_name" not found)
   */
  void execute(const std::string traj_name);
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
   * @brief interpolate - returns the pose interpolated from \e from pose towards \e to
   * pose at time t in [0,1].
   * @param from: initial pose
   * @param to: final pose
   * @param t: parameteric time
   */
  Eigen::Affine3d interpolate( const Eigen::Affine3d & from,
                               const Eigen::Affine3d & to,
                               double t);


protected:
  Robot();

  Eigen::Affine3d transformToBase(const Eigen::Affine3d &in,
                                         const std::string &in_frame) const;

  bool toJointTrajectory(const std::string traj_name,
                         std::vector<trajectory_msgs::JointTrajectoryPoint> & points);


  /**
   * @brief  jointInterpolation - joint Interpolation from last added point to
   * current trajectory point.
   * @param traj_point: next traj_point till which joint interpolation is required.
   * @param points: Joint Trajectory Point to be executed
   * @return true if traj_point is successfully added to the points.
   */
  bool jointInterpolation(const std::unique_ptr<TrajectoryPoint> & traj_point,
           std::vector<trajectory_msgs::JointTrajectoryPoint> & points);


  /**
   * @brief  cartesianInterpolation - Cartesian Interpolation from last added point to
   * current trajectory point.
   * @param traj_point: next traj_point till which cartesian interpolation is required.
   * @param points: Joint Trajectory Point to be executed
   * @param num_steps: number of steps to be interpolated between current point and traj_point
   * @return true if all the points including traj_point are added to the points.
   */
  bool cartesianInterpolation(const std::unique_ptr<TrajectoryPoint> & traj_point,
           std::vector<trajectory_msgs::JointTrajectoryPoint> & points,
           const unsigned int num_steps);

  /**
   * @brief interpolate - interpolates from \e from point towards \e to
   * point at time t in [0,1] and stores it in \e point.
   * @param from: initial point
   * @param to: final point
   * @param t: parameteric time
   * @param point: interpolated point
   */
  void interpolate( const std::unique_ptr<CartTrajectoryPoint>& from,
                         const std::unique_ptr<CartTrajectoryPoint>& to,
                         double t, std::unique_ptr<TrajectoryPoint> & point);

  void addTrajPoint(const std::string & traj_name,
                    std::unique_ptr<TrajectoryPoint> &point);

  bool isReachable(std::unique_ptr<TrajectoryPoint> & point, double timeout,
                   std::vector<double> joint_seed = std::vector<double>() ) const;


  bool getIK(const Eigen::Affine3d pose, const std::vector<double> & seed,
             std::vector<double> & joint_point, double timeout=1,
              unsigned int attempts=1) const;
  bool getIK(const Eigen::Affine3d pose, std::vector<double> & joint_point,
             double timeout=1, unsigned int attempts=1) const;

  bool getFK(const std::vector<double> & joint_point,
                               Eigen::Affine3d &pose) const;

  std::unique_ptr<TrajectoryPoint> lookupTrajectoryPoint(const std::string & name,
                                                              double time) const;

  bool isConfigChange(const std::vector<double> jp1,
                      const std::vector<double> jp2) const;

  trajectory_msgs::JointTrajectoryPoint toJointTrajPtMsg(
      const JointTrajectoryPoint & joint_point) const;

  void updateState(const sensor_msgs::JointStateConstPtr& msg);


  // Robot internal objects
  std::map<std::string, Trajectory> traj_map_;
  std::map<std::string, Trajectory_info> traj_info_map_;

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

  virtual std::unique_ptr<CartTrajectoryPoint> toCartTrajPoint(
      const Robot & robot) const=0;

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
      const Robot & robot,  double timeout, const std::vector<double> & seed) const;

  virtual std::unique_ptr<CartTrajectoryPoint> toCartTrajPoint(
      const Robot & robot) const;

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
      const Robot & robot,  double timeout, const std::vector<double> & seed) const;

  virtual std::unique_ptr<CartTrajectoryPoint> toCartTrajPoint(
      const Robot & robot) const;

private:
  Eigen::Affine3d pose_;
};


  /**
   * @brief ExecutionFailureException: An exception class to notify
   * execution failure
   *
   * This inherits from std::runtime_error.
   * This is an exception class to be thrown when sendGoalAndWait method
   * has failed execution.
   */

class ExecutionFailureException: public std::runtime_error
{ 
public:
  ExecutionFailureException(const std::string errorDescription) : std::runtime_error(errorDescription) { ; };
};


  /**
   * @brief IKFailException: An exception class to notify IK failure
   *
   * This inherits from std::runtime_error.
   * This is an exception class to be thrown when IK call fails to return
   * joint solution.
   */

class IKFailException: public std::runtime_error
{ 
public:
  IKFailException(const std::string errorDescription) : std::runtime_error(errorDescription) { ; };
};


}
