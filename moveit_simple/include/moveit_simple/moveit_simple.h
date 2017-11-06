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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
class OnlineRobot;
class ExecutionFailureException;
class IKFailException;

namespace interpolation_type
{
  enum InterpolationType
    {
      UNKNOWN = 0,
      JOINT = 1,
      CARTESIAN = 2
    };
}
typedef interpolation_type::InterpolationType InterpolationType;

struct TrajectoryPointInfo {
  std::unique_ptr<TrajectoryPoint> point;
  InterpolationType type;
  unsigned int num_steps;
};
typedef std::vector<TrajectoryPointInfo> TrajectoryInfo;

/**
 * @brief Robot is a wrapper around standard MoveIt objects.  It makes multiple
 assumptions about the type and complexity of the robot.  Assumptions are:
 - single arm manipulator (one joint group)
 - all cartesian poses are of the tool of the robot (could be fixed in the future)
 - point names are stored as joint positions in the SRDF or frame names in the URDF
 - frame transformations outside the URDF are provided by TF
*/
class Robot
{
public:
   Robot(const ros::NodeHandle & nh, const std::string &robot_description,
                                     const std::string &group_name);
   /**
   * @brief isInCollision  returns true if joint_point results in robot config that is
   * in collision with the environment as defined by the URDF.
   * @param joint_point(optional) - joint position of the robot to check
   * If no joint point is provided current position is checked for collision
   * @return
   */
  bool isInCollision(const std::vector<double> & joint_point = std::vector<double>() ) const;

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
   * @brief isReachable - check if point is reacheable.
   * @param name - name of point to check
   * @param joint_seed - (optional) tries to find joint solutions closest to seed
   * @param timeout - (optional) timeout for IK
   * @return
   */
  bool isReachable(const std::string & name, double timeout = 10.0,
                   std::vector<double> joint_seed = std::vector<double>() ) const;


  /**
   * @brief isReachable - check if pose (relative to named frame) is reacheable.
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
   * @brief addTrajPoint - add point to trajectory.
   * @param traj_name - name of trajectory buffer to add point to
   * @param point_name - name of point to add
   * @param time - time from start of trajectory to reach point
   * @param type - Type of interpolation from last point to this point
   * By deafult, it is set to JOINT. Can be set to "CARTESIAN" for cartesian Interpolation
   * @param num_steps - number of points to be interpolated
   * By deafult, it is set to 0 and only adds the given point to the Trajectory
   * @throws <std::invalid_argument> (point_name is not found)
   * @throws <tf2::TransformException> (transform of TF named point_name fails)
   */
  void addTrajPoint(const std::string & traj_name, const std::string & point_name,
                    double time, const InterpolationType & type = interpolation_type::JOINT,
                    const unsigned int num_steps = 0);
  /**
   * @brief Add trajectory point to motion buffer.
   * @param traj_name - name of trajectory buffer to add point to
   * @param pose - pose of point to add
   * @param frame - frame (must be a TF accessible frame) in which pose is defined
   * @param time - time from start of trajectory to reach point
   * @param type - Type of interpolation from last point to this point
   * By deafult, it is set to JOINT. Can be set to "CARTESIAN" for cartesian Interpolation
   * @param num_steps - number of points to be interpolated
   * By deafult, it is set to 0 and only adds the given point to the Trajectory
   * @param point_name - (optional) name of point (used in log messages)
   * @throws <tf2::TransformException> (Transform from frame to robot base failed)
  */
  void addTrajPoint(const std::string & traj_name, const Eigen::Affine3d pose,
                    const std::string & frame, double time,
                    const InterpolationType & type = interpolation_type::JOINT,
                    const unsigned int num_steps = 0,
                    const std::string & point_name = std::string());


  /**
   * @brief addTrajPoint - add point to trajectory.
   * @brief This function supports custom tool frame for adding traj points.
   * @param traj_name - name of trajectory buffer to add point to
   * @param point_name - name of point to add
   * @param custom_tool_frame - frame (must be a TF accessible frame) in which pose is defined
   * @param time - time from start of trajectory to reach point
   * @param type - Type of interpolation from last point to this point
   * By deafult, it is set to JOINT. Can be set to "CARTESIAN" for cartesian Interpolation
   * @param num_steps - number of points to be interpolated
   * By deafult, it is set to 0 and only adds the given point to the Trajectory
   * @throws <std::invalid_argument> (point_name is not found)
   * @throws <tf2::TransformException> (transform of TF named point_name fails)
   */
  void addTrajPoint(const std::string & traj_name, const std::string & point_name,
                    const std::string & custom_tool_frame, 
                    double time, const InterpolationType & type = interpolation_type::JOINT,
                    const unsigned int num_steps = 0);
  /**
   * @brief Add trajectory point to motion buffer.
   * @brief This function supports custom tool frame for adding traj points.
   * @param traj_name - name of trajectory buffer to add point to
   * @param pose - pose of point to add
   * @param pose_frame - frame (must be a TF accessible frame) in which pose is defined
   * @param custom_tool_frame - frame (must be a TF accessible frame) in which pose is defined
   * @param time - time from start of trajectory to reach point
   * @param type - Type of interpolation from last point to this point
   * By deafult, it is set to JOINT. Can be set to "CARTESIAN" for cartesian Interpolation
   * @param num_steps - number of points to be interpolated
   * By deafult, it is set to 0 and only adds the given point to the Trajectory
   * @param point_name - (optional) name of point (used in log messages)
   * @throws <tf2::TransformException> (Transform from frame to robot base failed)
  */
  void addTrajPoint(const std::string & traj_name, const Eigen::Affine3d pose,
                    const std::string & pose_frame, const std::string & custom_tool_frame, 
                    double time, const InterpolationType & type = interpolation_type::JOINT,
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
   * @brief getJointSolution returns joint solution for cartesian pose.
   * @brief This function supports custom tool frame for solving IK.
   * @param pose - desired pose
   * @param custom_tool_frame - frame (must be a TF accessible frame) in which pose is defined
   * @param timeout - ik solver timeout
   * @param attempts - number of IK solve attem
   * @param seed - ik seed
   * @param joint_point - joint solution for pose
   * @return true if joint solution found
   */
  bool getJointSolution(const Eigen::Affine3d &pose, const std::string& custom_tool_frame,
                        double timeout, const std::vector<double> & seed,
                        std::vector<double> & joint_point) const;

  /**
   * @brief getPose finds cartesian pose for the given joint positions.
   * @param joint_point - joint positions
   * @param pose - pose corresponding to joint_pose
   * @return true if pose is found
   */
  bool getPose(const std::vector<double> & joint_point,
             Eigen::Affine3d & pose) const;

  /**
   * @brief getPose finds cartesian pose for the given joint positions.
   * @brief This function supports custom tool frame for solving FK.
   * @param joint_point - joint positions
   * @param pose - pose corresponding to joint_pose
   * @param custom_tool_frame - frame (must be a TF accessible frame) in which pose is defined
   * @return true if pose is found
   */
  bool getPose(const std::vector<double> & joint_point,
             const std::string& custom_tool_frame,
             Eigen::Affine3d & pose) const;

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
                               double t) const;

  /**
   * @brief interpolate - returns the joint point interpolated from \e from joint point
   * towards \e to joint point at time t in [0,1].
   * @param from: initial joint point
   * @param to: final joint point
   * @param t: parameteric time
   */
  std::vector<double> interpolate( const std::vector<double> & from,
                                  const std::vector<double> & to,
                                  double t) const;

  /**
   * @brief isNearSingular: returns true if joint_point results in robot config that
   * is near a singularity configuration.
   * @param joint_point(optional) - joint position of the robot to check
   * If no joint point is provided current position is checked for singularity
   * @throws <std::invalid_argument> (joint_group is not a chain)
   */

  bool isNearSingular(const std::vector<double> & joint_point = std::vector<double>() ) const;

protected:
  Robot();

  Eigen::Affine3d transformToBase(const Eigen::Affine3d &in,
                                         const std::string &in_frame) const;

    /**
   * @brief customToolFrameTF transforms custom frame to moveit_end_link.
   * @param target_pose - goal pose for IK 
   * @param frame_in - Current Frame of Reference as Input
   * @param frame_out - Target Frame of Reference for Transform
   * @param
   */
  Eigen::Affine3d transformPoseBetweenFrames(const Eigen::Affine3d &target_pose, 
                                         const std::string& frame_in,
                                         const std::string& frame_out) const;

  bool toJointTrajectory(const std::string traj_name,
                         std::vector<trajectory_msgs::JointTrajectoryPoint> & points,
                         bool collision_check = false);


  /**
   * @brief  jointInterpolation - joint Interpolation from last added point to
   * current trajectory point(traj_point).
   * @param traj_point: target traj_point for joint interpolation
   * @param points: Vector of Joint Trajectory Point to be executed
   * @param num_steps: number of steps to be interpolated between current point and traj_point
   * @param collision_check - bool to turn check for collision on\off
   * @return true if all the points including traj_point are added to the points.
   */
  bool jointInterpolation(const std::unique_ptr<TrajectoryPoint> & traj_point,
           std::vector<trajectory_msgs::JointTrajectoryPoint> & points,
           const unsigned int num_steps, bool collision_check = false);

  /**
   * @brief  cartesianInterpolation - Cartesian Interpolation from last added point to
   * current trajectory point(traj_point).
   * @param traj_point: target traj_point for cartesian interpolation
   * @param points: Vector of Joint Trajectory Point to be executed
   * @param num_steps: number of steps to be interpolated between current point and traj_point
   * @param collision_check - bool to turn check for collision on\off
   * @return true if all the points including traj_point are added to the points.
   */
  bool cartesianInterpolation(const std::unique_ptr<TrajectoryPoint> & traj_point,
           std::vector<trajectory_msgs::JointTrajectoryPoint> & points,
           const unsigned int num_steps, bool collision_check = false);

  /**
   * @brief interpolate - Cartesian interpolation from \e from point towards \e to
   * point at time t in [0,1] and stores it in \e point.
   * @param from: initial Cartesian point
   * @param to: final Cartesian point
   * @param t: parameteric time
   * @param point: interpolated Cartesian point
   */
  void interpolate( const std::unique_ptr<CartTrajectoryPoint>& from,
                    const std::unique_ptr<CartTrajectoryPoint>& to,
                    double t, 
                    std::unique_ptr<CartTrajectoryPoint> & point) const;

  /**
   * @brief interpolate - Joint interpolation from \e from point towards \e to
   * point at time t in [0,1] and stores it in \e point.
   * @param from: initial Joint point
   * @param to: final Joint point
   * @param t: parameteric time
   * @param point: interpolated Joint point
   */
  void interpolate( const std::unique_ptr<JointTrajectoryPoint>& from,
                    const std::unique_ptr<JointTrajectoryPoint>& to,
                    double t, 
                    std::unique_ptr<JointTrajectoryPoint> & point) const;

  void addTrajPoint(const std::string & traj_name,
                    std::unique_ptr<TrajectoryPoint> &point,
                    const InterpolationType & type = interpolation_type::JOINT,
                    const unsigned int num_steps = 0);

  bool isReachable(std::unique_ptr<TrajectoryPoint> & point, double timeout,
                   std::vector<double> joint_seed = std::vector<double>() ) const;

  /**
  * @brief getJointState - Returns a vector<double> of the
  * robot position from updated from last IK call.
  *
  * @return std::vector<double> current_joint_position
  */
  virtual std::vector<double> getJointState(void) const;

  bool getIK(const Eigen::Affine3d pose, const std::vector<double> & seed,
             std::vector<double> & joint_point, double timeout=1,
              unsigned int attempts=1) const;
  bool getIK(const Eigen::Affine3d pose, std::vector<double> & joint_point,
             double timeout=1, unsigned int attempts=1) const;

  bool getFK(const std::vector<double> & joint_point,
                               Eigen::Affine3d &pose) const;

  std::unique_ptr<TrajectoryPoint> lookupTrajectoryPoint(const std::string & name,
                                                         double time) const;

  std::unique_ptr<TrajectoryPoint> lookupTrajectoryPoint(const std::string & traj_name,
                                                         const std::string & name,
                                                         const std::string & custom_tool_frame, 
                                                         double time) const;

  bool isConfigChange(const std::vector<double> jp1,
                      const std::vector<double> jp2) const;

  trajectory_msgs::JointTrajectoryPoint toJointTrajPtMsg(
      const JointTrajectoryPoint & joint_point) const;

  // Robot internal objects
  std::map<std::string, TrajectoryInfo> traj_info_map_;

  // MoveIt objects
  mutable moveit::core::RobotStatePtr virtual_robot_state_; // for IK calls
  planning_scene::PlanningScenePtr planning_scene_;
  const moveit::core::JointModelGroup *joint_group_;
  robot_model::RobotModelConstPtr robot_model_ptr_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;


  // Visualizations
  moveit_visual_tools::MoveItVisualToolsPtr virtual_visual_tools_;

  // TF for looking up waypoints
  // TODO: Consider swapping out the listener for an action based lookup - much
  // less overhead
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ROS objects
  ros::NodeHandle nh_;
  mutable std::recursive_mutex m_;

};




/**
 * @brief OnlineRobot is a wrapper around standard MoveIt objects.
 It inherits from Robot. Added assumption:
 - motion is executed via a "JointTrajectoryAction" interface
*/

class OnlineRobot : public Robot
{
public:
  OnlineRobot(const ros::NodeHandle & nh, const std::string &robot_description,
        const std::string &group_name);

  /**
   * @brief execute a given trajectory
   * @param traj_name - name of trajectory to be executed (must be filled with
   * prior calls to "addTrajPoint".
   * @param collision_check - bool to turn check for collision on\off
   * @throws <moveit_simple::ExecutionFailureException> (Execution failure)
   * @throws <moveit_simple::IKFailException> (Conversion to joint trajectory failed)
   * @throws <std::invalid_argument> (Trajectory "traj_name" not found)
   * @throws <moveit_simple::CollisionDetected> (One of interpolated point is
   * in Collision with itself or environment)
   */
  void execute(const std::string traj_name, bool collision_check = false);


  /**
  * @brief getJointState - Returns a vector<double> of the
  * current joint positions of the robot from current_robot_state_.
  *
  * @return std::vector<double> current_joint_position
  */
  virtual std::vector<double> getJointState(void) const;

protected:
  OnlineRobot();

  void updateCurrentState(const sensor_msgs::JointStateConstPtr& msg);
  mutable moveit::core::RobotStatePtr current_robot_state_;


  // Visualizations
  moveit_visual_tools::MoveItVisualToolsPtr online_visual_tools_;

  // Move robot action
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_;

  // ROS objects
  ros::Subscriber j_state_sub_;
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


  /**
   * @brief CollisionDetected: An exception class to notify collision
   *
   * This inherits from std::runtime_error.
   * This is an exception class to be thrown when collision is detected
   * at any interpolated point in the tarjectory.
   */

class CollisionDetected: public std::runtime_error
{
public:
  CollisionDetected(const std::string errorDescription) : std::runtime_error(errorDescription) { ; };
};


}