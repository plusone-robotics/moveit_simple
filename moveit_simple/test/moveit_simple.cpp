/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
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

#include <moveit_simple/moveit_simple.h>
#include "prettyprint/prettyprint.hpp"
#include <gtest/gtest.h>

using testing::Types;

namespace moveit_simple_test
{

/**
 * @brief UserRobotTest is a fixture for testing public methods.
 * Objects that can be directly used inside the test
 - robot pointer for moveit_simple::Robot 
*/
class UserRobotTest : public ::testing::Test
{
protected:
  std::unique_ptr<moveit_simple::Robot> robot;
  virtual void SetUp()
  {      
  robot = std::unique_ptr<moveit_simple::Robot> (new moveit_simple::Robot
                  (ros::NodeHandle(), "robot_description", "manipulator"));
  ros::Duration(2.0).sleep();  //wait for tf tree to populate
  }
  virtual void TearDown()
  {}
};

// Calling the protected methods from Robot for test
class DeveloperRobot: public moveit_simple::Robot
{
public:
  using moveit_simple::Robot::Robot;
  using moveit_simple::Robot::addTrajPoint;
  using moveit_simple::Robot::toJointTrajectory;
  using moveit_simple::Robot::interpolate;
  using moveit_simple::Robot::jointInterpolation;
  using moveit_simple::Robot::cartesianInterpolation;
};
  
/**
 * @brief DeveloperRobotTest is a fixture for testing public methods.
 * Objects that can be directly used inside the test
 - robot pointer for DeveloperRobot
*/

class DeveloperRobotTest : public ::testing::Test
{
protected:
  std::unique_ptr<DeveloperRobot> robot2;
  virtual void SetUp()
  {      
  robot2 = std::unique_ptr<DeveloperRobot> (new DeveloperRobot
                  (ros::NodeHandle(), "robot_description", "manipulator"));
  ros::Duration(2.0).sleep();  //wait for tf tree to populate
  }
  virtual void TearDown()
  {}
};


TEST_F(UserRobotTest, reachability)
{
  const Eigen::Affine3d pose = Eigen::Affine3d::Identity(); 

  ROS_INFO_STREAM("Testing reachability of unknown point, should fail");
  EXPECT_FALSE(robot->isReachable("unknown_name"));
  EXPECT_FALSE(robot->isReachable(pose, "random_link"));
  EXPECT_TRUE(robot->isReachable(pose, "tool0"));


  ROS_INFO_STREAM("Testing reach of points");
  ASSERT_TRUE(robot->isReachable("home"));        //stored in the SRDF
  ASSERT_TRUE(robot->isReachable("waypoint1"));   //stored in the URDF
  ASSERT_TRUE(robot->isReachable("waypoint2"));   //stored in the URDF
  ASSERT_TRUE(robot->isReachable("waypoint3"));   //stored in the URDF
  ASSERT_TRUE(robot->isReachable("tf_pub1"));     //stored published externally
  ASSERT_FALSE(robot->isReachable("waypoint4"));  //stored in the URDF
}


TEST_F(UserRobotTest, add_trajectory)
{
  const std::string TRAJECTORY_NAME("traj1");
  const Eigen::Affine3d pose = Eigen::Affine3d::Identity(); 
  const moveit_simple::InterpolationType cart = moveit_simple::interpolation_type::CARTESIAN;
  const moveit_simple::InterpolationType joint = moveit_simple::interpolation_type::JOINT;

  ROS_INFO_STREAM("Testing loading of unknown point, should fail");
  EXPECT_THROW(robot->addTrajPoint("bad_traj", "unknown_name", 1.0),std::invalid_argument);
  EXPECT_THROW(robot->addTrajPoint(TRAJECTORY_NAME, pose, "random_link", 5.0), tf2::TransformException);
  ROS_INFO_STREAM("Testing trajectory adding of points");
  EXPECT_NO_THROW(robot->addTrajPoint(TRAJECTORY_NAME, "home",      0.5));
  EXPECT_NO_THROW(robot->addTrajPoint(TRAJECTORY_NAME, "waypoint1", 1.0, joint, 5));
  EXPECT_NO_THROW(robot->addTrajPoint(TRAJECTORY_NAME, "tf_pub1",   2.0, cart, 8));
  EXPECT_NO_THROW(robot->addTrajPoint(TRAJECTORY_NAME, "waypoint2", 3.0));
  EXPECT_NO_THROW(robot->addTrajPoint(TRAJECTORY_NAME, "waypoint3", 4.0, cart));
  EXPECT_NO_THROW(robot->addTrajPoint(TRAJECTORY_NAME, pose, "tool0", 5.0));
  EXPECT_NO_THROW(robot->execute(TRAJECTORY_NAME));

  EXPECT_NO_THROW(robot->addTrajPoint("traj2", "waypoint4", 4.5));
  EXPECT_THROW(robot->execute("traj2"), moveit_simple::IKFailException);

  EXPECT_THROW(robot->execute("bad_traj"), std::invalid_argument);

}


TEST_F(DeveloperRobotTest, planning)
{
  const std::string TRAJECTORY_NAME("traj1");
  const moveit_simple::InterpolationType cart = moveit_simple::interpolation_type::CARTESIAN;
  const moveit_simple::InterpolationType joint = moveit_simple::interpolation_type::JOINT;

  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  std::vector<double>joint1(6,0);
  std::vector<double>joint2(6,M_PI/6);
  std::vector<double>joint_interpolated_expected_joint(6,M_PI/12);
  std::vector<double>seed;
  std::vector<double>cart_interpolated_expected_joint;

  Eigen::Affine3d pose1;
  Eigen::Affine3d pose2;
  Eigen::Affine3d  joint_interpolated_expected_pose;

  EXPECT_TRUE(robot2->getPose(joint1, pose1));
  EXPECT_TRUE(robot2->getPose(joint2, pose2));
  EXPECT_TRUE(robot2->getPose(joint_interpolated_expected_joint, joint_interpolated_expected_pose));

  Eigen::Quaterniond point1_quaternion(pose1.rotation());
  Eigen::Quaterniond point2_quaternion(pose2.rotation());
  Eigen::Affine3d cart_interpolated_expected_pose(point1_quaternion.slerp(0.5, point2_quaternion));
  cart_interpolated_expected_pose.translation() =  0.5*(pose2.translation() + pose1.translation());

  EXPECT_TRUE(robot2->getJointSolution(cart_interpolated_expected_pose, 3.0, seed, cart_interpolated_expected_joint));

  // joint_point1,joint_point4, cart_point1 and cart_point3 
  // represent the same pose
  // joint_point2, cart_point2 and joint_point3 represent the same pose
  std::unique_ptr<moveit_simple::TrajectoryPoint> joint_point1 =
     std::unique_ptr<moveit_simple::TrajectoryPoint>
     (new moveit_simple::JointTrajectoryPoint(joint1, 1.0, "joint_point1"));

  std::unique_ptr<moveit_simple::TrajectoryPoint> joint_point2 =
     std::unique_ptr<moveit_simple::TrajectoryPoint>
     (new moveit_simple::JointTrajectoryPoint(joint2, 2.0, "joint_point2"));

  std::unique_ptr<moveit_simple::TrajectoryPoint> cart_point1 =
     std::unique_ptr<moveit_simple::TrajectoryPoint>
     (new moveit_simple::CartTrajectoryPoint(pose1, 3.0, "cart_point1"));

  std::unique_ptr<moveit_simple::TrajectoryPoint> cart_point2 =
     std::unique_ptr<moveit_simple::TrajectoryPoint>
     (new moveit_simple::CartTrajectoryPoint(pose2, 4.0, "cart_point2"));

  std::unique_ptr<moveit_simple::TrajectoryPoint> cart_point3 =
     std::unique_ptr<moveit_simple::TrajectoryPoint>
     (new moveit_simple::CartTrajectoryPoint(pose1, 5.0, "cart_point2"));

  std::unique_ptr<moveit_simple::TrajectoryPoint> joint_point3 =
     std::unique_ptr<moveit_simple::TrajectoryPoint>
     (new moveit_simple::JointTrajectoryPoint(joint2, 6.0, "joint_point3"));

  std::unique_ptr<moveit_simple::TrajectoryPoint> joint_point4 =
     std::unique_ptr<moveit_simple::TrajectoryPoint>
     (new moveit_simple::JointTrajectoryPoint(joint1, 7.0, "joint_point4"));

  // Add first point to start from a known point
  EXPECT_NO_THROW(robot2->addTrajPoint(TRAJECTORY_NAME, joint_point1));
  // joint interpolation between two joint points
  EXPECT_NO_THROW(robot2->addTrajPoint(TRAJECTORY_NAME,  joint_point2,  joint, 1));
  // joint interpolation between a cartesian point and a joint point
  EXPECT_NO_THROW(robot2->addTrajPoint(TRAJECTORY_NAME, cart_point1, joint, 1));
  // cartesian interpolation between two cartesian points
  EXPECT_NO_THROW(robot2->addTrajPoint(TRAJECTORY_NAME, cart_point2,  cart, 1));
  // joint interpolation between two cartesian points
  EXPECT_NO_THROW(robot2->addTrajPoint(TRAJECTORY_NAME, cart_point3,  joint, 1));
  // cartesian interpolation between a cartesian point and a joint point
  EXPECT_NO_THROW(robot2->addTrajPoint(TRAJECTORY_NAME,  joint_point3, cart, 1));
  // cartesian interpolation between two joint points
  EXPECT_NO_THROW(robot2->addTrajPoint(TRAJECTORY_NAME,  joint_point4, cart, 1));

  // Convert the input trajectory to vector of
  //  trajectory_msgs::JointTrajectoryPoint
  EXPECT_TRUE(robot2->toJointTrajectory(TRAJECTORY_NAME,points));
  EXPECT_EQ(points.size(),14);


  // EXPECT_NO_THROW(robot2.execute(TRAJECTORY_NAME));

  ROS_INFO_STREAM("Converting the joint positions to poses to compare " <<
                                                "against expected poses");
  std::vector<Eigen::Affine3d> pose_out;
  pose_out.resize(points.size());
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    EXPECT_TRUE(robot2->getPose(points[i].positions, pose_out[i]));
  }


  ROS_INFO_STREAM("Testing if the planned path is correct");

  EXPECT_TRUE(pose1.isApprox(pose_out[1],1e-3));
  ROS_INFO_STREAM(" pose1: " << std::endl << pose1.matrix());
  ROS_INFO_STREAM(" pose_out[1]: " << std::endl << pose_out[1].matrix());

  EXPECT_TRUE(joint_interpolated_expected_pose.isApprox(pose_out[2],1e-3));
  ROS_INFO_STREAM(" joint_interpolated_expected_pose: " << std::endl
                       << joint_interpolated_expected_pose.matrix());
  ROS_INFO_STREAM(" pose_out[2]: " << std::endl << pose_out[2].matrix());

  EXPECT_TRUE(pose2.isApprox(pose_out[3],1e-3));
  ROS_INFO_STREAM(" pose2: " << std::endl << pose2.matrix());
  ROS_INFO_STREAM(" pose_out[3]: " << std::endl << pose_out[3].matrix());

  EXPECT_TRUE(joint_interpolated_expected_pose.isApprox(pose_out[4],1e-3));
  ROS_INFO_STREAM(" pose_out[4]: " << std::endl << pose_out[4].matrix());
  ROS_INFO_STREAM(" joint_interpolated_expected_pose: " << std::endl
                       << joint_interpolated_expected_pose.matrix());

  EXPECT_TRUE(pose1.isApprox(pose_out[5],1e-3));
  ROS_INFO_STREAM(" pose1: " << std::endl << pose1.matrix());
  ROS_INFO_STREAM(" pose_out[5]: " << std::endl << pose_out[5].matrix());

  EXPECT_TRUE(cart_interpolated_expected_pose.isApprox(pose_out[6],1e-3));
  ROS_INFO_STREAM(" pose_out[6]: " << std::endl << pose_out[6].matrix());
  ROS_INFO_STREAM(" cart_interpolated_expected_pose: " << std::endl
                       << cart_interpolated_expected_pose.matrix());

  EXPECT_TRUE(pose2.isApprox(pose_out[7],1e-3));
  ROS_INFO_STREAM(" pose2: " << std::endl << pose2.matrix());
  ROS_INFO_STREAM(" pose_out[7]: " << std::endl << pose_out[7].matrix());

  EXPECT_TRUE(joint_interpolated_expected_pose.isApprox(pose_out[8],1e-3));
  ROS_INFO_STREAM(" pose_out[8]: " << std::endl << pose_out[8].matrix());
  ROS_INFO_STREAM(" joint_interpolated_expected_pose: " << std::endl
                       << joint_interpolated_expected_pose.matrix());

  EXPECT_TRUE(pose1.isApprox(pose_out[9],1e-3));
  ROS_INFO_STREAM(" pose1: " << std::endl << pose1.matrix());
  ROS_INFO_STREAM(" pose_out[9]: " << std::endl << pose_out[9].matrix());

  EXPECT_TRUE(cart_interpolated_expected_pose.isApprox(pose_out[10],1e-3));
  ROS_INFO_STREAM(" pose_out[10]: " << std::endl << pose_out[10].matrix());
  ROS_INFO_STREAM(" cart_interpolated_expected_pose: " << std::endl
                       << cart_interpolated_expected_pose.matrix());

  EXPECT_TRUE(pose2.isApprox(pose_out[11],1e-3));
  ROS_INFO_STREAM(" pose2: " << std::endl << pose2.matrix());
  ROS_INFO_STREAM(" pose_out[11]: " << std::endl << pose_out[11].matrix());

  EXPECT_TRUE(cart_interpolated_expected_pose.isApprox(pose_out[12],1e-3));
  ROS_INFO_STREAM(" pose_out[12]: " << std::endl << pose_out[12].matrix());
  ROS_INFO_STREAM(" cart_interpolated_expected_pose: " << std::endl
                       << cart_interpolated_expected_pose.matrix());

  EXPECT_TRUE(pose1.isApprox(pose_out[13],1e-3));
  ROS_INFO_STREAM(" pose2: " << std::endl << pose1.matrix());
  ROS_INFO_STREAM(" pose_out[13]: " << std::endl << pose_out[13].matrix());
}


TEST_F(DeveloperRobotTest, interpolation)
{
  const std::string TRAJECTORY_NAME("traj1");

  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  // Joint Interpolation Test
  std::vector<double>joint1(6,M_PI/6);
  std::vector<double>joint2(6,-1*M_PI/6);
  std::vector<double>joint_expected(6,0);

  const std::unique_ptr<moveit_simple::JointTrajectoryPoint> joint_point1 =
     std::unique_ptr<moveit_simple::JointTrajectoryPoint>
     (new moveit_simple::JointTrajectoryPoint(joint1, 1.0, "joint_point1"));

  const std::unique_ptr<moveit_simple::JointTrajectoryPoint> joint_point2 =
     std::unique_ptr<moveit_simple::JointTrajectoryPoint>
     (new moveit_simple::JointTrajectoryPoint(joint2, 2.0, "joint_point2"));

  std::unique_ptr<moveit_simple::JointTrajectoryPoint> joint_point_out;

  robot2->interpolate(joint_point1,joint_point2,0.5,joint_point_out);
  std::vector<double>joint_out = joint_point_out->jointPoint();

  ROS_INFO_STREAM(" joint_out: " << joint_out);
  ROS_INFO_STREAM(" joint_expected: " << joint_expected);
  double error_joint = fabs(joint_point_out->time() - 1.5);
  for (std::size_t i = 0; i < joint_expected.size(); ++i)
  {
    error_joint += fabs(joint_out[i] - joint_expected[i]);
  }
  EXPECT_NEAR(error_joint, 0.0, 1e-2);
  ROS_INFO_STREAM("joint_out" << joint_out);
  ROS_INFO_STREAM("joint_expected" << joint_out);

  // Cartesian Interpolation Test
  Eigen::Affine3d pose1 = Eigen::Affine3d::Identity();
  pose1.translation() = Eigen::Vector3d(1.9,0.0,2.2);
  Eigen::Quaterniond rot1;
  rot1.setFromTwoVectors(Eigen::Vector3d(0,-0.5,0), Eigen::Vector3d(1.9,0.0,2.2));
  pose1.linear() = rot1.toRotationMatrix();

  Eigen::Affine3d pose2 = Eigen::Affine3d::Identity();
  pose2.translation() = Eigen::Vector3d(1.9,0.0,2.7);

  Eigen::Affine3d pose3 = Eigen::Affine3d::Identity();
  pose3.translation() = Eigen::Vector3d(-0.592,-0.000,3.452);
  Eigen::Quaterniond rot3;
  rot3.setFromTwoVectors(Eigen::Vector3d(3.142,-0.964,3.142), Eigen::Vector3d(-0.592,-0.000,3.452));
  pose3.linear() = rot3.toRotationMatrix();

  Eigen::Quaterniond point1_quaternion(pose1.rotation());
  Eigen::Quaterniond point2_quaternion(pose2.rotation());
  Eigen::Affine3d pose_expected(point1_quaternion.slerp(0.5, point2_quaternion));
  pose_expected.translation() = Eigen::Vector3d(1.9,0.0,2.45);

  const std::unique_ptr<moveit_simple::CartTrajectoryPoint> cart_point1 =
     std::unique_ptr<moveit_simple::CartTrajectoryPoint>
     (new moveit_simple::CartTrajectoryPoint(pose1, 3.0, "cart_point1"));

  const std::unique_ptr<moveit_simple::CartTrajectoryPoint> cart_point2 =
     std::unique_ptr<moveit_simple::CartTrajectoryPoint>
     (new moveit_simple::CartTrajectoryPoint(pose2, 4.0, "cart_point2"));

  std::unique_ptr<moveit_simple::CartTrajectoryPoint> cart_point_out;

  robot2->interpolate(cart_point1,cart_point2,0.5,cart_point_out);

  EXPECT_TRUE(pose_expected.isApprox(cart_point_out->pose(),1e-3));

  // Test for mixed interpolation and point count in points
  const std::unique_ptr<moveit_simple::TrajectoryPoint> traj_point_joint1 =
     std::unique_ptr<moveit_simple::TrajectoryPoint>
     (new moveit_simple::JointTrajectoryPoint(joint1, 2.0, "traj_joint_point1"));

  const std::unique_ptr<moveit_simple::TrajectoryPoint> traj_point_joint2 =
     std::unique_ptr<moveit_simple::TrajectoryPoint>
     (new moveit_simple::JointTrajectoryPoint(joint1, 3.0, "traj_joint_point2"));

  const std::unique_ptr<moveit_simple::TrajectoryPoint> traj_point_cart1 =
     std::unique_ptr<moveit_simple::TrajectoryPoint>
     (new moveit_simple::CartTrajectoryPoint(pose1, 4.0, "traj_point_cart1"));

  const std::unique_ptr<moveit_simple::TrajectoryPoint> traj_point_cart2 =
     std::unique_ptr<moveit_simple::TrajectoryPoint>
     (new moveit_simple::CartTrajectoryPoint(pose3, 5.0, "traj_point_cart2"));

  EXPECT_NO_THROW(robot2->addTrajPoint(TRAJECTORY_NAME, "home",      0.5));
  // Populating points for further use and a fixed known point to start from
  EXPECT_TRUE(robot2->toJointTrajectory(TRAJECTORY_NAME,points));
  EXPECT_EQ(points.size(),2);
  // joint interpolation towards joint point
  EXPECT_TRUE(robot2->jointInterpolation(traj_point_joint1, points, (unsigned int) 10));
  EXPECT_EQ(points.size(),13);
  // Cartesian Interpolation towards cartesian point
  EXPECT_TRUE(robot2->cartesianInterpolation(traj_point_cart1, points, (unsigned int) 0));
  EXPECT_EQ(points.size(),14);
  // Cartesian Interpolation between pose1 and pose3 is not possible but joint interpolation is
  EXPECT_FALSE(robot2->cartesianInterpolation(traj_point_cart2, points, (unsigned int) 5));
  EXPECT_EQ(points.size(),14);
  // Joint Interpolation towards cartesian point
  EXPECT_TRUE(robot2->jointInterpolation(traj_point_cart2, points, (unsigned int) 5));
  EXPECT_EQ(points.size(),20);
  // Cartesian Interpolation towards joint point
  EXPECT_TRUE(robot2->jointInterpolation(traj_point_joint2, points, (unsigned int) 15));
  EXPECT_EQ(points.size(),36);
}



TEST_F(UserRobotTest, kinematics)
{
  const Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  Eigen::Affine3d pose1;
  Eigen::Affine3d pose2;
  Eigen::Affine3d pose3;
  std::vector<double> joint_point1(6,M_PI/6);
  std::vector<double> joint_point2;
  std::vector<double> joint_point3;
  std::vector<double> joint_point4;
  std::vector<double> seed = joint_point1;
  ros::Duration(2.0).sleep();  //wait for tf tree to populate

  EXPECT_TRUE(robot->getPose(joint_point1, pose1));
  EXPECT_TRUE(robot->getJointSolution(pose1, 3.0, seed, joint_point2));
  EXPECT_TRUE(robot->getPose(joint_point2, pose2));
  EXPECT_TRUE(robot->getJointSolution(pose2, 3.0, seed, joint_point3));
  EXPECT_TRUE(robot->getPose(joint_point3, pose3));
  EXPECT_FALSE(robot->getJointSolution(pose, 3.0, seed, joint_point3));

  // Check for error in getJointSolution
  ROS_INFO_STREAM(" joint_point1: " << joint_point1);
  ROS_INFO_STREAM(" joint_point2: " << joint_point2);
  ROS_INFO_STREAM(" joint_point3: " << joint_point3);

  double error_joint1 = 0.0;
  for (std::size_t i = 0; i < joint_point1.size(); ++i)
  {
    error_joint1 += fabs(joint_point1[i] - joint_point2[i]);
  }
  EXPECT_NEAR(error_joint1, 0.0, 1e-2);

  double error_joint2 = 0.0;
  for (std::size_t i = 0; i < joint_point1.size(); ++i)
  {
    error_joint2 += fabs(joint_point1[i] - joint_point3[i]);
  }
  EXPECT_NEAR(error_joint2, 0.0, 1e-2);

  // Check for error in getPose
  ROS_INFO_STREAM(" pose1: " << std::endl << pose1.matrix());
  ROS_INFO_STREAM(" pose2: " << std::endl << pose2.matrix());
  ROS_INFO_STREAM(" pose3: " << std::endl << pose3.matrix());

  EXPECT_TRUE(pose1.isApprox(pose2,1e-3));
  EXPECT_TRUE(pose1.isApprox(pose3,1e-3));

}


TEST_F(UserRobotTest, copy_current_pose)
{
  const std::string TRAJECTORY_NAME("traj1");

  // Adding trajectory points.
  EXPECT_NO_THROW(robot->addTrajPoint(TRAJECTORY_NAME, "home",      0.5));
  EXPECT_NO_THROW(robot->addTrajPoint(TRAJECTORY_NAME, "waypoint1", 1.0));
  EXPECT_NO_THROW(robot->addTrajPoint(TRAJECTORY_NAME, "tf_pub1",   2.0));
  EXPECT_NO_THROW(robot->addTrajPoint(TRAJECTORY_NAME, "waypoint2", 3.0));
  EXPECT_NO_THROW(robot->addTrajPoint(TRAJECTORY_NAME, "waypoint3", 4.0));

  std::vector<double> before_execution_1, before_execution_2,
                      before_execution_3, final_position;

  // Before the first execution robot should be at position "home"
  before_execution_1 = robot->getJointState();
  EXPECT_NO_THROW(robot->execute(TRAJECTORY_NAME));

  // Second Execution
  // Robot should be at "waypoint3" at all checkpoints from here on out.
  before_execution_2 = robot->getJointState();
  EXPECT_NO_THROW(robot->execute(TRAJECTORY_NAME));

  // Third Execution
  before_execution_3 = robot->getJointState();
  EXPECT_NO_THROW(robot->execute(TRAJECTORY_NAME));

  // Final Position
  final_position = robot->getJointState();

  ROS_INFO_STREAM("before_execution_1" << before_execution_1);
  ROS_INFO_STREAM("before_execution_2" << before_execution_2);
  ROS_INFO_STREAM("before_execution_3" << before_execution_3);
  ROS_INFO_STREAM("final_poistion" << final_position);

  // Testing
  EXPECT_FALSE(before_execution_1 == before_execution_2);
  EXPECT_TRUE(before_execution_2 == before_execution_3);
  EXPECT_TRUE(before_execution_2 == final_position);
}


TEST(MoveitSimpleTest, collision)
{
  // Calling the protected methods from Robot for test
  class RobotTest: public moveit_simple::Robot
  {
  public:
    using moveit_simple::Robot::Robot;
    using moveit_simple::Robot::addTrajPoint;
    using moveit_simple::Robot::toJointTrajectory;
    using moveit_simple::Robot::interpolate;
    using moveit_simple::Robot::jointInterpolation;
    using moveit_simple::Robot::cartesianInterpolation;
    using moveit_simple::Robot::isInCollision;
  };
  RobotTest robot2(ros::NodeHandle(), "robot_description", "manipulator");
  ros::Duration(2.0).sleep();  //wait for tf tree to populate

  const std::string TRAJECTORY_NAME("traj1");
  const moveit_simple::InterpolationType cart = moveit_simple::interpolation_type::CARTESIAN;
  const moveit_simple::InterpolationType joint = moveit_simple::interpolation_type::JOINT;

  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  std::vector<double>joint1(6,0);
  std::vector<double>joint2(6,0);
  std::vector<double>joint3(6,0);
  joint2[2] = M_PI;
  joint3[2] = M_PI/2;
  std::unique_ptr<moveit_simple::TrajectoryPoint> joint_point1 =
     std::unique_ptr<moveit_simple::TrajectoryPoint>
     (new moveit_simple::JointTrajectoryPoint(joint1, 1.0, "joint_point1"));

  std::unique_ptr<moveit_simple::TrajectoryPoint> joint_point2 =
     std::unique_ptr<moveit_simple::TrajectoryPoint>
     (new moveit_simple::JointTrajectoryPoint(joint2, 2.0, "joint_point2"));
  ROS_INFO_STREAM("joint1: " << joint1);
  ROS_INFO_STREAM("joint2: " << joint2);
  ROS_INFO_STREAM("joint3: " << joint3);
  // Add first point to start from a known point
  EXPECT_NO_THROW(robot2.addTrajPoint(TRAJECTORY_NAME, joint_point1));
  // joint interpolation between two joint points
  EXPECT_NO_THROW(robot2.addTrajPoint(TRAJECTORY_NAME,  joint_point2,  joint, 1));

  EXPECT_NO_THROW(robot2.execute(TRAJECTORY_NAME));
  EXPECT_FALSE(robot2.isInCollision(joint1));
  EXPECT_TRUE(robot2.isInCollision(joint3));
  EXPECT_THROW(robot2.execute(TRAJECTORY_NAME, true), moveit_simple::CollisionDetected);
}

}
