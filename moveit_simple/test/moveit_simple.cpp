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
#include <gtest/gtest.h>

using testing::Types;

namespace moveit_simple_test
{

TEST(MoveitSimpleTest, construction)
{
  moveit_simple::Robot robot(ros::NodeHandle(), "robot_description", "manipulator");
}

TEST(MoveitSimpleTest, reachability)
{
  moveit_simple::Robot robot(ros::NodeHandle(), "robot_description", "manipulator");
  const Eigen::Affine3d pose = Eigen::Affine3d::Identity(); 
  ros::Duration(2.0).sleep();  //wait for tf tree to populate

  ROS_INFO_STREAM("Testing reachability of unknown point, should fail");
  EXPECT_FALSE(robot.isReachable("unknown_name"));
  EXPECT_FALSE(robot.isReachable(pose, "random_link"));
  EXPECT_TRUE(robot.isReachable(pose, "tool0"));


  ROS_INFO_STREAM("Testing reach of points");
  ASSERT_TRUE(robot.isReachable("home"));        //stored in the SRDF
  ASSERT_TRUE(robot.isReachable("waypoint1"));   //stored in the URDF
  ASSERT_TRUE(robot.isReachable("waypoint2"));   //stored in the URDF
  ASSERT_TRUE(robot.isReachable("waypoint3"));   //stored in the URDF
  ASSERT_TRUE(robot.isReachable("tf_pub1"));     //stored published externally
  ASSERT_FALSE(robot.isReachable("waypoint4"));  //stored in the URDF
}


TEST(MoveitSimpleTest, add_trajectory)
{
  moveit_simple::Robot robot(ros::NodeHandle(), "robot_description", "manipulator");
  const std::string TRAJECTORY_NAME("traj1");
  const Eigen::Affine3d pose = Eigen::Affine3d::Identity(); 
  ros::Duration(2.0).sleep();  //wait for tf tree to populate
  const moveit_simple::InterpolationType cart = moveit_simple::CARTESIAN;
  const moveit_simple::InterpolationType joint = moveit_simple::JOINT;

  ROS_INFO_STREAM("Testing loading of unknown point, should fail");
  EXPECT_THROW(robot.addTrajPoint("bad_traj", "unknown_name", 1.0),std::invalid_argument);
  EXPECT_THROW(robot.addTrajPoint(TRAJECTORY_NAME, pose, "random_link", 5.0), tf2::TransformException);
  ROS_INFO_STREAM("Testing trajectory adding of points");
  EXPECT_NO_THROW(robot.addTrajPoint(TRAJECTORY_NAME, "home",      0.5));
  EXPECT_NO_THROW(robot.addTrajPoint(TRAJECTORY_NAME, "waypoint1", 1.0, joint, 5));
  EXPECT_NO_THROW(robot.addTrajPoint(TRAJECTORY_NAME, "tf_pub1",   2.0, cart, 8));
//  EXPECT_NO_THROW(robot.addTrajPoint(TRAJECTORY_NAME, "tf_pub1",   2.0));
  EXPECT_NO_THROW(robot.addTrajPoint(TRAJECTORY_NAME, "waypoint2", 3.0));
  EXPECT_NO_THROW(robot.addTrajPoint(TRAJECTORY_NAME, "waypoint3", 4.0, cart));
  EXPECT_NO_THROW(robot.addTrajPoint(TRAJECTORY_NAME, pose, "tool0", 5.0));
  EXPECT_NO_THROW(robot.execute(TRAJECTORY_NAME));

  EXPECT_NO_THROW(robot.addTrajPoint("traj2", "waypoint4", 4.5));
  EXPECT_THROW(robot.execute("traj2"), moveit_simple::IKFailException);

  EXPECT_THROW(robot.execute("bad_traj"), std::invalid_argument);

}


TEST(MoveitSimpleTest, interpolation)
{
  class Robot2: public moveit_simple::Robot
  {
  public:
    using moveit_simple::Robot::Robot;
    using moveit_simple::Robot::toJointTrajectory;
    using moveit_simple::Robot::interpolate;
    using moveit_simple::Robot::jointInterpolation;
    using moveit_simple::Robot::cartesianInterpolation;
  };
  Robot2 robot2(ros::NodeHandle(), "robot_description", "manipulator");
  ros::Duration(2.0).sleep();  //wait for tf tree to populate

  const std::string TRAJECTORY_NAME("traj1");
  const moveit_simple::InterpolationType cart = moveit_simple::CARTESIAN;
  const moveit_simple::InterpolationType joint = moveit_simple::JOINT;

  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  // Joint Interpolation Test
  std::vector<double>joint1(6,M_PI/6);
  std::vector<double>joint2(6,-1*M_PI/3);
  std::vector<double>joint_expected(6,-1*M_PI/12);

  const std::unique_ptr<moveit_simple::JointTrajectoryPoint> joint_point1 =
     std::unique_ptr<moveit_simple::JointTrajectoryPoint>
     (new moveit_simple::JointTrajectoryPoint(joint1, 1.0, "joint_point1"));

  const std::unique_ptr<moveit_simple::JointTrajectoryPoint> joint_point2 =
     std::unique_ptr<moveit_simple::JointTrajectoryPoint>
     (new moveit_simple::JointTrajectoryPoint(joint2, 2.0, "joint_point2"));

  std::unique_ptr<moveit_simple::JointTrajectoryPoint> joint_point_out;

  robot2.interpolate(joint_point1,joint_point2,0.5,joint_point_out);
  std::vector<double>joint_out = joint_point_out->jointPoint();

  double error_joint = fabs(joint_point_out->time() - 1.5);
  for (std::size_t i = 0; i < joint_expected.size(); ++i)
  {
    error_joint += fabs(joint_out[i] - joint_expected[i]);
  }
  EXPECT_NEAR(error_joint, 0.0, 1e-2);

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

  Eigen::Affine3d pose_expected = Eigen::Affine3d::Identity();
  pose_expected.translation() = Eigen::Vector3d(1.9,0.0,2.45);

  const std::unique_ptr<moveit_simple::CartTrajectoryPoint> cart_point1 =
     std::unique_ptr<moveit_simple::CartTrajectoryPoint>
     (new moveit_simple::CartTrajectoryPoint(pose1, 3.0, "cart_point1"));

  const std::unique_ptr<moveit_simple::CartTrajectoryPoint> cart_point2 =
     std::unique_ptr<moveit_simple::CartTrajectoryPoint>
     (new moveit_simple::CartTrajectoryPoint(pose2, 4.0, "cart_point2"));

  std::unique_ptr<moveit_simple::CartTrajectoryPoint> cart_point_out;

  robot2.interpolate(cart_point1,cart_point2,0.5,cart_point_out);

  double error_cart = fabs(cart_point_out->time() - 3.5)
               + (pose_expected.translation() - cart_point_out->pose().translation()).norm();
  EXPECT_NEAR(error_cart, 0.0, 1e-2);


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

  EXPECT_NO_THROW(robot2.addTrajPoint(TRAJECTORY_NAME, "home",      0.5));
  // Populating points for further use and a fixed known point to start from
  EXPECT_TRUE(robot2.toJointTrajectory(TRAJECTORY_NAME,points));
  EXPECT_EQ(points.size(),2);
  // joint interpolation towards joint point
  EXPECT_TRUE(robot2.jointInterpolation(traj_point_joint1, points, (unsigned int) 10));
  EXPECT_EQ(points.size(),13);
  // Cartesian Interpolation towards cartesian point
  EXPECT_TRUE(robot2.cartesianInterpolation(traj_point_cart1, points, (unsigned int) 0));
  EXPECT_EQ(points.size(),14);
  // Cartesian Interpolation between pose1 and pose3 is not possible but joint interpolation is
  EXPECT_FALSE(robot2.cartesianInterpolation(traj_point_cart2, points, (unsigned int) 5));
  EXPECT_EQ(points.size(),14);
  // Joint Interpolation towards cartesian point
  EXPECT_TRUE(robot2.jointInterpolation(traj_point_cart2, points, (unsigned int) 5));
  EXPECT_EQ(points.size(),20);
  // Cartesian Interpolation towards joint point
  EXPECT_TRUE(robot2.jointInterpolation(traj_point_joint2, points, (unsigned int) 15));
  EXPECT_EQ(points.size(),36);
}


TEST(MoveitSimpleTest, kinematics)
{
  moveit_simple::Robot robot(ros::NodeHandle(), "robot_description", "manipulator");
  const Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  Eigen::Affine3d pose1;
  Eigen::Affine3d pose2;
  std::vector<double> joint_point1(6,0);
  std::vector<double> joint_point2;
  std::vector<double> joint_point3;
  std::vector<double> seed(6,0.1);
  ros::Duration(2.0).sleep();  //wait for tf tree to populate

  EXPECT_TRUE(robot.getPose(joint_point1, pose1));
  EXPECT_TRUE(robot.getJointSolution(pose1, 3.0, seed, joint_point2));
  EXPECT_FALSE(robot.getJointSolution(pose, 4.0, seed, joint_point3));
  EXPECT_TRUE(robot.getPose(joint_point2, pose2));

  double error_pose = (pose1.translation() - pose2.translation()).norm() +
                      (pose1.linear() - pose2.linear()).norm();
  EXPECT_NEAR(error_pose, 0.0, 1e-2);


  double error_joint = 0.0;
  for (std::size_t i = 0; i < joint_point1.size(); ++i)
  {
    error_joint += fabs(joint_point1[i] - joint_point2[i]);
  }
  EXPECT_NEAR(error_joint, 0.0, 1e-2);

}

}
