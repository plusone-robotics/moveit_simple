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

#include <eigen_conversions/eigen_msg.h>
#include <gtest/gtest.h>
#include <moveit_simple/moveit_simple.h>

using testing::Types;

namespace moveit_simple_test
{
/**
 * @brief UserRobotTest is a fixture for testing public methods.
 * Objects that can be directly used inside the test
 - robot pointer for moveit_simple::OnlineRobot
*/
class UserRobotTest : public ::testing::Test
{
protected:
  std::unique_ptr<moveit_simple::OnlineRobot> user_robot;
  
  virtual void SetUp()
  {      
    user_robot = std::unique_ptr<moveit_simple::OnlineRobot> (new moveit_simple::OnlineRobot
      (ros::NodeHandle(), "robot_description", "manipulator", "base_link", "link_t"));
    
    ros::Duration(2.0).sleep();  //wait for tf tree to populate
  }

  virtual void TearDown() { }
};

/**
 * @brief DeveloperRobot is a class inheriting from moveit_simple::OnlineRobot to test protected methods
 * Add the protected methods(that are required to be tested) in the following list
*/
class DeveloperRobot: public moveit_simple::OnlineRobot
{
public:
  using moveit_simple::OnlineRobot::OnlineRobot;
  using moveit_simple::OnlineRobot::addTrajPoint;
  using moveit_simple::OnlineRobot::toJointTrajectory;
  using moveit_simple::OnlineRobot::interpolate;
  using moveit_simple::OnlineRobot::jointInterpolation;
  using moveit_simple::OnlineRobot::cartesianInterpolation;
  using moveit_simple::OnlineRobot::isInCollision;
  using moveit_simple::OnlineRobot::getFK;
  using moveit_simple::OnlineRobot::getIK;  
};
  
/**
 * @brief DeveloperRobotTest is a fixture for testing protected methods.
 * Objects that can be directly used inside the test
 - robot pointer for DeveloperRobot
*/
class DeveloperRobotTest : public ::testing::Test
{
protected:
  std::unique_ptr<DeveloperRobot> developer_robot;
  virtual void SetUp()
  {      
    developer_robot = std::unique_ptr<DeveloperRobot> (new DeveloperRobot
      (ros::NodeHandle(), "robot_description", "manipulator", "base_link", "link_t"));

    ros::Duration(2.0).sleep();  // Wait for tf tree to populate
  }

  virtual void TearDown() { }
};

struct KinematicsTestData
{
  std::vector<double> joints; // [joint_s, joint_l, joint_u, joint_r, joint_b, joint_t]

  // Forward Kinematics from base -> tool0
  std::vector<double> translation_tool0; // [x, y, z]
  std::vector<double> rotation_tool0; // Quaternion [x, y, z, w]

  // Forward kinematics from base -> tool_custom
  std::vector<double> translation_tool_custom; // [x, y, z]
  std::vector<double> rotation_tool_custom; // Quaternion [x, y, z, w]  
};

TEST_F(DeveloperRobotTest, kinematics)
{
  KinematicsTestData pose_home;
  pose_home.joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  pose_home.translation_tool0 = {0.479, 0.000, 0.680};
  pose_home.rotation_tool0 = {0.000, 0.707, 0.000, 0.707};
  pose_home.translation_tool_custom = {0.629, 0.000, 0.680};
  pose_home.rotation_tool_custom = {0.000, 0.707, 0.000, 0.707};

  KinematicsTestData pose_1;
  pose_1.joints = {0.2291, 0.31208875, 0.4684298, -3.07667758, 1.26189528, 3.560005};
  pose_1.translation_tool0 = {0.505, 0.112, 0.635};
  pose_1.rotation_tool0 = {-0.312, 0.921, -0.054, 0.227};
  pose_1.translation_tool_custom = {0.572, 0.119, 0.501};
  pose_1.rotation_tool_custom = {-0.312, 0.921, -0.054, 0.227};

  KinematicsTestData pose_2;
  pose_2.joints = {0.14301422, 0.60628475, 0.4671596, 0.599551, -1.26189528, 1.44636962};
  pose_2.translation_tool0 = {0.589, 0.038, 0.511};
  pose_2.rotation_tool0 = {0.725, -0.620, 0.278, 0.110};
  pose_2.translation_tool_custom = {0.629, -0.038, 0.388};
  pose_2.rotation_tool_custom = {0.725, -0.620, 0.278, 0.110};

  KinematicsTestData pose_3;
  pose_3.joints = {-0.32163364, 0.9676505, 1.079396, -0.7593869, -0.44681216, 1.5142271};
  pose_3.translation_tool0 = {0.694, -0.204, 0.561};
  pose_3.rotation_tool0 = {-0.277, 0.727, -0.230, 0.585};
  pose_3.translation_tool_custom = {0.840, -0.206, 0.530};
  pose_3.rotation_tool_custom = {-0.277, 0.727, -0.230, 0.585};

  KinematicsTestData pose_4;
  pose_4.joints = {-0.32163364, 0.334979, -0.0938454, -0.0397932, -1.0515794, 1.5142271};
  pose_4.translation_tool0 = {0.467, -0.153, 0.446};
  pose_4.rotation_tool0 = {-0.553, 0.831, -0.025, 0.042};
  pose_4.translation_tool_custom = {0.482, -0.152, 0.297};
  pose_4.rotation_tool_custom = {-0.553, 0.831, -0.025, 0.042};

  KinematicsTestData pose_5;
  pose_5.joints = {-1.00109954, 0.9001055, 0.4671596, 1.07905894, -1.28807568, -2.42276336};
  pose_5.translation_tool0 = {0.278, -0.570, 0.385};
  pose_5.rotation_tool0 = {0.872, 0.058, -0.407, 0.265};
  pose_5.translation_tool_custom = {0.176, -0.646, 0.306};
  pose_5.rotation_tool_custom = {0.872, 0.058, -0.407, 0.265};

  auto testKinematics = [&](const KinematicsTestData &pose)
  {
    const double ABS_ERROR = 0.01; // The numbers in the data are rounded
    const std::vector<double> JOINT_SEED(pose.joints.size(), 0.0);

    // Testing forward kinematics to tool_custom
    Eigen::Affine3d tool_custom_calculated_pose;
    ASSERT_TRUE(developer_robot->getFK(pose.joints, tool_custom_calculated_pose));

    Eigen::Vector3d tool_custom_translation = tool_custom_calculated_pose.translation();
    EXPECT_NEAR(tool_custom_translation.x(), pose.translation_tool_custom[0], ABS_ERROR);
    EXPECT_NEAR(tool_custom_translation.y(), pose.translation_tool_custom[1], ABS_ERROR);
    EXPECT_NEAR(tool_custom_translation.z(), pose.translation_tool_custom[2], ABS_ERROR);
    
    Eigen::Quaterniond tool_custom_rotation = Eigen::Quaterniond(tool_custom_calculated_pose.linear());
    EXPECT_NEAR(tool_custom_rotation.x(), pose.rotation_tool_custom[0], ABS_ERROR);
    EXPECT_NEAR(tool_custom_rotation.y(), pose.rotation_tool_custom[1], ABS_ERROR);
    EXPECT_NEAR(tool_custom_rotation.z(), pose.rotation_tool_custom[2], ABS_ERROR);
    EXPECT_NEAR(tool_custom_rotation.w(), pose.rotation_tool_custom[3], ABS_ERROR);

    // Testing inverse kinematics
    std::vector<double> joint_solution;
    ASSERT_TRUE(developer_robot->getIK(tool_custom_calculated_pose, JOINT_SEED, joint_solution, 10, 15));

    // Checking IK solution with forward kinematics
    Eigen::Affine3d check_joint_solution;
    ASSERT_TRUE(developer_robot->getFK(joint_solution, check_joint_solution));

    Eigen::Vector3d check_joint_translation = check_joint_solution.translation();
    EXPECT_NEAR(check_joint_translation.x(), pose.translation_tool_custom[0], ABS_ERROR);
    EXPECT_NEAR(check_joint_translation.y(), pose.translation_tool_custom[1], ABS_ERROR);
    EXPECT_NEAR(check_joint_translation.z(), pose.translation_tool_custom[2], ABS_ERROR);
    
    Eigen::Quaterniond check_joint_rotation = Eigen::Quaterniond(check_joint_solution.linear());
    EXPECT_NEAR(check_joint_rotation.x(), pose.rotation_tool_custom[0], ABS_ERROR);
    EXPECT_NEAR(check_joint_rotation.y(), pose.rotation_tool_custom[1], ABS_ERROR);
    EXPECT_NEAR(check_joint_rotation.z(), pose.rotation_tool_custom[2], ABS_ERROR);
    EXPECT_NEAR(check_joint_rotation.w(), pose.rotation_tool_custom[3], ABS_ERROR);    
  };

  testKinematics(pose_home);
  testKinematics(pose_1);
  testKinematics(pose_2);
  testKinematics(pose_3);
  testKinematics(pose_4);
  testKinematics(pose_5);
}

TEST(MoveitSimpleTest, construction_robot)
{
  moveit_simple::Robot robot(ros::NodeHandle(), "robot_description", "manipulator");  
}

TEST(MoveitSimpleTest, construction_online_robot)
{
  moveit_simple::OnlineRobot online_robot(ros::NodeHandle(), "robot_description", "manipulator");
}

TEST(MoveitSimpleTest, construction_robot_ikfast)
{
  moveit_simple::Robot robot(ros::NodeHandle(), "robot_description", "manipulator", 
    "base_link", "link_t"); 
}

TEST(MoveitSimpleTest, construction_online_robot_ikfast)
{
  moveit_simple::OnlineRobot online_robot(ros::NodeHandle(), "robot_description", "manipulator", 
    "base_link", "link_t");
}

TEST_F(UserRobotTest, reachability)
{
  const Eigen::Affine3d pose = Eigen::Affine3d::Identity(); 

  ROS_INFO_STREAM("Testing reachability of unknown point, should fail");
  EXPECT_FALSE(user_robot->isReachable("unknown_name"));
  EXPECT_FALSE(user_robot->isReachable(pose, "random_link"));
  EXPECT_TRUE(user_robot->isReachable(pose, "link_t"));

  ROS_INFO_STREAM("Testing reach of points");
  EXPECT_TRUE(user_robot->isReachable("home")); // Defined in SRDF
  EXPECT_TRUE(user_robot->isReachable("wp1")); // Defined in URDF
  EXPECT_TRUE(user_robot->isReachable("wp2")); // Defined in URDF
  EXPECT_TRUE(user_robot->isReachable("wp3")); // Defined in URDF
  EXPECT_TRUE(user_robot->isReachable("tf_pub1")); // Published externally (defined in a launch file)
  EXPECT_FALSE(user_robot->isReachable("wp4"));  // Defined in URDF
}

TEST_F(UserRobotTest, add_trajectory)
{
  const std::string TRAJECTORY_NAME("traj1");
  const Eigen::Affine3d pose = Eigen::Affine3d::Identity(); 
  const moveit_simple::InterpolationType cart = moveit_simple::interpolation_type::CARTESIAN;
  const moveit_simple::InterpolationType joint = moveit_simple::interpolation_type::JOINT;

  ROS_INFO_STREAM("Testing loading of unknown point, should fail");
  EXPECT_THROW(user_robot->addTrajPoint("bad_traj", "unknown_name", 1.0), std::invalid_argument);
  EXPECT_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, pose, "random_link", 5.0), tf2::TransformException);
  
  ROS_INFO_STREAM("Testing trajectory adding of points");
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "home", 0.5));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "wp1", 1.0, joint, 5));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "tf_pub1", 2.0, cart, 8));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "wp2", 3.0));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "wp3", 4.0, joint));
  EXPECT_NO_THROW(user_robot->execute(TRAJECTORY_NAME));

  EXPECT_NO_THROW(user_robot->addTrajPoint("traj2", "wp4", 4.5));
  EXPECT_THROW(user_robot->execute("traj2"), moveit_simple::IKFailException);

  EXPECT_THROW(user_robot->execute("bad_traj"), std::invalid_argument);
}

TEST_F(DeveloperRobotTest, planning)
{
  const double ALLOWED_ERROR = 1e-2;
  const std::string TRAJECTORY_NAME("traj1");
  const moveit_simple::InterpolationType cart = moveit_simple::interpolation_type::CARTESIAN;
  const moveit_simple::InterpolationType joint = moveit_simple::interpolation_type::JOINT;

  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  std::vector<double> joint1(6, 0);
  std::vector<double> joint2(6, M_PI/6);
  std::vector<double> joint_interpolated_expected_joint(6, M_PI/12);
  std::vector<double> seed;
  std::vector<double> cart_interpolated_expected_joint;

  Eigen::Affine3d pose1;
  Eigen::Affine3d pose2;
  Eigen::Affine3d joint_interpolated_expected_pose;

  EXPECT_TRUE(developer_robot->getPose(joint1, pose1));
  EXPECT_TRUE(developer_robot->getPose(joint2, pose2));
  EXPECT_TRUE(developer_robot->getPose(joint_interpolated_expected_joint, joint_interpolated_expected_pose));

  Eigen::Quaterniond point1_quaternion(pose1.rotation());
  Eigen::Quaterniond point2_quaternion(pose2.rotation());
  Eigen::Affine3d cart_interpolated_expected_pose(point1_quaternion.slerp(0.5, point2_quaternion));
  cart_interpolated_expected_pose.translation() =  0.5*(pose2.translation() + pose1.translation());

  EXPECT_TRUE(developer_robot->getJointSolution(cart_interpolated_expected_pose, 3.0, seed, cart_interpolated_expected_joint));

  // joint_point1, joint_point4, cart_point1, and cart_point3 represent the same pose
  // joint_point2, cart_point2, and joint_point3 represent the same pose
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
  EXPECT_NO_THROW(developer_robot->addTrajPoint(TRAJECTORY_NAME, joint_point1));
  // joint interpolation between two joint points
  EXPECT_NO_THROW(developer_robot->addTrajPoint(TRAJECTORY_NAME, joint_point2, joint, 1));
  // joint interpolation between a cartesian point and a joint point
  EXPECT_NO_THROW(developer_robot->addTrajPoint(TRAJECTORY_NAME, cart_point1, joint, 1));
  // cartesian interpolation between two cartesian points
  EXPECT_NO_THROW(developer_robot->addTrajPoint(TRAJECTORY_NAME, cart_point2, cart, 1));
  // joint interpolation between two cartesian points
  EXPECT_NO_THROW(developer_robot->addTrajPoint(TRAJECTORY_NAME, cart_point3, joint, 1));
  // cartesian interpolation between a cartesian point and a joint point
  EXPECT_NO_THROW(developer_robot->addTrajPoint(TRAJECTORY_NAME, joint_point3, cart, 1));
  // cartesian interpolation between two joint points
  EXPECT_NO_THROW(developer_robot->addTrajPoint(TRAJECTORY_NAME, joint_point4, cart, 1));

  // Convert the input trajectory to vector of trajectory_msgs::JointTrajectoryPoint
  EXPECT_TRUE(developer_robot->toJointTrajectory(TRAJECTORY_NAME,points));
  EXPECT_EQ(points.size(),14);

  ROS_INFO_STREAM("Converting the joint positions to poses to compare against expected poses");
  
  std::vector<Eigen::Affine3d> pose_out;
  pose_out.resize(points.size());
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    EXPECT_TRUE(developer_robot->getPose(points[i].positions, pose_out[i]));
  }

  ROS_INFO_STREAM("Testing if the planned path is correct");

  EXPECT_TRUE(pose1.isApprox(pose_out[1], ALLOWED_ERROR));
  ROS_INFO_STREAM(" pose1: " << std::endl << pose1.matrix());
  ROS_INFO_STREAM(" pose_out[1]: " << std::endl << pose_out[1].matrix());

  EXPECT_TRUE(joint_interpolated_expected_pose.isApprox(pose_out[2], ALLOWED_ERROR));
  ROS_INFO_STREAM(" joint_interpolated_expected_pose: " << std::endl
                       << joint_interpolated_expected_pose.matrix());
  ROS_INFO_STREAM(" pose_out[2]: " << std::endl << pose_out[2].matrix());

  EXPECT_TRUE(pose2.isApprox(pose_out[3], ALLOWED_ERROR));
  ROS_INFO_STREAM(" pose2: " << std::endl << pose2.matrix());
  ROS_INFO_STREAM(" pose_out[3]: " << std::endl << pose_out[3].matrix());

  EXPECT_TRUE(joint_interpolated_expected_pose.isApprox(pose_out[4], ALLOWED_ERROR));
  ROS_INFO_STREAM(" pose_out[4]: " << std::endl << pose_out[4].matrix());
  ROS_INFO_STREAM(" joint_interpolated_expected_pose: " << std::endl
                       << joint_interpolated_expected_pose.matrix());

  EXPECT_TRUE(pose1.isApprox(pose_out[5], ALLOWED_ERROR));
  ROS_INFO_STREAM(" pose1: " << std::endl << pose1.matrix());
  ROS_INFO_STREAM(" pose_out[5]: " << std::endl << pose_out[5].matrix());

  EXPECT_TRUE(cart_interpolated_expected_pose.isApprox(pose_out[6], ALLOWED_ERROR));
  ROS_INFO_STREAM(" pose_out[6]: " << std::endl << pose_out[6].matrix());
  ROS_INFO_STREAM(" cart_interpolated_expected_pose: " << std::endl
                       << cart_interpolated_expected_pose.matrix());

  EXPECT_TRUE(pose2.isApprox(pose_out[7], ALLOWED_ERROR));
  ROS_INFO_STREAM(" pose2: " << std::endl << pose2.matrix());
  ROS_INFO_STREAM(" pose_out[7]: " << std::endl << pose_out[7].matrix());

  EXPECT_TRUE(joint_interpolated_expected_pose.isApprox(pose_out[8], ALLOWED_ERROR));
  ROS_INFO_STREAM(" pose_out[8]: " << std::endl << pose_out[8].matrix());
  ROS_INFO_STREAM(" joint_interpolated_expected_pose: " << std::endl
                       << joint_interpolated_expected_pose.matrix());

  EXPECT_TRUE(pose1.isApprox(pose_out[9], ALLOWED_ERROR));
  ROS_INFO_STREAM(" pose1: " << std::endl << pose1.matrix());
  ROS_INFO_STREAM(" pose_out[9]: " << std::endl << pose_out[9].matrix());

  EXPECT_TRUE(cart_interpolated_expected_pose.isApprox(pose_out[10], ALLOWED_ERROR));
  ROS_INFO_STREAM(" pose_out[10]: " << std::endl << pose_out[10].matrix());
  ROS_INFO_STREAM(" cart_interpolated_expected_pose: " << std::endl
                       << cart_interpolated_expected_pose.matrix());

  EXPECT_TRUE(pose2.isApprox(pose_out[11], ALLOWED_ERROR));
  ROS_INFO_STREAM(" pose2: " << std::endl << pose2.matrix());
  ROS_INFO_STREAM(" pose_out[11]: " << std::endl << pose_out[11].matrix());

  EXPECT_TRUE(cart_interpolated_expected_pose.isApprox(pose_out[12], ALLOWED_ERROR));
  ROS_INFO_STREAM(" pose_out[12]: " << std::endl << pose_out[12].matrix());
  ROS_INFO_STREAM(" cart_interpolated_expected_pose: " << std::endl
                       << cart_interpolated_expected_pose.matrix());

  EXPECT_TRUE(pose1.isApprox(pose_out[13], ALLOWED_ERROR));
  ROS_INFO_STREAM(" pose2: " << std::endl << pose1.matrix());
  ROS_INFO_STREAM(" pose_out[13]: " << std::endl << pose_out[13].matrix());
}

TEST_F(DeveloperRobotTest, interpolation)
{
  const std::string TRAJECTORY_NAME("traj1");

  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  // Joint Interpolation Test
  std::vector<double>joint1(6, M_PI/6);
  std::vector<double>joint2(6, -1*M_PI/6);
  std::vector<double>joint_expected(6, 0);

  const std::unique_ptr<moveit_simple::JointTrajectoryPoint> joint_point1 =
     std::unique_ptr<moveit_simple::JointTrajectoryPoint>
     (new moveit_simple::JointTrajectoryPoint(joint1, 1.0, "joint_point1"));

  const std::unique_ptr<moveit_simple::JointTrajectoryPoint> joint_point2 =
     std::unique_ptr<moveit_simple::JointTrajectoryPoint>
     (new moveit_simple::JointTrajectoryPoint(joint2, 2.0, "joint_point2"));

  std::unique_ptr<moveit_simple::JointTrajectoryPoint> joint_point_out;

  developer_robot->interpolate(joint_point1,joint_point2,0.5,joint_point_out);
  std::vector<double>joint_out = joint_point_out->jointPoint();

  ROS_INFO_STREAM("joint_out: " << joint_out);
  ROS_INFO_STREAM("joint_expected: " << joint_expected);
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
  pose1.translation() = Eigen::Vector3d(0.8, 0.1, 0.3);
  Eigen::Quaterniond rot1(0.66, 0.0, 0.7513, 0.0);
  pose1.linear() = rot1.toRotationMatrix();

  Eigen::Affine3d pose2 = Eigen::Affine3d::Identity();
  pose2.translation() = Eigen::Vector3d(0.8, 0.1, 0.5);
  Eigen::Quaterniond rot2(0.4085, 0, 0.9127, 0);
  pose2.linear() = rot2.toRotationMatrix();

  Eigen::Affine3d pose3 = Eigen::Affine3d::Identity();
  pose3.translation() = Eigen::Vector3d(-0.3, -0.5, 0.2);
  Eigen::Quaterniond rot3(0.8253, 0.0, 0.5646, 0.0);
  pose3.linear() = rot3.toRotationMatrix();

  Eigen::Quaterniond point1_quaternion(pose1.rotation());
  Eigen::Quaterniond point2_quaternion(pose2.rotation());
  Eigen::Affine3d pose_expected(point1_quaternion.slerp(0.5, point2_quaternion));
  pose_expected.translation() = Eigen::Vector3d(0.8, 0.1, 0.4);

  const std::unique_ptr<moveit_simple::CartTrajectoryPoint> cart_point1 =
     std::unique_ptr<moveit_simple::CartTrajectoryPoint>
     (new moveit_simple::CartTrajectoryPoint(pose1, 3.0, "cart_point1"));

  const std::unique_ptr<moveit_simple::CartTrajectoryPoint> cart_point2 =
     std::unique_ptr<moveit_simple::CartTrajectoryPoint>
     (new moveit_simple::CartTrajectoryPoint(pose2, 4.0, "cart_point2"));

  std::unique_ptr<moveit_simple::CartTrajectoryPoint> cart_point_out;

  developer_robot->interpolate(cart_point1,cart_point2,0.5,cart_point_out);

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

  EXPECT_NO_THROW(developer_robot->addTrajPoint(TRAJECTORY_NAME, "home",      0.5));
  // Populating points for further use and a fixed known point to start from
  EXPECT_TRUE(developer_robot->toJointTrajectory(TRAJECTORY_NAME,points));
  EXPECT_EQ(points.size(),2);
  // joint interpolation towards joint point
  EXPECT_TRUE(developer_robot->jointInterpolation(traj_point_joint1, points, (unsigned int) 10));
  EXPECT_EQ(points.size(),13);
  // Cartesian Interpolation towards cartesian point
  EXPECT_TRUE(developer_robot->cartesianInterpolation(traj_point_cart1, points, (unsigned int) 0));
  EXPECT_EQ(points.size(),14);

  // Cartesian Interpolation between pose1 and pose3 is not possible but joint interpolation is
  EXPECT_FALSE(developer_robot->cartesianInterpolation(traj_point_cart2, points, (unsigned int) 5));
  EXPECT_EQ(points.size(),14);
  // Joint Interpolation towards cartesian point
  // This is false because we don't allow for large config changes in joint interpolations even if it is possible
  EXPECT_FALSE(developer_robot->jointInterpolation(traj_point_cart2, points, (unsigned int) 5));
  EXPECT_EQ(points.size(),14);

  // Cartesian Interpolation towards joint point
  EXPECT_TRUE(developer_robot->cartesianInterpolation(traj_point_joint2, points, (unsigned int) 15));
  EXPECT_EQ(points.size(),30);
}

TEST_F(UserRobotTest, speed_reconfiguration)
{
  const std::string TRAJECTORY_NAME("traj1");

  ros::Duration(2.0).sleep(); //wait for tf tree to populate

  double execution_time_tolerance = 0.25; // Empirically assumed

  double delta_half_min_speeds = 0.0;
  double delta_max_half_speeds = 0.0;
  double delta_time_for_speed_limits = 0.0;

  double execution_time_check_1 = INT_MAX;
  double execution_time_check_2 = INT_MAX;
  double execution_time_check_3 = INT_MAX;

  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "home",      0.5));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "wp1", 1.0));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "tf_pub1",   2.0));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "wp2", 3.0));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "wp3", 4.0));
  
  // Test 1 -- Max_Execution_Speed: Plan & then Execute that Plan separately
  user_robot->setSpeedModifier(1.0);
  EXPECT_TRUE(user_robot->getSpeedModifier() == 1.0);

  std::vector<moveit_simple::JointTrajectoryPoint> goal;

  EXPECT_NO_THROW(goal = user_robot->plan(TRAJECTORY_NAME));
  EXPECT_NO_THROW(user_robot->execute(goal));  
  
  execution_time_check_1 = goal[goal.size()-1].time();
  EXPECT_TRUE(execution_time_check_1 >= 0.0);
  ROS_INFO_STREAM("Time for single traj. execution at MAX speed: " 
    << execution_time_check_1 << " seconds");

  // Test 2 -- Half_Execution_Speed: Plan & Execute
  user_robot->setSpeedModifier(0.50);
  EXPECT_TRUE(user_robot->getSpeedModifier() == 0.50);

  double start_half_speed_execution = ros::Time::now().toSec();
  EXPECT_NO_THROW(user_robot->execute(TRAJECTORY_NAME));
  double end_half_speed_execution = ros::Time::now().toSec();

  execution_time_check_2 = end_half_speed_execution - start_half_speed_execution;
  EXPECT_TRUE(execution_time_check_2 >= 0.0);
  ROS_INFO_STREAM("Time for single traj. execution at Half speed: "
    << execution_time_check_2 << " seconds");

  // Test 3 -- Min_Execution_Speed: Plan & Execute
  user_robot->setSpeedModifier(0.25);
  EXPECT_TRUE(user_robot->getSpeedModifier() == 0.25);

  double start_min_speed_execution = ros::Time::now().toSec();
  EXPECT_NO_THROW(user_robot->execute(TRAJECTORY_NAME));
  double end_min_speed_execution = ros::Time::now().toSec();

  execution_time_check_3 = end_min_speed_execution - start_min_speed_execution;
  EXPECT_TRUE(execution_time_check_3 >= 0.0);
  ROS_INFO_STREAM("Time for single traj. execution at MIN speed: "
    << execution_time_check_3 << " seconds");

  delta_half_min_speeds = execution_time_check_2 / execution_time_check_1;
  EXPECT_TRUE(delta_half_min_speeds >= 0.0);

  delta_max_half_speeds = execution_time_check_3 / execution_time_check_2;
  EXPECT_TRUE(delta_max_half_speeds >= 0.0);

  delta_time_for_speed_limits = delta_max_half_speeds - delta_half_min_speeds;
  EXPECT_NEAR(delta_time_for_speed_limits, 0.0, execution_time_tolerance);

  if(abs(delta_time_for_speed_limits) > execution_time_tolerance) 
  {
    ROS_ERROR_STREAM("Time diff between [MAX_SPEED/REGULAR_SPEED] --> [" << 
                     execution_time_check_1 << ", " << execution_time_check_2 << 
                     "] & [REGULAR_SPEED/MIN_SPEED] --> [" << execution_time_check_2 <<
                     ", " << execution_time_check_3 << "] is " << delta_time_for_speed_limits << 
                     "; but tolerance limit is [" << execution_time_tolerance << "]");
  }
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

  EXPECT_TRUE(user_robot->getPose(joint_point1, pose1));
  EXPECT_TRUE(user_robot->getJointSolution(pose1, 3.0, seed, joint_point2));
  EXPECT_TRUE(user_robot->getPose(joint_point2, pose2));
  EXPECT_TRUE(user_robot->getJointSolution(pose2, 3.0, seed, joint_point3));
  EXPECT_TRUE(user_robot->getPose(joint_point3, pose3));
  EXPECT_FALSE(user_robot->getJointSolution(pose, 3.0, seed, joint_point3));

  // Check for error in getJointSolution
  ROS_INFO_STREAM("joint_point1: " << joint_point1);
  ROS_INFO_STREAM("joint_point2: " << joint_point2);
  ROS_INFO_STREAM("joint_point3: " << joint_point3);

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


TEST_F(UserRobotTest, custom_tool_link)
{
  const moveit_simple::InterpolationType cart = moveit_simple::interpolation_type::CARTESIAN;
  const moveit_simple::InterpolationType joint = moveit_simple::interpolation_type::JOINT;

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

  std::string tool_name = "tool_custom";
  EXPECT_TRUE(user_robot->getPose(joint_point1, tool_name, pose1));
  EXPECT_TRUE(user_robot->getJointSolution(pose1, tool_name, 3.0, seed, joint_point2));
  EXPECT_TRUE(user_robot->getPose(joint_point2, tool_name, pose2));
  EXPECT_TRUE(user_robot->getJointSolution(pose2, tool_name, 3.0, seed, joint_point3));
  EXPECT_TRUE(user_robot->getPose(joint_point3, tool_name, pose3));
  EXPECT_FALSE(user_robot->getJointSolution(pose, tool_name, 3.0, seed, joint_point3));

  // Check for error in getJointSolution
  ROS_INFO_STREAM("joint_point1: " << joint_point1);
  ROS_INFO_STREAM("joint_point2: " << joint_point2);
  ROS_INFO_STREAM("joint_point3: " << joint_point3);

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

  // Testing addTrajPoint() with custom_tool_frame as eef
  Eigen::Affine3d pose_msg_to_eigen;
  geometry_msgs::Pose pose_buffer;

  // We take the Origin of our reference frame in consideration as the "known pose"
  // In this case our reference frame resolves to the variable "point_name"
  pose_buffer.position.x = 0.000;
  pose_buffer.position.y = 0.000;
  pose_buffer.position.z = 0.000;

  pose_buffer.orientation.w = 1.000;
  pose_buffer.orientation.x = 0.000;
  pose_buffer.orientation.y = 0.000;
  pose_buffer.orientation.z = 0.000;

  tf::poseMsgToEigen(pose_buffer, pose_msg_to_eigen);

  const Eigen::Affine3d pose_eigen = pose_msg_to_eigen;

  const std::string TRAJECTORY_NAME("traj1");

  ROS_INFO_STREAM("Testing trajectory adding of points");
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "home", 0.5));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, pose_eigen, "wp1", tool_name, 1.0));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "tf_pub1", tool_name, 2.0, cart, 8));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "wp2", "tool0", 3.0));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "wp3", "tool_custom", 4.0));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "wp1", tool_name, 5.0, joint, 5));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, pose_eigen, "tool0", 6.0));
  
  EXPECT_NO_THROW(user_robot->execute(TRAJECTORY_NAME));
}


TEST_F(UserRobotTest, copy_current_pose)
{
  const std::string TRAJECTORY_NAME("traj1");

  // Adding trajectory points.
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "home", 0.5));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "wp1", 1.0));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "tf_pub1", 2.0));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "wp2", 3.0));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "wp3", 4.0));

  std::vector<double> before_execution_1, before_execution_2, before_execution_3, final_position;

  // Before the first execution robot should be at position "home"
  before_execution_1 = user_robot->getJointState();
  EXPECT_NO_THROW(user_robot->execute(TRAJECTORY_NAME));
  ros::Duration(0.5).sleep();

  // Second Execution
  // Robot should be at "wp3" at all checkpoints from here on out.
  before_execution_2 = user_robot->getJointState();
  EXPECT_NO_THROW(user_robot->execute(TRAJECTORY_NAME));
  ros::Duration(0.5).sleep();

  // Third Execution
  before_execution_3 = user_robot->getJointState();
  EXPECT_NO_THROW(user_robot->execute(TRAJECTORY_NAME));
  ros::Duration(0.5).sleep();

  // Final Position
  final_position = user_robot->getJointState();

  ROS_INFO_STREAM("before_execution_1" << before_execution_1);
  ROS_INFO_STREAM("before_execution_2" << before_execution_2);
  ROS_INFO_STREAM("before_execution_3" << before_execution_3);
  ROS_INFO_STREAM("final_poistion" << final_position);

  // Testing
  EXPECT_FALSE(before_execution_1 == before_execution_2);
  EXPECT_TRUE(before_execution_2 == before_execution_3);
  EXPECT_TRUE(before_execution_2 == final_position);
}


TEST_F(DeveloperRobotTest, collision)
{
  const std::string TRAJECTORY_NAME("traj1");
  const moveit_simple::InterpolationType cart = moveit_simple::interpolation_type::CARTESIAN;
  const moveit_simple::InterpolationType joint = moveit_simple::interpolation_type::JOINT;

  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  std::vector<double>joint1(6,0);
  std::vector<double>joint2(6,0);
  std::vector<double>joint3(6,0);

  joint2[1] = M_PI;
  joint3[3] = -M_PI/2;

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
  EXPECT_NO_THROW(developer_robot->addTrajPoint(TRAJECTORY_NAME, joint_point1));
  // joint interpolation between two joint points
  EXPECT_NO_THROW(developer_robot->addTrajPoint(TRAJECTORY_NAME,  joint_point2,  joint, 1));

  EXPECT_NO_THROW(developer_robot->execute(TRAJECTORY_NAME));
  EXPECT_FALSE(developer_robot->isInCollision(joint1));

  EXPECT_TRUE(developer_robot->isInCollision(joint2));
  EXPECT_FALSE(developer_robot->isInCollision(joint3));
  EXPECT_THROW(developer_robot->execute(TRAJECTORY_NAME, true), moveit_simple::CollisionDetected);

  // Test to see if above collision detection works when you separate out plan(...) & execute(...)
  std::vector<moveit_simple::JointTrajectoryPoint> goal;

  EXPECT_NO_THROW(goal = developer_robot->plan(TRAJECTORY_NAME));
  EXPECT_THROW(developer_robot->execute(goal, true), moveit_simple::CollisionDetected);
}

//
// Disabling this test temporarily since isNearSingular always returns true for the MH5.
//
/*
TEST_F(UserRobotTest, singularity)
{
  std::vector<double>joint1(6,0);
  std::vector<double>joint2(6,0);
  std::vector<double>joint3(6,0);
  std::vector<double>joint4(6,0);
  std::vector<double>joint5(6,0);

  joint1[4] = M_PI/2;

  joint2[4] = M_PI/60;

  joint3[2] = -M_PI/2 + M_PI/90;
  joint3[4] = M_PI/2;

  joint4[1] = 3*M_PI/8 + M_PI/60;
  joint4[2] = 5*M_PI/8 - M_PI/90;
  joint4[4] = M_PI/2+M_PI/60;

  joint5[1] = M_PI/4;
  joint5[2] = -M_PI/2+M_PI/60;
  joint5[4] = M_PI/60;

  ROS_INFO_STREAM("joint1: " << joint1);
  ROS_INFO_STREAM("joint2: " << joint2);
  ROS_INFO_STREAM("joint3: " << joint3);
  ROS_INFO_STREAM("joint4: " << joint4);
  ROS_INFO_STREAM("joint5: " << joint5);

  EXPECT_FALSE(user_robot->isNearSingular(joint1));
  // Axes 4 and 6 aligned (wrist singularity)
  EXPECT_TRUE(user_robot->isNearSingular(joint2));
  // Elbow is straight (Elbow Singularity)
  EXPECT_TRUE(user_robot->isNearSingular(joint3));
  // Axes 1 and 6 are parallel (Alignment Singularity)
  EXPECT_TRUE(user_robot->isNearSingular(joint4));
  // Boundary Singularity (occurs because of elbow and wrist singularity)
  EXPECT_TRUE(user_robot->isNearSingular(joint5));
}
*/

TEST_F(UserRobotTest, non_blocking_execution)
{
  const std::string TRAJECTORY_NAME("non_blocking_execution_traj");
  const moveit_simple::InterpolationType cart = moveit_simple::interpolation_type::CARTESIAN;
  const moveit_simple::InterpolationType joint = moveit_simple::interpolation_type::JOINT;

  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "home", 0.5));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "wp1", 1.0, joint, 5));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "tf_pub1", 2.0, cart, 8));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "wp2", 3.0));
  EXPECT_NO_THROW(user_robot->addTrajPoint(TRAJECTORY_NAME, "wp3", 4.0, joint));

  EXPECT_TRUE(user_robot->setExecuteGoal(TRAJECTORY_NAME));

  ASSERT_TRUE(user_robot->startExecution());

  ros::Duration timeout;
  EXPECT_TRUE(user_robot->getExecutionTimeout(timeout));

  auto end_time = ros::Time::now() + timeout;
  while (ros::Time::now() < end_time && user_robot->isExecuting())
  {
    EXPECT_FALSE(user_robot->isExecutionStopped());
  }

  EXPECT_TRUE(user_robot->isExecutionStopped());
}
}
