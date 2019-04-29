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

#include <ros/ros.h>

#include <moveit_simple/moveit_simple.h>
#include <moveit_simple/prettyprint.hpp>


int main(int argc, char **argv)
{
  std::string name = "bin_pick_server_main";
  ros::init(argc, argv, name);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  auto robot = std::unique_ptr<moveit_simple::OnlineRobot> (new moveit_simple::OnlineRobot
                    (ros::NodeHandle(), "robot_description", "manipulator"));

  const std::string trajectory_name("traj1");
  const Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  const moveit_simple::InterpolationType cart = moveit_simple::interpolation_type::CARTESIAN;
  const moveit_simple::InterpolationType joint = moveit_simple::interpolation_type::JOINT;

  robot->addTrajPoint(trajectory_name, "home",      0.5);
  robot->addTrajPoint(trajectory_name, "waypoint1", 1.0, joint, 5);
  robot->addTrajPoint(trajectory_name, "tf_pub1",   2.0, cart, 8);
  robot->addTrajPoint(trajectory_name, "waypoint2", 3.0);
  robot->addTrajPoint(trajectory_name, "waypoint3", 4.0, joint);
  robot->addTrajPoint(trajectory_name, pose, "tool0", 5.0);

  robot->execute(trajectory_name);

  ros::waitForShutdown();
  return 0;

}