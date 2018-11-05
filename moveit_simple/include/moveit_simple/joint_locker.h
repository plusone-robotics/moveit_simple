/*
 * Software License Agreement (Apache License)
 *
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

#ifndef JOINT_LOCKER_H
#define JOINT_LOCKER_H

#include <moveit_simple/joint_lock_options.h>
#include <vector>

namespace moveit_simple
{

struct JointLocker
{
  static void lockJoints(const std::vector<double> &prev_joints, std::vector<double> &current_joints,
                         JointLockOptions options = JointLockOptions::LOCK_NONE);
};


} // namespace moveit_simple

#endif // JOINT_LOCKER_H
