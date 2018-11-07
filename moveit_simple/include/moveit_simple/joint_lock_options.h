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

#ifndef JOINT_LOCK_OPTIONS_H
#define JOINT_LOCK_OPTIONS_H

namespace moveit_simple
{

typedef unsigned char uchar_t;

// Only supports robots with 6 joints.
enum class JointLockOptions : uchar_t
{
  LOCK_NONE = 0x00,
  LOCK_JOINT_1 = 1 << 0,
  LOCK_JOINT_2 = 1 << 1,
  LOCK_JOINT_3 = 1 << 2,
  LOCK_JOINT_4 = 1 << 3,
  LOCK_JOINT_5 = 1 << 4,
  LOCK_JOINT_6 = 1 << 5,
};

JointLockOptions operator&(JointLockOptions lhs, JointLockOptions rhs);

JointLockOptions operator|(JointLockOptions lhs, JointLockOptions rhs);

} // namespace moveit_simple

#endif // JOINT_LOCK_OPTIONS_H
