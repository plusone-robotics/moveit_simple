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

#include <moveit_simple/joint_locker.h>

namespace moveit_simple
{

void JointLocker::lockJoints(const std::vector<double> &prev_joints, std::vector<double> &current_joints, JointLockOptions options)
{
  if ((options & JointLockOptions::LOCK_JOINT_1) != JointLockOptions::LOCK_NONE)
  {
    current_joints[0] = prev_joints[0];
  }

  if ((options & JointLockOptions::LOCK_JOINT_2) != JointLockOptions::LOCK_NONE)
  {
    current_joints[1] = prev_joints[1];
  }

  if ((options & JointLockOptions::LOCK_JOINT_3) != JointLockOptions::LOCK_NONE)
  {
    current_joints[2] = prev_joints[2];
  }

  if ((options & JointLockOptions::LOCK_JOINT_4) != JointLockOptions::LOCK_NONE)
  {
    current_joints[3] = prev_joints[3];
  }

  if ((options & JointLockOptions::LOCK_JOINT_5) != JointLockOptions::LOCK_NONE)
  {
    current_joints[4] = prev_joints[4];
  }

  if ((options & JointLockOptions::LOCK_JOINT_6) != JointLockOptions::LOCK_NONE)
  {
    current_joints[5] = prev_joints[5];
  }
}

std::string JointLocker::resolveToString(const JointLockOptions options)
{
  std::vector<std::string> locked_joints;

  if ((options & JointLockOptions::LOCK_JOINT_1) != JointLockOptions::LOCK_NONE)
  {
    locked_joints.push_back("LOCK_JOINT_1");
  }

  if ((options & JointLockOptions::LOCK_JOINT_2) != JointLockOptions::LOCK_NONE)
  {
    locked_joints.push_back("LOCK_JOINT_2");
  }

  if ((options & JointLockOptions::LOCK_JOINT_3) != JointLockOptions::LOCK_NONE)
  {
    locked_joints.push_back("LOCK_JOINT_3");
  }

  if ((options & JointLockOptions::LOCK_JOINT_4) != JointLockOptions::LOCK_NONE)
  {
    locked_joints.push_back("LOCK_JOINT_4");
  }

  if ((options & JointLockOptions::LOCK_JOINT_5) != JointLockOptions::LOCK_NONE)
  {
    locked_joints.push_back("LOCK_JOINT_5");
  }

  if ((options & JointLockOptions::LOCK_JOINT_6) != JointLockOptions::LOCK_NONE)
  {
    locked_joints.push_back("LOCK_JOINT_6");
  }

  if (locked_joints.size() == 0)
  {
    return std::string("LOCK_NONE");
  }
  else
  {
    std::string locked_joints_str;
    for (size_t i = 0; i < locked_joints.size(); ++i)
    {
      if (i == 0)
      {
        locked_joints_str += locked_joints[i];
      }
      else
      {
        locked_joints_str += ", " + locked_joints[i];
      }
    }

    return locked_joints_str;
  }
}

} // namespace moveit_simple
