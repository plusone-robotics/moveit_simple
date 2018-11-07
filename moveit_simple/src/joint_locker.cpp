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
  for (auto i = 0UL; i < sizeof(JointLockOptions); i++)
  {
    if ((static_cast<uchar_t>(options) & (1 << i)) != 0)
    {
      current_joints[i] = prev_joints[i];
    }
  }
}

std::string JointLocker::resolveToString(const JointLockOptions options)
{
  std::string result;
  const std::string prefix = "LOCK_JOINT_";

  for (auto i = 0UL; i < sizeof(options); i++)
  {
    if ((static_cast<uchar_t>(options) & (1 << i)) != 0)
    {
      auto joint_string = prefix + std::to_string(i + 1);
      if (!result.empty())
      {
        result += ", ";
      }

      result += joint_string;
    }
  }

  if (result == "")
  {
    return "LOCK_NONE";
  }

  return result;
}

} // namespace moveit_simple
