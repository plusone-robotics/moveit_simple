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

#include <gtest/gtest.h>
#include <moveit_simple/joint_lock_options.h>

using namespace moveit_simple;
using testing::Types;

namespace moveit_simple_test
{

std::vector<bool> checkOptions(const JointLockOptions &options)
{
  std::vector<bool> result = {false, false, false, false, false, false};

  if ((options & JointLockOptions::LOCK_JOINT_1) != JointLockOptions::LOCK_NONE)
  {
    result[0] = true;
  }

  if ((options & JointLockOptions::LOCK_JOINT_2) != JointLockOptions::LOCK_NONE)
  {
    result[1] = true;
  }

  if ((options & JointLockOptions::LOCK_JOINT_3) != JointLockOptions::LOCK_NONE)
  {
    result[2] = true;
  }

  if ((options & JointLockOptions::LOCK_JOINT_4) != JointLockOptions::LOCK_NONE)
  {
    result[3] = true;
  }

  if ((options & JointLockOptions::LOCK_JOINT_5) != JointLockOptions::LOCK_NONE)
  {
    result[4] = true;
  }

  if ((options & JointLockOptions::LOCK_JOINT_6) != JointLockOptions::LOCK_NONE)
  {
    result[5] = true;
  }

  return result;
}

TEST(MoveitSimpleTest, joint_locker_options_none)
{
  auto r0 = checkOptions(JointLockOptions::LOCK_NONE);
  EXPECT_FALSE(r0[0]);
  EXPECT_FALSE(r0[1]);
  EXPECT_FALSE(r0[2]);
  EXPECT_FALSE(r0[3]);
  EXPECT_FALSE(r0[4]);
  EXPECT_FALSE(r0[5]);
}

TEST(MoveitSimpleTest, joint_locker_options_j1_j2)
{
  auto r1 = checkOptions(JointLockOptions::LOCK_JOINT_1 | JointLockOptions::LOCK_JOINT_2);
  EXPECT_TRUE(r1[0]);
  EXPECT_TRUE(r1[1]);
  EXPECT_FALSE(r1[2]);
  EXPECT_FALSE(r1[3]);
  EXPECT_FALSE(r1[4]);
  EXPECT_FALSE(r1[5]);
}

TEST(MoveitSimpleTest, joint_locker_options_none_j3)
{
  auto r2 = checkOptions(JointLockOptions::LOCK_NONE | JointLockOptions::LOCK_JOINT_3);
  EXPECT_FALSE(r2[0]);
  EXPECT_FALSE(r2[1]);
  EXPECT_TRUE(r2[2]);
  EXPECT_FALSE(r2[3]);
  EXPECT_FALSE(r2[4]);
  EXPECT_FALSE(r2[5]);
}

TEST(MoveitSimpleTest, joint_locker_options_all)
{
  auto r3 = checkOptions(JointLockOptions::LOCK_JOINT_1 | JointLockOptions::LOCK_JOINT_2 |
                         JointLockOptions::LOCK_JOINT_3 | JointLockOptions::LOCK_JOINT_4 |
                         JointLockOptions::LOCK_JOINT_5 | JointLockOptions::LOCK_JOINT_6);
  EXPECT_TRUE(r3[0]);
  EXPECT_TRUE(r3[1]);
  EXPECT_TRUE(r3[2]);
  EXPECT_TRUE(r3[3]);
  EXPECT_TRUE(r3[4]);
  EXPECT_TRUE(r3[5]);
}

TEST(MoveitSimpleTest, joint_locker_options_all_none)
{
  auto r4 = checkOptions(JointLockOptions::LOCK_JOINT_1 | JointLockOptions::LOCK_JOINT_2 |
                         JointLockOptions::LOCK_JOINT_3 | JointLockOptions::LOCK_JOINT_4 |
                         JointLockOptions::LOCK_JOINT_5 | JointLockOptions::LOCK_JOINT_6 |
                         JointLockOptions::LOCK_NONE);
  EXPECT_TRUE(r4[0]);
  EXPECT_TRUE(r4[1]);
  EXPECT_TRUE(r4[2]);
  EXPECT_TRUE(r4[3]);
  EXPECT_TRUE(r4[4]);
  EXPECT_TRUE(r4[5]);
}

} // namespace moveit_simple_test
