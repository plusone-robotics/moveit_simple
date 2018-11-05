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
#include <moveit_simple/joint_locker.h>

using namespace moveit_simple;
using testing::Types;

namespace moveit_simple_test
{

static const std::vector<double> wp1 = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
static const std::vector<double> wp2 = {2.5, 2.5, 2.5, 2.5, 2.5, 2.5};

TEST(MoveitSimpleTest, joint_locker_default)
{
  auto default_locking(wp2);
  auto default_locking_expected(wp2);
  JointLocker::lockJoints(wp1, default_locking);
  ASSERT_EQ(default_locking, default_locking_expected);
}

TEST(MoveitSimpleTest, joint_locker_no_locking)
{
  auto no_locking(wp2);
  auto no_locking_expected(wp2);
  JointLocker::lockJoints(wp1, no_locking, JointLockOptions::LOCK_NONE);
  ASSERT_EQ(no_locking, no_locking_expected);
}

TEST(MoveitSimpleTest, joint_locker_lock_j1)
{
  auto lock_j1(wp2);
  auto lock_j1_expected(wp2);
  lock_j1_expected[0] = 1.5;
  JointLocker::lockJoints(wp1, lock_j1, JointLockOptions::LOCK_JOINT_1);
  ASSERT_EQ(lock_j1, lock_j1_expected);
}

TEST(MoveitSimpleTest, joint_locker_lock_j2)
{
  auto lock_j2(wp2);
  auto lock_j2_expected(wp2);
  lock_j2_expected[1] = 1.5;
  JointLocker::lockJoints(wp1, lock_j2, JointLockOptions::LOCK_JOINT_2);
  ASSERT_EQ(lock_j2, lock_j2_expected);
}

TEST(MoveitSimpleTest, joint_locker_lock_j3)
{
  auto lock_j3(wp2);
  auto lock_j3_expected(wp2);
  lock_j3_expected[2] = 1.5;
  JointLocker::lockJoints(wp1, lock_j3, JointLockOptions::LOCK_JOINT_3);
  ASSERT_EQ(lock_j3, lock_j3_expected);
}

TEST(MoveitSimpleTest, joint_locker_lock_j4)
{
  auto lock_j4(wp2);
  auto lock_j4_expected(wp2);
  lock_j4_expected[3] = 1.5;
  JointLocker::lockJoints(wp1, lock_j4, JointLockOptions::LOCK_JOINT_4);
  ASSERT_EQ(lock_j4, lock_j4_expected);
}

TEST(MoveitSimpleTest, joint_locker_lock_j5)
{
  auto lock_j5(wp2);
  auto lock_j5_expected(wp2);
  lock_j5_expected[4] = 1.5;
  JointLocker::lockJoints(wp1, lock_j5, JointLockOptions::LOCK_JOINT_5);
  ASSERT_EQ(lock_j5, lock_j5_expected);
}

TEST(MoveitSimpleTest, joint_locker_lock_j6)
{
  auto lock_j6(wp2);
  auto lock_j6_expected(wp2);
  lock_j6_expected[5] = 1.5;
  JointLocker::lockJoints(wp1, lock_j6, JointLockOptions::LOCK_JOINT_6);
  ASSERT_EQ(lock_j6, lock_j6_expected);
}

TEST(MoveitSimpleTest, joint_locker_lock_j1_j3)
{
  auto lock_j1_j3(wp2);
  auto lock_j1_j3_expected(wp2);
  lock_j1_j3_expected[0] = 1.5;
  lock_j1_j3_expected[2] = 1.5;
  JointLocker::lockJoints(wp1, lock_j1_j3, JointLockOptions::LOCK_JOINT_1 | JointLockOptions::LOCK_JOINT_3);
  ASSERT_EQ(lock_j1_j3, lock_j1_j3_expected);
}

TEST(MoveitSimpleTest, joint_locker_lock_j1_j3_j5)
{
  auto lock_j1_j3_j5(wp2);
  auto lock_j1_j3_j5_expected(wp2);
  lock_j1_j3_j5_expected[0] = 1.5;
  lock_j1_j3_j5_expected[2] = 1.5;
  lock_j1_j3_j5_expected[4] = 1.5;
  JointLocker::lockJoints(wp1, lock_j1_j3_j5,
                          JointLockOptions::LOCK_JOINT_1 | JointLockOptions::LOCK_JOINT_3 | JointLockOptions::LOCK_JOINT_5);
  ASSERT_EQ(lock_j1_j3_j5, lock_j1_j3_j5_expected);
}

TEST(MoveitSimpleTest, joint_locker_lock_j4_j6)
{
  auto lock_j4_j6(wp2);
  auto lock_j4_j6_expected(wp2);
  lock_j4_j6_expected[3] = 1.5;
  lock_j4_j6_expected[5] = 1.5;
  JointLocker::lockJoints(wp1, lock_j4_j6, JointLockOptions::LOCK_JOINT_4 | JointLockOptions::LOCK_JOINT_6);
  ASSERT_EQ(lock_j4_j6, lock_j4_j6_expected);
}

TEST(MoveitSimpleTest, joint_locker_lock_all)
{
  auto lock_all(wp2);
  auto lock_all_expected(wp1);
  JointLocker::lockJoints(wp1, lock_all, JointLockOptions::LOCK_JOINT_1 | JointLockOptions::LOCK_JOINT_2 | JointLockOptions::LOCK_JOINT_3 |
                          JointLockOptions::LOCK_JOINT_4 | JointLockOptions::LOCK_JOINT_5 | JointLockOptions::LOCK_JOINT_6);
  ASSERT_EQ(lock_all, lock_all_expected)  ;
}

TEST(MoveitSimpleTest, resolve_to_string_empty)
{
  auto str = JointLocker::resolveToString();
  auto str_expected = std::string("LOCK_NONE");
  ASSERT_EQ(str, str_expected);
}

TEST(MoveitSimpleTest, resolve_to_string_none)
{
  auto str = JointLocker::resolveToString();
  auto str_expected = std::string("LOCK_NONE");
  ASSERT_EQ(str, str_expected);
}

TEST(MoveitSimpleTest, resolve_to_string_single_j1)
{
  auto str = JointLocker::resolveToString(JointLockOptions::LOCK_JOINT_1);
  auto str_expected = std::string("LOCK_JOINT_1");
  ASSERT_EQ(str, str_expected);
}

TEST(MoveitSimpleTest, resolve_to_string_single_j2)
{
  auto str = JointLocker::resolveToString(JointLockOptions::LOCK_JOINT_2);
  auto str_expected = std::string("LOCK_JOINT_2");
  ASSERT_EQ(str, str_expected);
}

TEST(MoveitSimpleTest, resolve_to_string_single_j3)
{
  auto str = JointLocker::resolveToString(JointLockOptions::LOCK_JOINT_3);
  auto str_expected = std::string("LOCK_JOINT_3");
  ASSERT_EQ(str, str_expected);
}

TEST(MoveitSimpleTest, resolve_to_string_single_j4)
{
  auto str = JointLocker::resolveToString(JointLockOptions::LOCK_JOINT_4);
  auto str_expected = std::string("LOCK_JOINT_4");
  ASSERT_EQ(str, str_expected);
}

TEST(MoveitSimpleTest, resolve_to_string_single_j5)
{
  auto str = JointLocker::resolveToString(JointLockOptions::LOCK_JOINT_5);
  auto str_expected = std::string("LOCK_JOINT_5");
  ASSERT_EQ(str, str_expected);
}

TEST(MoveitSimpleTest, resolve_to_string_single_j6)
{
  auto str = JointLocker::resolveToString(JointLockOptions::LOCK_JOINT_6);
  auto str_expected = std::string("LOCK_JOINT_6");
  ASSERT_EQ(str, str_expected);
}

TEST(MoveitSimpleTest, resolve_to_string_double)
{
  auto str = JointLocker::resolveToString(JointLockOptions::LOCK_JOINT_1 | JointLockOptions::LOCK_JOINT_3);
  auto str_expected = std::string("LOCK_JOINT_1, LOCK_JOINT_3");
  ASSERT_EQ(str, str_expected);
}

TEST(MoveitSimpleTest, resolve_to_string_triple)
{
  auto str = JointLocker::resolveToString(JointLockOptions::LOCK_JOINT_1 | JointLockOptions::LOCK_JOINT_2 |
                                          JointLockOptions::LOCK_JOINT_4);
  auto str_expected = std::string("LOCK_JOINT_1, LOCK_JOINT_2, LOCK_JOINT_4");
  ASSERT_EQ(str, str_expected);
}

TEST(MoveitSimpleTest, resolve_to_string_quad)
{
  auto str = JointLocker::resolveToString(JointLockOptions::LOCK_JOINT_1 | JointLockOptions::LOCK_JOINT_3 |
                                          JointLockOptions::LOCK_JOINT_5 | JointLockOptions::LOCK_JOINT_6);
  auto str_expected = std::string("LOCK_JOINT_1, LOCK_JOINT_3, LOCK_JOINT_5, LOCK_JOINT_6");
  ASSERT_EQ(str, str_expected);
}

TEST(MoveitSimpleTest, resolve_to_string_all)
{
  auto str = JointLocker::resolveToString(JointLockOptions::LOCK_JOINT_1 | JointLockOptions::LOCK_JOINT_2 |
                                          JointLockOptions::LOCK_JOINT_3 | JointLockOptions::LOCK_JOINT_4 |
                                          JointLockOptions::LOCK_JOINT_5 | JointLockOptions::LOCK_JOINT_6);
  auto str_expected = std::string("LOCK_JOINT_1, LOCK_JOINT_2, LOCK_JOINT_3, LOCK_JOINT_4, LOCK_JOINT_5, LOCK_JOINT_6");
  ASSERT_EQ(str, str_expected);
}

} // namespace moveit_simple_test
