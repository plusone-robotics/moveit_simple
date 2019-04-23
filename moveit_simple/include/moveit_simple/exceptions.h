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

#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

#include <stdexcept>
#include <string>

namespace moveit_simple
{
/**
 * @brief ExecutionFailureException: An exception class to notify
 * execution failure
 *
 * This inherits from std::runtime_error.
 * This is an exception class to be thrown when sendGoalAndWait method
 * has failed execution.
 */
class ExecutionFailureException : public std::runtime_error
{
public:
  ExecutionFailureException(const std::string errorDescription)
    : std::runtime_error(errorDescription) { }
};

/**
 * @brief IKFailException: An exception class to notify IK failure
 *
 * This inherits from std::runtime_error.
 * This is an exception class to be thrown when IK call fails to return
 * joint solution.
 */
class IKFailException : public std::runtime_error
{
public:
  IKFailException(const std::string errorDescription)
    : std::runtime_error(errorDescription) { }
};

/**
 * @brief IKSolverTransformException:
 *
 * This inherits from std::runtime_error.
 * This is an exception class to be thrown when a custom iksolver is
 * used but the required transforms cannot be calculated
 */
class IKSolverTransformException : public std::runtime_error
{
public:
  IKSolverTransformException(const std::string error_description)
    : std::runtime_error(error_description) { }
};

/**
 * @brief CollisionDetected: An exception class to notify collision
 *
 * This inherits from std::runtime_error.
 * This is an exception class to be thrown when collision is detected
 * at any interpolated point in the tarjectory.
 */
class CollisionDetected : public std::runtime_error
{
public:
  CollisionDetected(const std::string errorDescription)
    : std::runtime_error(errorDescription) { }
};

/**
 * @brief JointSeedException:
 *
 * This inherits from std::runtime_error.
 * This is an exception class to be thrown when a pre-defined joint
 * seed is given for collision and reach checking but it does not
 * actually exist.
 */
class JointSeedException : public std::runtime_error
{
public:
  JointSeedException(const std::string error_description)
    : std::runtime_error(error_description) { }
};
} // namespace moveit_simple
#endif // EXCEPTIONS_H