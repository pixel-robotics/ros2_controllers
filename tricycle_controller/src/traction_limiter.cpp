// Copyright 2022 Pixel Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Tony Najjar
 */

#include <algorithm>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rcppmath/clamp.hpp"
#include "tricycle_controller/traction_limiter.hpp"

namespace tricycle_controller
{
TractionLimiter::TractionLimiter(
  bool has_velocity_limits, bool has_acceleration_limits, bool has_deceleration_limits,
  bool has_jerk_limits, double min_velocity, double max_velocity, double min_acceleration,
  double max_acceleration, double min_deceleration, double max_deceleration, double min_jerk,
  double max_jerk)
: has_velocity_limits_(has_velocity_limits),
  has_acceleration_limits_(has_acceleration_limits),
  has_deceleration_limits_(has_deceleration_limits),
  has_jerk_limits_(has_jerk_limits),
  min_velocity_(min_velocity),
  max_velocity_(max_velocity),
  min_acceleration_(min_acceleration),
  max_acceleration_(max_acceleration),
  min_deceleration_(min_deceleration),
  max_deceleration_(max_deceleration),
  min_jerk_(min_jerk),
  max_jerk_(max_jerk)
{
  // Check if limits are valid, max must be specified, min defaults to -max if unspecified
  if (has_velocity_limits_)
  {
    if (std::isnan(max_velocity_))
    {
      throw std::runtime_error("Cannot apply velocity limits if max_velocity is not specified");
    }
    if (std::isnan(min_velocity_))
    {
      min_velocity_ = -max_velocity_;
    }
  }
  if (has_acceleration_limits_)
  {
    if (std::isnan(max_acceleration_))
    {
      throw std::runtime_error(
        "Cannot apply acceleration limits if max_acceleration is not specified");
    }
    if (std::isnan(min_acceleration_))
    {
      min_acceleration_ = 0;
    }
    else if (min_acceleration_ < 0)
    {
      throw std::runtime_error("Acceleration cannot be negative");
    }
  }
  if (has_deceleration_limits_)
  {
    if (std::isnan(max_deceleration_))
    {
      throw std::runtime_error(
        "Cannot apply acceleration limits if max_acceleration is not specified");
    }
    if (std::isnan(min_deceleration_))
    {
      min_deceleration_ = 0;
    }
  }
  if (has_jerk_limits_)
  {
    if (std::isnan(max_jerk_))
    {
      throw std::runtime_error("Cannot apply jerk limits if max_jerk is not specified");
    }
    if (std::isnan(min_jerk_))
    {
      min_jerk_ = -max_jerk_;
    }
  }
}

double TractionLimiter::limit(double & v, double v0, double v1, double dt)
{
  const double tmp = v;

  limit_jerk(v, v0, v1, dt);
  RCLCPP_ERROR(rclcpp::get_logger("test"), "v: %f, v0: %f", v, v0);
  // if ((v >= 0) != (v0 >= 0))
  //   v = 0;
  limit_acceleration(v, v0, dt);
  limit_velocity(v);

  return tmp != 0.0 ? v / tmp : 1.0;
}

double TractionLimiter::limit_velocity(double & v)
{
  const double tmp = v;

  if (has_velocity_limits_)
  {
    v = rcppmath::clamp(v, min_velocity_, max_velocity_);
  }

  return tmp != 0.0 ? v / tmp : 1.0;
}

double TractionLimiter::limit_acceleration(double & v, double v0, double dt)
{
  const double tmp = v;

  if (has_acceleration_limits_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("test"), "Limit ACC");

    double dv_min;
    double dv_max;

    if (abs(v) >= abs(v0))
    {
      dv_min = min_acceleration_ * dt;
      dv_max = max_acceleration_ * dt;
    }
    else
    {
      dv_min = min_deceleration_ * dt;
      dv_max = max_deceleration_ * dt;
    }

    const double dv = rcppmath::clamp(abs(v - v0), dv_min, dv_max) * (v - v0 > 0 ? 1 : -1);

    v = v0 + dv;
  }

  return tmp != 0.0 ? v / tmp : 1.0;
}

double TractionLimiter::limit_jerk(double & v, double v0, double v1, double dt)
{
  const double tmp = v;

  if (has_jerk_limits_)
  {
    const double dv = v - v0;
    const double dv0 = v0 - v1;

    const double dt2 = 2. * dt * dt;

    const double da_min = min_jerk_ * dt2;
    const double da_max = max_jerk_ * dt2;

    const double da = rcppmath::clamp(dv - dv0, da_min, da_max);

    v = v0 + dv0 + da;
  }

  return tmp != 0.0 ? v / tmp : 1.0;
}

}  // namespace tricycle_controller
