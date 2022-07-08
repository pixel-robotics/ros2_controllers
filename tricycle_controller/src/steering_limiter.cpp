// Copyright 2020 PAL Robotics S.L.
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
 * Author: Enrique Fernández
 */

#include <algorithm>
#include <stdexcept>

#include "tricycle_controller/steering_limiter.hpp"
#include "rcppmath/clamp.hpp"

namespace tricycle_controller
{
SteeringLimiter::SteeringLimiter(
  bool has_position_limits, bool has_velocity_limits, bool has_acceleration_limits, double min_position,
  double max_position, double min_velocity,
  double max_velocity, double min_acceleration, double max_acceleration)
: has_position_limits_(has_position_limits),
  has_velocity_limits_(has_velocity_limits),
  has_acceleration_limits_(has_acceleration_limits),
  min_position_(min_position),
  max_position_(max_position),
  min_velocity_(min_velocity),
  max_velocity_(max_velocity),
  min_acceleration_(min_acceleration),
  max_acceleration_(max_acceleration)
{
  // Check if limits are valid, max must be specified, min defaults to -max if unspecified
  if (has_position_limits_)
  {
    if (std::isnan(max_position_))
    {
      throw std::runtime_error("Cannot apply position limits if max_position is not specified");
    }
    if (std::isnan(min_position_))
    {
      min_position_ = -max_position_;
    }
  }
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
      min_acceleration_ = -max_acceleration_;
    }
  }
}

double SteeringLimiter::limit(double & p, double p0, double p1, double dt)
{
  const double tmp = p;

  limit_acceleration(p, p0, p1, dt);
  limit_velocity(p, p0, dt);
  limit_position(p);

  return tmp != 0.0 ? p / tmp : 1.0;
}


double SteeringLimiter::limit_position(double & p)
{
  const double tmp = p;

  if (has_position_limits_)
  {
    p = rcppmath::clamp(p, min_position_, max_position_);
  }

  return tmp != 0.0 ? p / tmp : 1.0;
}

double SteeringLimiter::limit_velocity(double & p, double p0, double dt)
{
  const double tmp = p;

  if (has_velocity_limits_)
  {
    const double dv_min = min_velocity_ * dt;
    const double dv_max = max_velocity_ * dt;

    const double dv = rcppmath::clamp(p - p0, dv_min, dv_max);

    p = p0 + dv;
  }

  return tmp != 0.0 ? p / tmp : 1.0;
}

double SteeringLimiter::limit_acceleration(double & p, double p0, double p1, double dt)
{
  const double tmp = p;

  if (has_acceleration_limits_)
  {
    const double dv = p - p0;
    const double dp0 = p0 - p1;

    const double dt2 = 2. * dt * dt;

    const double da_min = min_acceleration_ * dt2;
    const double da_max = max_acceleration_ * dt2;

    const double da = rcppmath::clamp(dv - dp0, da_min, da_max);

    p = p0 + dp0 + da;
  }

  return tmp != 0.0 ? p / tmp : 1.0;
}


}  // namespace tricycle_controller
