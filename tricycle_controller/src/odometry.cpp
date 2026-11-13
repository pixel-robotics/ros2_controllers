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

#include "tricycle_controller/odometry.hpp"

#include <algorithm>
#include <stdexcept>

namespace tricycle_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_(0.0),
  linear_y_(0.0),
  angular_(0.0),
  wheelbase_(0.0),
  wheel_radius_(0.0),
  pivot_offset_(0.0),
  pivot_offset_full_steer_(0.0),
  steering_deadband_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_(velocity_rolling_window_size),
  linear_y_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

bool Odometry::update(double Ws, double alpha, const rclcpp::Duration & dt)
{
  double Vx, Vy, theta_dot;
  wheelToTwist(Ws, alpha, Vx, Vy, theta_dot);

  // Integrate odometry:
  integrateExact(Vx * dt.seconds(), Vy * dt.seconds(), theta_dot * dt.seconds());

  // Estimate speeds using a rolling mean to filter them out:
  linear_accumulator_.accumulate(Vx);
  linear_y_accumulator_.accumulate(Vy);
  angular_accumulator_.accumulate(theta_dot);

  linear_ = linear_accumulator_.getRollingMean();
  linear_y_ = linear_y_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void Odometry::wheelToTwist(
  double Ws, double alpha, double & linear, double & lateral, double & angular) const
{
  // using naming convention in http://users.isr.ist.utl.pt/~mir/cadeiras/robmovel/Kinematics.pdf
  // The wheel turns by the true angle delta_t (measured minus the deadband); the robot turns about
  // the pivot c(delta_t) behind base_link, i.e. at the arm L = wheelbase + c from the drive wheel.
  const double Vs = Ws * wheel_radius_;
  const double delta = trueSteering(alpha);
  const double c = getPivotOffset(delta);
  linear = Vs * std::cos(delta);
  angular = Vs * std::sin(delta) / yawRateWheelbase(delta);
  // base_link sits c in front of the point the robot turns about, so it moves sideways at
  // c * angular, in both driving directions. An offset of 0 gives an exact 0.0 (not -0.0).
  lateral = (c != 0.0) ? c * angular : 0.0;
}

void Odometry::updateOpenLoop(
  double linear, double angular, const rclcpp::Duration & dt, double steering)
{
  const double c = getPivotOffset(trueSteering(steering));

  /// Save last linear and angular velocity:
  linear_ = linear;
  linear_y_ = (c != 0.0) ? c * angular : 0.0;
  angular_ = angular;

  /// Integrate odometry:
  integrateExact(linear * dt.seconds(), linear_y_ * dt.seconds(), angular * dt.seconds());
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
  resetAccumulators();
}

void Odometry::setWheelParams(double wheelbase, double wheel_radius)
{
  wheelbase_ = wheelbase;
  wheel_radius_ = wheel_radius;
}

void Odometry::setPivotOffset(double pivot_offset, double pivot_offset_full_steer)
{
  if (!std::isfinite(pivot_offset) || !std::isfinite(pivot_offset_full_steer))
  {
    throw std::invalid_argument("pivot_offset and pivot_offset_full_steer must be finite");
  }
  pivot_offset_ = pivot_offset;
  pivot_offset_full_steer_ = pivot_offset_full_steer;
}

void Odometry::setSteeringDeadband(double steering_deadband)
{
  if (!std::isfinite(steering_deadband) || steering_deadband < 0.0)
  {
    throw std::invalid_argument("steering_deadband must be finite and >= 0");
  }
  steering_deadband_ = steering_deadband;
}

double Odometry::trueSteering(double steering) const
{
  if (steering_deadband_ == 0.0)
  {
    return steering;  // bit for bit the measured angle
  }
  return std::copysign(std::max(std::abs(steering) - steering_deadband_, 0.0), steering);
}

double Odometry::getPivotOffset(double steering_true) const
{
  if (pivot_offset_full_steer_ == 0.0)
  {
    return pivot_offset_;
  }
  const double s = std::sin(steering_true);
  return pivot_offset_ + pivot_offset_full_steer_ * s * s;
}

double Odometry::yawRateWheelbase(double steering_true) const
{
  // Pivot model (stage 2): `wheelbase` is the physical drive wheel -> base_link distance (CAD);
  // the robot turns about the pivot c(delta) behind base_link, so the arm of the yaw rate is
  // wheelbase + c(delta). With c = 0 this is exactly `wheelbase` (x + 0.0 == x bit for bit).
  // Do not configure a site wheelbase that already contains the mean pivot together with c
  // (double counting): the site wheelbases 1.266-1.28 were the average of this L(delta).
  return wheelbase_ + getPivotOffset(steering_true);
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}

// linear / lateral: displacement along the robot's x / y axis over the step [m], angular: heading
// change [rad]. lateral == 0.0 takes exactly the classic (unicycle) path.
void Odometry::integrateRungeKutta2(double linear, double lateral, double angular)
{
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_ += linear * cos(direction);
  y_ += linear * sin(direction);
  if (lateral != 0.0)
  {
    x_ -= lateral * sin(direction);
    y_ += lateral * cos(direction);
  }
  heading_ += angular;
}

void Odometry::integrateExact(double linear, double lateral, double angular)
{
  if (fabs(angular) < 1e-6)
  {
    integrateRungeKutta2(linear, lateral, angular);
  }
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r = linear / angular;
    heading_ += angular;
    x_ += r * (sin(heading_) - sin(heading_old));
    y_ += -r * (cos(heading_) - cos(heading_old));
    if (lateral != 0.0)
    {
      // Constant body velocity v_y at constant yaw rate: integral of v_y * (-sin, cos)(heading).
      // With v_y = c * w this is c * (cos, sin)(heading) - c * (cos, sin)(heading_old): the
      // pivot runs the classic arc and base_link stays c ahead of it.
      const double r_y = lateral / angular;
      x_ += r_y * (cos(heading_) - cos(heading_old));
      y_ += r_y * (sin(heading_) - sin(heading_old));
    }
  }
}

void Odometry::resetAccumulators()
{
  linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  linear_y_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace tricycle_controller
