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
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_(velocity_rolling_window_size),
  linear_y_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

bool Odometry::update(double Ws, double alpha, const rclcpp::Duration & dt)
{
  // using naming convention in http://users.isr.ist.utl.pt/~mir/cadeiras/robmovel/Kinematics.pdf
  double Vs = Ws * wheel_radius_;
  double Vx = Vs * std::cos(alpha);
  double theta_dot = Vs * std::sin(alpha) / yawRateWheelbase(alpha);
  // base_link sits pivot_offset in front of the point the robot turns about, so it moves sideways
  // at c * theta_dot, in both driving directions. An offset of 0 gives an exact 0.0 (not -0.0).
  const double c = getPivotOffset(alpha);
  const double Vy = (c != 0.0) ? c * theta_dot : 0.0;

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

void Odometry::updateOpenLoop(
  double linear, double angular, const rclcpp::Duration & dt, double steering)
{
  const double c = getPivotOffset(steering);

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

void Odometry::setPivotOffset(
  double pivot_offset, const std::vector<double> & steering_abs, const std::vector<double> & offset)
{
  if (!std::isfinite(pivot_offset))
  {
    throw std::invalid_argument("pivot_offset must be finite");
  }
  if (steering_abs.size() != offset.size())
  {
    throw std::invalid_argument(
      "pivot_offset_table: steering_abs and offset must have the same length");
  }
  for (size_t i = 0; i < steering_abs.size(); ++i)
  {
    if (!std::isfinite(steering_abs[i]) || !std::isfinite(offset[i]) || steering_abs[i] < 0.0)
    {
      throw std::invalid_argument(
        "pivot_offset_table: knots must be finite and steering_abs >= 0");
    }
    if (i > 0 && !(steering_abs[i] > steering_abs[i - 1]))
    {
      throw std::invalid_argument("pivot_offset_table: steering_abs must be strictly increasing");
    }
  }
  pivot_offset_ = pivot_offset;
  pivot_table_steering_abs_ = steering_abs;
  pivot_table_offset_ = offset;
}

double Odometry::getPivotOffset(double steering) const
{
  const auto & knots = pivot_table_steering_abs_;
  const auto & values = pivot_table_offset_;
  if (knots.empty())
  {
    return pivot_offset_;
  }
  const double a = std::abs(steering);
  if (a <= knots.front())
  {
    return values.front();
  }
  if (!(a < knots.back()))  // also catches NaN
  {
    return values.back();
  }
  // knots.front() < a < knots.back(): i in [1, size - 1]
  const size_t i =
    static_cast<size_t>(std::upper_bound(knots.begin(), knots.end(), a) - knots.begin());
  const double t = (a - knots[i - 1]) / (knots[i] - knots[i - 1]);
  return values[i - 1] + t * (values[i] - values[i - 1]);
}

double Odometry::yawRateWheelbase(double /*alpha*/) const
{
  // HOOK (pivot model stage 2, disabled): steering-dependent yaw-rate wheelbase
  //   L(alpha) = wheelbase + c1 * s(alpha),  s(0) = 0, s(pi/2) = 1 (or a monotone knot table),
  // i.e. the drive wheel -> pivot distance grows by the same c1 by which the pivot offset grows
  // with |steering|. `wheelbase` is already drive wheel -> pivot at zero steering, so the constant
  // pivot_offset must never be added here (double counting).
  // TODO(pivot stage 2): enable only after the gyro check of the design review: with
  // pivot_offset ~5 cm at a spin and wheelbase 1.266, Westfalia spins must over-rotate ~2.8 % vs
  // the gyro. If they do not, the spin offset is a camera/marker artefact and c1 stays out of L.
  // When enabled, the inverse kinematics must use L at the COMMANDED steering angle as well.
  return wheelbase_;
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
