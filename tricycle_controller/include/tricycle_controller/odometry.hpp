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

#ifndef TRICYCLE_CONTROLLER__ODOMETRY_HPP_
#define TRICYCLE_CONTROLLER__ODOMETRY_HPP_

#include <cmath>
#include <vector>

#include <rclcpp/duration.hpp>
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
#include "rcpputils/rolling_mean_accumulator.hpp"
#else
#include "rcppmath/rolling_mean_accumulator.hpp"
#endif

namespace tricycle_controller
{
class Odometry
{
public:
  explicit Odometry(size_t velocity_rolling_window_size = 10);

  /// Odometry from the measured traction wheel speed Ws (rad/s) and steering angle alpha (rad).
  bool update(double Ws, double alpha, const rclcpp::Duration & dt);
  /// Odometry from the commanded velocities. \p steering is the commanded steering angle; it only
  /// selects the pivot offset of the lateral velocity (see setPivotOffset()).
  void updateOpenLoop(
    double linear, double angular, const rclcpp::Duration & dt, double steering = 0.0);
  void resetOdometry();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinear() const { return linear_; }
  /// Lateral velocity of base_link (m/s), pivot_offset * angular; 0 with the default pivot offset.
  double getLinearY() const { return linear_y_; }
  double getAngular() const { return angular_; }

  void setWheelParams(double wheelbase, double wheel_radius);
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

  /**
   * \brief Pivot point of the non-steered axle, \p pivot_offset metres behind base_link.
   *
   * The robot turns about the pivot (zero lateral velocity there), so base_link moves sideways at
   * v_y = c * angular. With \p steering_abs / \p offset non-empty, c = c(|steering|) is linearly
   * interpolated between the knots (clamped outside) and replaces the constant. Empty table and
   * offset 0 (the defaults) give the classic tricycle odometry bit for bit.
   *
   * \throws std::invalid_argument if the table sizes differ or steering_abs is not strictly
   *         increasing.
   */
  void setPivotOffset(
    double pivot_offset, const std::vector<double> & steering_abs = {},
    const std::vector<double> & offset = {});
  /// Pivot offset c (m) used for the lateral velocity at steering angle \p steering (rad).
  double getPivotOffset(double steering) const;

private:
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
  using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<double>;
#else
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;
#endif

  void integrateRungeKutta2(double linear, double lateral, double angular);
  void integrateExact(double linear, double lateral, double angular);
  /// Wheelbase for the yaw rate at steering angle \p alpha. Stage-1 hook, see odometry.cpp.
  double yawRateWheelbase(double alpha) const;
  void resetAccumulators();

  // Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  // Current velocity:
  double linear_;   //   [m/s]
  double linear_y_; //   [m/s]
  double angular_;  // [rad/s]

  // Wheel kinematic parameters [m]:
  double wheelbase_;
  double wheel_radius_;

  // Pivot point behind base_link [m]: constant, or c(|steering|) table when non-empty.
  double pivot_offset_;
  std::vector<double> pivot_table_steering_abs_;
  std::vector<double> pivot_table_offset_;

  // Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_accumulator_;
  RollingMeanAccumulator linear_y_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace tricycle_controller

#endif  // TRICYCLE_CONTROLLER__ODOMETRY_HPP_
