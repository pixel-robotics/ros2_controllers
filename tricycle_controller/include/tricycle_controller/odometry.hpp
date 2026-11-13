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
  /// Lateral velocity of base_link (m/s), c * angular; 0 with the default pivot offset.
  double getLinearY() const { return linear_y_; }
  double getAngular() const { return angular_; }

  void setWheelParams(double wheelbase, double wheel_radius);
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

  /**
   * \brief Pivot point of the non-steered axle, c(delta) metres behind base_link.
   *
   * c(delta) = pivot_offset + pivot_offset_full_steer * sin^2(delta_t), delta_t the true steering
   * angle (see trueSteering()). The robot turns about the pivot (zero lateral velocity there), so
   * base_link moves sideways at v_y = c * angular, and the drive wheel -> pivot distance that sets
   * the yaw rate is wheelbase + c (see yawRateWheelbase()). Both 0 (the defaults) give the classic
   * tricycle odometry bit for bit.
   *
   * \throws std::invalid_argument if a value is not finite.
   */
  void setPivotOffset(double pivot_offset, double pivot_offset_full_steer = 0.0);
  /**
   * \brief Symmetric steering deadband \p steering_deadband (rad, >= 0): the wheel turns
   * |delta| - b less than measured, delta_t = sign(delta) * max(|delta| - b, 0). 0 (the default)
   * leaves the measured angle untouched. Odometry only; the inverse kinematics does not compensate
   * it (see TricycleController::twist_to_ackermann).
   *
   * \throws std::invalid_argument if negative or not finite.
   */
  void setSteeringDeadband(double steering_deadband);

  /// True steering angle delta_t (rad) for a measured (or commanded) angle \p steering.
  double trueSteering(double steering) const;
  /// Pivot offset c (m) behind base_link at the TRUE steering angle \p steering_true (rad).
  double getPivotOffset(double steering_true) const;
  /// Yaw-rate wheelbase L = wheelbase + c(steering_true) (m), drive wheel -> pivot.
  double yawRateWheelbase(double steering_true) const;
  /// False when deadband, pivot_offset and pivot_offset_full_steer are all 0 (classic model).
  bool hasPivotModel() const
  {
    return steering_deadband_ != 0.0 || pivot_offset_ != 0.0 || pivot_offset_full_steer_ != 0.0;
  }
  /**
   * \brief Body velocities of base_link for traction wheel speed \p Ws (rad/s) at measured or
   * commanded steering \p alpha (rad): v_d = r * Ws, delta_t = trueSteering(alpha),
   * linear = v_d cos(delta_t), angular = v_d sin(delta_t) / L, lateral = c * angular.
   */
  void wheelToTwist(
    double Ws, double alpha, double & linear, double & lateral, double & angular) const;

private:
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
  using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<double>;
#else
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;
#endif

  void integrateRungeKutta2(double linear, double lateral, double angular);
  void integrateExact(double linear, double lateral, double angular);
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

  // Pivot point behind base_link [m]: c = pivot_offset_ + pivot_offset_full_steer_ * sin^2.
  double pivot_offset_;
  double pivot_offset_full_steer_;
  // Symmetric steering deadband [rad]
  double steering_deadband_;

  // Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_accumulator_;
  RollingMeanAccumulator linear_y_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace tricycle_controller

#endif  // TRICYCLE_CONTROLLER__ODOMETRY_HPP_
