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

#ifndef TRICYCLE_CONTROLLER__TRACTION_LIMITER_HPP_
#define TRICYCLE_CONTROLLER__TRACTION_LIMITER_HPP_

#include <cmath>

namespace tricycle_controller
{

class TractionLimiter
{
public:
  /**
   * \brief Constructor
   * \param [in] min_velocity Minimum velocity [m/s], usually <= 0
   * \param [in] max_velocity Maximum velocity [m/s], usually >= 0
   * \param [in] min_acceleration Minimum acceleration [m/s^2], usually <= 0
   * \param [in] max_acceleration Maximum acceleration [m/s^2], usually >= 0
   * \param [in] min_jerk Minimum jerk [m/s^3], usually <= 0
   * \param [in] max_jerk Maximum jerk [m/s^3], usually >= 0
   */
  TractionLimiter(
    double min_velocity = 0.0, double max_velocity = std::numeric_limits<double>::max(),
    double min_acceleration = 0.0, double max_acceleration = std::numeric_limits<double>::max(),
    double min_deceleration = 0.0, double max_deceleration = std::numeric_limits<double>::max(),
    double min_jerk = 0.0, double max_jerk = std::numeric_limits<double>::max());

  /**
   * \brief Limit the velocity and acceleration
   * \param [in, out] v  Velocity [m/s]
   * \param [in]      v0 Previous velocity to v  [m/s]
   * \param [in]      v1 Previous velocity to v0 [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   */
  double limit(double & v, double v0, double v1, double dt);

  /**
   * \brief Limit the velocity
   * \param [in, out] v Velocity [m/s]
   * \return Limiting factor (1.0 if none)
   */
  double limit_velocity(double & v);

  /**
   * \brief Limit the acceleration
   * \param [in, out] v  Velocity [m/s]
   * \param [in]      v0 Previous velocity [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   */
  double limit_acceleration(double & v, double v0, double dt);

  /**
   * \brief Limit the jerk
   * \param [in, out] v  Velocity [m/s]
   * \param [in]      v0 Previous velocity to v  [m/s]
   * \param [in]      v1 Previous velocity to v0 [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   * \see http://en.wikipedia.org/wiki/Jerk_%28physics%29#Motion_control
   */
  double limit_jerk(double & v, double v0, double v1, double dt);

private:
  // Velocity limits:
  double min_velocity_;
  double max_velocity_;

  // Acceleration limits:
  double min_acceleration_;
  double max_acceleration_;

  // Deceleration limits:
  double min_deceleration_;
  double max_deceleration_;

  // Jerk limits:
  double min_jerk_;
  double max_jerk_;
};

}  // namespace tricycle_controller

#endif  // TRICYCLE_CONTROLLER__TRACTION_LIMITER_HPP_
