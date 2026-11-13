// Copyright 2026 Pixel Robotics.
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

// Pivot-point odometry (model A): steering deadband b, pivot c(delta) = c0 + c1 * sin^2(delta_t)
// behind base_link (tandem fork rollers). The robot turns about the pivot: w = v_d sin(delta_t) / L
// with L = wheelbase + c, and base_link moves sideways at v_y = c * w.

#include <gmock/gmock.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "tricycle_controller/odometry.hpp"

using tricycle_controller::Odometry;

namespace
{
constexpr double WHEELBASE = 1.2526;  // CAD
constexpr double WHEEL_RADIUS = 0.125;
constexpr size_t WINDOW = 10;

uint64_t bits(double v)
{
  uint64_t b;
  std::memcpy(&b, &v, sizeof(b));
  return b;
}

#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<double>;
#else
using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;
#endif

/// The odometry as it was before the pivot offset (b44c73e), copied verbatim, as the reference
/// for the bit-identical default.
class LegacyOdometry
{
public:
  LegacyOdometry(double wheelbase, double wheel_radius, size_t window)
  : wheelbase_(wheelbase), wheel_radius_(wheel_radius), lin_acc_(window), ang_acc_(window)
  {
  }

  void update(double Ws, double alpha, const rclcpp::Duration & dt)
  {
    double Vs = Ws * wheel_radius_;
    double Vx = Vs * std::cos(alpha);
    double theta_dot = Vs * std::sin(alpha) / wheelbase_;
    integrateExact(Vx * dt.seconds(), theta_dot * dt.seconds());
    lin_acc_.accumulate(Vx);
    ang_acc_.accumulate(theta_dot);
    linear_ = lin_acc_.getRollingMean();
    angular_ = ang_acc_.getRollingMean();
  }

  void updateOpenLoop(double linear, double angular, const rclcpp::Duration & dt)
  {
    linear_ = linear;
    angular_ = angular;
    integrateExact(linear * dt.seconds(), angular * dt.seconds());
  }

  double x_ = 0.0, y_ = 0.0, heading_ = 0.0, linear_ = 0.0, angular_ = 0.0;

private:
  void integrateRungeKutta2(double linear, double angular)
  {
    const double direction = heading_ + angular * 0.5;
    x_ += linear * cos(direction);
    y_ += linear * sin(direction);
    heading_ += angular;
  }

  void integrateExact(double linear, double angular)
  {
    if (fabs(angular) < 1e-6)
    {
      integrateRungeKutta2(linear, angular);
    }
    else
    {
      const double heading_old = heading_;
      const double r = linear / angular;
      heading_ += angular;
      x_ += r * (sin(heading_) - sin(heading_old));
      y_ += -r * (cos(heading_) - cos(heading_old));
    }
  }

  double wheelbase_, wheel_radius_;
  RollingMeanAccumulator lin_acc_, ang_acc_;
};

Odometry make_odometry(
  double pivot_offset = 0.0, double pivot_offset_full_steer = 0.0, double steering_deadband = 0.0,
  double wheelbase = WHEELBASE)
{
  Odometry odom(WINDOW);
  odom.setWheelParams(wheelbase, WHEEL_RADIUS);
  odom.setPivotOffset(pivot_offset, pivot_offset_full_steer);
  odom.setSteeringDeadband(steering_deadband);
  return odom;
}

/// Drive with constant (Ws, alpha) until the rolling window holds only that input.
void drive(Odometry & odom, double Ws, double alpha, int steps = 2 * static_cast<int>(WINDOW))
{
  const auto dt = rclcpp::Duration::from_seconds(0.02);
  for (int i = 0; i < steps; ++i)
  {
    odom.update(Ws, alpha, dt);
  }
}

/// Mixed inputs: forward/reverse, left/right, straight (RK2 branch), tiny yaw (RK2 branch with
/// non-zero angular), spins at +-pi/2 (both wheel sides), standstill, varying dt.
std::vector<std::tuple<double, double, double>> mixed_inputs()
{
  const std::vector<std::tuple<double, double>> ws_alpha = {
    {8.0, 0.0},      {8.0, 0.3},          {8.0, -0.3},        {-6.0, 0.4},
    {-6.0, -0.4},    {4.0, M_PI_2},       {4.0, -M_PI_2},     {-4.0, M_PI_2},
    {0.0, 1.2},      {0.0, 0.0},          {12.0, 1e-8},       {3.3, 1.5},
    {-2.5, -1.45},   {10.0, 0.05},        {7.0, -1.0},        {1e-3, 0.9},
  };
  const std::vector<double> dts = {0.02, 0.01, 0.033, 0.02};
  std::vector<std::tuple<double, double, double>> out;
  for (int rep = 0; rep < 13; ++rep)
  {
    for (size_t i = 0; i < ws_alpha.size(); ++i)
    {
      out.emplace_back(
        std::get<0>(ws_alpha[i]) * (1.0 + 0.01 * rep), std::get<1>(ws_alpha[i]),
        dts[(i + static_cast<size_t>(rep)) % dts.size()]);
    }
  }
  return out;
}
}  // namespace

// --- defaults: bit-identical to the odometry before the pivot model ------------------------------

TEST(PivotOdometryTest, default_odometry_is_bit_identical_to_the_previous_model)
{
  Odometry odom(WINDOW);  // no setPivotOffset / setSteeringDeadband at all: constructor defaults
  odom.setWheelParams(WHEELBASE, WHEEL_RADIUS);
  Odometry odom_zero = make_odometry(0.0, 0.0, 0.0);  // explicit zeros
  LegacyOdometry legacy(WHEELBASE, WHEEL_RADIUS, WINDOW);
  ASSERT_FALSE(odom.hasPivotModel());
  ASSERT_FALSE(odom_zero.hasPivotModel());

  for (const auto & [Ws, alpha, dt_s] : mixed_inputs())
  {
    const auto dt = rclcpp::Duration::from_seconds(dt_s);
    odom.update(Ws, alpha, dt);
    odom_zero.update(Ws, alpha, dt);
    legacy.update(Ws, alpha, dt);
    for (const Odometry * o : {&odom, &odom_zero})
    {
      ASSERT_EQ(bits(o->getX()), bits(legacy.x_));
      ASSERT_EQ(bits(o->getY()), bits(legacy.y_));
      ASSERT_EQ(bits(o->getHeading()), bits(legacy.heading_));
      ASSERT_EQ(bits(o->getLinear()), bits(legacy.linear_));
      ASSERT_EQ(bits(o->getAngular()), bits(legacy.angular_));
      // the message's twist.linear.y used to be left at its initial +0.0
      ASSERT_EQ(bits(o->getLinearY()), bits(0.0));
    }
  }
  EXPECT_NE(legacy.x_, 0.0);  // the inputs did move the robot
  EXPECT_NE(legacy.y_, 0.0);
}

TEST(PivotOdometryTest, default_open_loop_odometry_is_bit_identical_to_the_previous_model)
{
  Odometry odom = make_odometry();
  LegacyOdometry legacy(WHEELBASE, WHEEL_RADIUS, WINDOW);
  for (const auto & [Ws, alpha, dt_s] : mixed_inputs())
  {
    const auto dt = rclcpp::Duration::from_seconds(dt_s);
    const double linear = Ws * WHEEL_RADIUS * std::cos(alpha);
    const double angular = linear * std::tan(alpha) / WHEELBASE;
    odom.updateOpenLoop(linear, angular, dt, alpha);
    legacy.updateOpenLoop(linear, angular, dt);
    ASSERT_EQ(bits(odom.getX()), bits(legacy.x_));
    ASSERT_EQ(bits(odom.getY()), bits(legacy.y_));
    ASSERT_EQ(bits(odom.getHeading()), bits(legacy.heading_));
    ASSERT_EQ(bits(odom.getLinearY()), bits(0.0));
  }
}

TEST(PivotOdometryTest, default_model_helpers_are_the_identity)
{
  const Odometry odom = make_odometry();
  for (double a : {0.0, -0.0, 1e-9, -0.3, 1.2, M_PI_2, -M_PI_2})
  {
    EXPECT_EQ(bits(odom.trueSteering(a)), bits(a));
    EXPECT_EQ(bits(odom.yawRateWheelbase(a)), bits(WHEELBASE));
    EXPECT_EQ(bits(odom.getPivotOffset(a)), bits(0.0));
  }
}

// --- steering deadband ---------------------------------------------------------------------------

TEST(PivotOdometryTest, deadband_true_steering_is_symmetric_and_clamped_at_zero)
{
  constexpr double b = 0.006;
  const Odometry odom = make_odometry(0.0, 0.0, b);
  EXPECT_EQ(odom.trueSteering(0.0), 0.0);
  EXPECT_EQ(odom.trueSteering(0.004), 0.0);
  EXPECT_EQ(odom.trueSteering(-0.004), 0.0);
  EXPECT_EQ(odom.trueSteering(b), 0.0);
  EXPECT_NEAR(odom.trueSteering(0.5), 0.5 - b, 1e-15);
  EXPECT_NEAR(odom.trueSteering(-0.5), -0.5 + b, 1e-15);
  EXPECT_NEAR(odom.trueSteering(M_PI_2), M_PI_2 - b, 1e-15);
}

TEST(PivotOdometryTest, steering_inside_the_deadband_drives_straight)
{
  constexpr double b = 0.0066;  // 0.38 deg
  for (const double alpha : {0.0065, -0.0065, 0.003, -0.001})
  {
    for (const double Ws : {8.0, -6.0})
    {
      Odometry odom = make_odometry(0.01, 0.03, b);
      drive(odom, Ws, alpha, 100);
      EXPECT_EQ(odom.getAngular(), 0.0) << alpha << " " << Ws;
      EXPECT_EQ(odom.getLinearY(), 0.0) << alpha << " " << Ws;
      EXPECT_EQ(odom.getHeading(), 0.0) << alpha << " " << Ws;
      EXPECT_EQ(odom.getY(), 0.0) << alpha << " " << Ws;
      EXPECT_NEAR(odom.getLinear(), Ws * WHEEL_RADIUS, 1e-12);  // the full wheel speed
      EXPECT_NEAR(odom.getX(), 100 * 0.02 * Ws * WHEEL_RADIUS, 1e-9);
    }
  }
}

TEST(PivotOdometryTest, yaw_rate_follows_the_steering_minus_the_deadband)
{
  constexpr double b = 0.0089;  // 0.51 deg
  const Odometry odom = make_odometry(0.0, 0.0, b);
  for (const double alpha : {0.02, -0.02, 0.1, -0.4, 1.0, -M_PI_2})
  {
    for (const double Ws : {8.0, -3.0})
    {
      double vx, vy, w;
      odom.wheelToTwist(Ws, alpha, vx, vy, w);
      const double v_d = Ws * WHEEL_RADIUS;
      const double expected = v_d * std::copysign(std::sin(std::abs(alpha) - b), alpha) / WHEELBASE;
      EXPECT_NEAR(w, expected, 1e-14) << alpha << " " << Ws;
      EXPECT_NEAR(vx, v_d * std::cos(std::abs(alpha) - b), 1e-14);
      EXPECT_EQ(vy, 0.0);
    }
  }
  // the small-angle signature the fit found: L_eff = L * sin(delta) / sin(delta - b) ~ cot(delta)
  double vx, vy, w;
  odom.wheelToTwist(8.0, 0.2, vx, vy, w);
  const double l_eff = 8.0 * WHEEL_RADIUS * std::sin(0.2) / w;
  EXPECT_NEAR(l_eff / WHEELBASE, std::sin(0.2) / std::sin(0.2 - b), 1e-12);
  // to first order b * cot(delta) (+4.4 %); the second-order term adds ~ b^2 cot^2 = 0.2 %
  EXPECT_NEAR(l_eff / WHEELBASE - 1.0, b / std::tan(0.2), 3e-3);
}

// --- pivot c(delta), L(delta) and v_y --------------------------------------------------------------

TEST(PivotOdometryTest, pivot_offset_and_wheelbase_follow_c0_plus_c1_sin_squared)
{
  constexpr double c0 = 0.005, c1 = 0.025;
  const Odometry odom = make_odometry(c0, c1);
  EXPECT_DOUBLE_EQ(odom.getPivotOffset(0.0), c0);
  EXPECT_DOUBLE_EQ(odom.getPivotOffset(M_PI_2), c0 + c1);
  EXPECT_DOUBLE_EQ(odom.getPivotOffset(-M_PI_2), c0 + c1);
  EXPECT_NEAR(odom.getPivotOffset(M_PI_4), c0 + 0.5 * c1, 1e-15);
  EXPECT_NEAR(odom.getPivotOffset(-1.3), c0 + c1 * std::pow(std::sin(1.3), 2), 1e-15);
  for (double a : {0.0, 0.3, -0.8, 1.3, M_PI_2})
  {
    EXPECT_DOUBLE_EQ(odom.yawRateWheelbase(a), WHEELBASE + odom.getPivotOffset(a));
  }
}

TEST(PivotOdometryTest, yaw_rate_arm_and_vy_are_consistent_with_c)
{
  constexpr double c0 = 0.004, c1 = 0.03, b = 0.005;
  const Odometry odom = make_odometry(c0, c1, b);
  for (const double alpha : {0.05, -0.3, 0.9, -1.4, M_PI_2})
  {
    for (const double Ws : {6.0, -6.0})
    {
      double vx, vy, w;
      odom.wheelToTwist(Ws, alpha, vx, vy, w);
      const double v_d = Ws * WHEEL_RADIUS;
      const double dt = odom.trueSteering(alpha);
      const double c = c0 + c1 * std::sin(dt) * std::sin(dt);
      EXPECT_NEAR(w * (WHEELBASE + c), v_d * std::sin(dt), 1e-14);  // w = v_d sin / (L0 + c)
      EXPECT_NEAR(vx, v_d * std::cos(dt), 1e-14);
      EXPECT_NEAR(vy, c * w, 1e-15);                                 // v_y = c * w
      // the drive wheel's lateral speed in the body frame is w * L: the pivot has no lateral speed
      EXPECT_NEAR(vy - c * w, 0.0, 1e-15);
    }
  }
}

TEST(PivotOdometryTest, left_turn_forward_and_reverse_both_give_vy_equal_c_times_w)
{
  constexpr double c = 0.05;
  // forward, steering left: w > 0
  {
    Odometry odom = make_odometry(c);
    drive(odom, 8.0, 0.4);
    ASSERT_GT(odom.getLinear(), 0.0);
    ASSERT_GT(odom.getAngular(), 0.0);
    EXPECT_GT(odom.getLinearY(), 0.0);
    EXPECT_NEAR(odom.getLinearY(), c * odom.getAngular(), 1e-12);
  }
  // reverse, turning left (heading increases): wheel backwards, steering right
  {
    Odometry odom = make_odometry(c);
    drive(odom, -8.0, -0.4);
    ASSERT_LT(odom.getLinear(), 0.0);
    ASSERT_GT(odom.getAngular(), 0.0);
    EXPECT_GT(odom.getLinearY(), 0.0);
    EXPECT_NEAR(odom.getLinearY(), c * odom.getAngular(), 1e-12);
  }
  // right turns, forward and reverse: v_y = c * w < 0
  for (const auto & [Ws, alpha] : {std::tuple{8.0, -0.4}, std::tuple{-8.0, 0.4}})
  {
    Odometry odom = make_odometry(c);
    drive(odom, Ws, alpha);
    ASSERT_LT(odom.getAngular(), 0.0);
    EXPECT_LT(odom.getLinearY(), 0.0);
    EXPECT_NEAR(odom.getLinearY(), c * odom.getAngular(), 1e-12);
  }
}

TEST(PivotOdometryTest, vy_is_the_rolling_mean_of_c_times_w)
{
  // A step in the yaw rate at constant c: v_y must follow the same rolling mean as w.
  constexpr double c = 0.03;
  Odometry odom = make_odometry(c);
  const auto dt = rclcpp::Duration::from_seconds(0.02);
  for (int i = 0; i < 3 * static_cast<int>(WINDOW); ++i)
  {
    odom.update(6.0, i < static_cast<int>(WINDOW) ? 0.0 : 0.6, dt);
    EXPECT_NEAR(odom.getLinearY(), c * odom.getAngular(), 1e-12) << "step " << i;
  }
}

TEST(PivotOdometryTest, vy_with_steering_dependent_c_is_the_rolling_mean_of_the_samples)
{
  // c changes with the steering: v_y is the mean of the per-sample c * w, not mean(c) * mean(w).
  Odometry odom = make_odometry(0.0, 0.04);
  const auto dt = rclcpp::Duration::from_seconds(0.02);
  std::vector<double> samples;
  for (int i = 0; i < 3 * static_cast<int>(WINDOW); ++i)
  {
    const double alpha = 0.1 * i;
    odom.update(5.0, alpha, dt);
    double vx, vy, w;
    odom.wheelToTwist(5.0, alpha, vx, vy, w);
    samples.push_back(vy);
    double mean = 0.0;
    const size_t n = std::min(samples.size(), WINDOW);
    for (size_t k = samples.size() - n; k < samples.size(); ++k)
    {
      mean += samples[k];
    }
    EXPECT_NEAR(odom.getLinearY(), mean / static_cast<double>(n), 1e-12) << "step " << i;
  }
}

TEST(PivotOdometryTest, zero_speed_gives_zero_vy_and_no_motion)
{
  for (double alpha : {0.0, 0.7, -0.7, M_PI_2, -M_PI_2})
  {
    Odometry odom = make_odometry(0.02, 0.03, 0.005);
    drive(odom, 0.0, alpha);
    EXPECT_EQ(odom.getLinearY(), 0.0) << alpha;
    EXPECT_EQ(odom.getAngular(), 0.0) << alpha;
    EXPECT_EQ(odom.getX(), 0.0) << alpha;
    EXPECT_EQ(odom.getY(), 0.0) << alpha;
    EXPECT_EQ(odom.getHeading(), 0.0) << alpha;
  }
}

TEST(PivotOdometryTest, open_loop_uses_the_offset_at_the_commanded_steering)
{
  constexpr double c0 = 0.01, c1 = 0.04, b = 0.01;
  const Odometry model = make_odometry(c0, c1, b);
  Odometry odom = make_odometry(c0, c1, b);
  odom.updateOpenLoop(0.5, 0.2, rclcpp::Duration::from_seconds(0.02), -0.5);
  const double s = std::sin(0.5 - b);
  EXPECT_NEAR(odom.getLinearY(), (c0 + c1 * s * s) * 0.2, 1e-15);
  EXPECT_NEAR(model.getPivotOffset(model.trueSteering(-0.5)), c0 + c1 * s * s, 1e-15);
}

// --- pose integration ----------------------------------------------------------------------------

namespace
{
/// Spin on the spot at steering +-pi/2 until the heading has turned by ~target (rad, signed).
void spin(Odometry & odom, double Ws, double alpha, double target)
{
  double vx, vy, w;
  odom.wheelToTwist(Ws, alpha, vx, vy, w);
  const int n = 500;
  const auto dt = rclcpp::Duration::from_seconds(target / w / n);
  for (int i = 0; i < n; ++i)
  {
    odom.update(Ws, alpha, dt);
  }
}
}  // namespace

TEST(PivotOdometryTest, spin_of_90_deg_moves_base_link_by_c_cos_minus_1_sin)
{
  // (c0, c1): constant pivot, and the model-A form where the spin pivot is c0 + c1
  for (const auto & [c0, c1] : {std::tuple{0.05, 0.0}, std::tuple{0.0, 0.025}, std::tuple{0.01, 0.03}})
  {
    const double c = c0 + c1;  // at |delta| = pi/2
    // (Ws, alpha): left spin, right spin, and the keep_wheel_side variants (wheel on the other
    // side, turning backwards)
    for (const auto & [Ws, alpha] :
         {std::tuple{4.0, M_PI_2}, std::tuple{4.0, -M_PI_2}, std::tuple{-4.0, M_PI_2},
          std::tuple{-4.0, -M_PI_2}})
    {
      const double target = (Ws * alpha > 0.0 ? 1.0 : -1.0) * M_PI_2;
      Odometry pivot = make_odometry(c0, c1);
      spin(pivot, Ws, alpha, target);

      const double theta = pivot.getHeading();
      ASSERT_NEAR(theta, target, 1e-6);
      // about the pivot at (-c, 0): base_link moves by c * (cos - 1, sin) from the origin
      // (v_x = v_d cos(pi/2) ~ 6e-17 * v_d contributes nothing measurable)
      EXPECT_NEAR(pivot.getX(), c * (std::cos(theta) - 1.0), 1e-12) << c0 << " " << c1;
      EXPECT_NEAR(pivot.getY(), c * std::sin(theta), 1e-12) << c0 << " " << c1;
      // i.e. -c along and +-c beside the start heading for a quarter turn
      EXPECT_NEAR(pivot.getX(), -c, 1e-6);
      EXPECT_NEAR(pivot.getY(), std::copysign(c, target), 1e-6);
      // and the spin takes the arm L(pi/2) = wheelbase + c
      EXPECT_NEAR(pivot.getAngular(), Ws * WHEEL_RADIUS * std::sin(alpha) / (WHEELBASE + c), 1e-12);
    }
  }
}

TEST(PivotOdometryTest, pivot_point_runs_the_classic_trajectory_with_arm_wheelbase_plus_c)
{
  // For any drive with a constant c, base_link - c * (cos, sin)(heading) (= the pivot) follows
  // exactly the trajectory of the plain tricycle odometry with wheelbase + c started at the pivot.
  constexpr double c = 0.04;
  Odometry pivot = make_odometry(c);
  Odometry classic = make_odometry(0.0, 0.0, 0.0, WHEELBASE + c);
  for (const auto & [Ws, alpha, dt_s] : mixed_inputs())
  {
    const auto dt = rclcpp::Duration::from_seconds(dt_s);
    pivot.update(Ws, alpha, dt);
    classic.update(Ws, alpha, dt);
    const double th = pivot.getHeading();
    ASSERT_EQ(th, classic.getHeading());
    // start: base_link at the origin, the pivot at (-c, 0)
    ASSERT_NEAR(pivot.getX() - c * std::cos(th) + c, classic.getX(), 1e-9);
    ASSERT_NEAR(pivot.getY() - c * std::sin(th), classic.getY(), 1e-9);
  }
}

TEST(PivotOdometryTest, straight_driving_is_unaffected_by_the_pivot_model)
{
  Odometry pivot = make_odometry(0.05, 0.03, 0.006);
  Odometry classic = make_odometry();
  drive(pivot, 8.0, 0.0, 100);
  drive(classic, 8.0, 0.0, 100);
  EXPECT_EQ(pivot.getX(), classic.getX());
  EXPECT_EQ(pivot.getY(), classic.getY());
  EXPECT_EQ(pivot.getLinearY(), 0.0);
}

// --- validation ------------------------------------------------------------------------------------

TEST(PivotOdometryTest, invalid_model_values_are_rejected)
{
  Odometry odom(WINDOW);
  EXPECT_THROW(odom.setPivotOffset(std::nan("")), std::invalid_argument);
  EXPECT_THROW(odom.setPivotOffset(0.0, std::nan("")), std::invalid_argument);
  EXPECT_THROW(odom.setPivotOffset(0.0, INFINITY), std::invalid_argument);
  EXPECT_THROW(odom.setSteeringDeadband(-0.001), std::invalid_argument);
  EXPECT_THROW(odom.setSteeringDeadband(std::nan("")), std::invalid_argument);
  EXPECT_NO_THROW(odom.setPivotOffset(-0.02));  // a pivot ahead of base_link (load) is allowed
  EXPECT_DOUBLE_EQ(odom.getPivotOffset(0.3), -0.02);
}
