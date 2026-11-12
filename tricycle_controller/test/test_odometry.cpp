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

// Pivot-point odometry (pivot_offset / pivot_offset_table). The robot turns about a pivot point
// c behind base_link (tandem fork rollers), so base_link moves sideways at v_y = c * w.

#include <gmock/gmock.h>

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
constexpr double WHEELBASE = 1.266;
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
  double pivot_offset = 0.0, const std::vector<double> & steering_abs = {},
  const std::vector<double> & offset = {})
{
  Odometry odom(WINDOW);
  odom.setWheelParams(WHEELBASE, WHEEL_RADIUS);
  odom.setPivotOffset(pivot_offset, steering_abs, offset);
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

// --- defaults: bit-identical to the odometry before the pivot offset ----------------------------

TEST(PivotOdometryTest, default_odometry_is_bit_identical_to_the_previous_model)
{
  Odometry odom(WINDOW);  // no setPivotOffset at all: constructor defaults
  odom.setWheelParams(WHEELBASE, WHEEL_RADIUS);
  Odometry odom_zero = make_odometry(0.0);  // explicit pivot_offset 0, empty table
  LegacyOdometry legacy(WHEELBASE, WHEEL_RADIUS, WINDOW);

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
  Odometry odom = make_odometry(0.0);
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

// --- lateral velocity ----------------------------------------------------------------------------

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
  // A step in the yaw rate: v_y must follow the same rolling mean as w, sample by sample.
  constexpr double c = 0.03;
  Odometry odom = make_odometry(c);
  const auto dt = rclcpp::Duration::from_seconds(0.02);
  for (int i = 0; i < 3 * static_cast<int>(WINDOW); ++i)
  {
    odom.update(6.0, i < static_cast<int>(WINDOW) ? 0.0 : 0.6, dt);
    EXPECT_NEAR(odom.getLinearY(), c * odom.getAngular(), 1e-12) << "step " << i;
  }
}

TEST(PivotOdometryTest, zero_speed_gives_zero_vy_and_no_motion)
{
  for (double alpha : {0.0, 0.7, -0.7, M_PI_2, -M_PI_2})
  {
    Odometry odom = make_odometry(0.05);
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
  Odometry odom = make_odometry(0.05, {0.0, 1.0}, {0.02, 0.04});
  odom.updateOpenLoop(0.5, 0.2, rclcpp::Duration::from_seconds(0.02), -0.5);
  EXPECT_NEAR(odom.getLinearY(), 0.03 * 0.2, 1e-15);
}

// --- pose integration ----------------------------------------------------------------------------

namespace
{
/// Spin on the spot at steering +-pi/2 until the heading has turned by ~target (rad, signed).
void spin(Odometry & odom, double Ws, double alpha, double target)
{
  // w = Ws * r * sin(alpha) / L; step count chosen for ~target, dt is quantised to ns anyway
  const double w = Ws * WHEEL_RADIUS * std::sin(alpha) / WHEELBASE;
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
  constexpr double c = 0.05;
  // (Ws, alpha): left spin, right spin, and the keep_wheel_side variants (wheel on the other
  // side, turning backwards)
  for (const auto & [Ws, alpha] :
       {std::tuple{4.0, M_PI_2}, std::tuple{4.0, -M_PI_2}, std::tuple{-4.0, M_PI_2},
        std::tuple{-4.0, -M_PI_2}})
  {
    const double target = (Ws * alpha > 0.0 ? 1.0 : -1.0) * M_PI_2;
    Odometry pivot = make_odometry(c);
    Odometry at_base_link = make_odometry(0.0);
    spin(pivot, Ws, alpha, target);
    spin(at_base_link, Ws, alpha, target);

    const double theta = pivot.getHeading();
    ASSERT_NEAR(theta, target, 1e-6);
    ASSERT_EQ(theta, at_base_link.getHeading());  // the yaw rate is unchanged
    // relative to the pivot-at-base_link case, base_link is displaced by c * (cos - 1, sin)
    EXPECT_NEAR(pivot.getX() - at_base_link.getX(), c * (std::cos(theta) - 1.0), 1e-12);
    EXPECT_NEAR(pivot.getY() - at_base_link.getY(), c * std::sin(theta), 1e-12);
    // i.e. -c along and +-c beside the start heading for a quarter turn
    EXPECT_NEAR(pivot.getX() - at_base_link.getX(), -c, 1e-6);
    EXPECT_NEAR(pivot.getY() - at_base_link.getY(), std::copysign(c, target), 1e-6);
  }
}

TEST(PivotOdometryTest, pivot_point_runs_the_classic_trajectory)
{
  // For any drive with a constant c, base_link - c * (cos, sin)(heading) (= the pivot) follows
  // exactly the trajectory of the plain tricycle odometry started at the pivot.
  constexpr double c = 0.04;
  Odometry pivot = make_odometry(c);
  Odometry classic = make_odometry(0.0);
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

TEST(PivotOdometryTest, straight_driving_is_unaffected_by_the_offset)
{
  Odometry pivot = make_odometry(0.05);
  Odometry classic = make_odometry(0.0);
  drive(pivot, 8.0, 0.0, 100);
  drive(classic, 8.0, 0.0, 100);
  EXPECT_EQ(pivot.getX(), classic.getX());
  EXPECT_EQ(pivot.getY(), classic.getY());
  EXPECT_EQ(pivot.getLinearY(), 0.0);
}

// --- steering-dependent table ----------------------------------------------------------------------

TEST(PivotOdometryTest, table_interpolates_linearly_and_clamps)
{
  const std::vector<double> knots = {0.2, 0.5, M_PI_2};
  const std::vector<double> offsets = {0.017, 0.03, 0.05};
  // the constant (0.1) is ignored once the table is set
  Odometry odom = make_odometry(0.1, knots, offsets);

  EXPECT_DOUBLE_EQ(odom.getPivotOffset(0.0), 0.017);    // below the first knot: clamped
  EXPECT_DOUBLE_EQ(odom.getPivotOffset(0.2), 0.017);    // on a knot
  EXPECT_NEAR(odom.getPivotOffset(0.35), 0.0235, 1e-15);   // halfway 0.2 .. 0.5
  EXPECT_NEAR(odom.getPivotOffset(-0.35), 0.0235, 1e-15);  // symmetric in the steering sign
  EXPECT_DOUBLE_EQ(odom.getPivotOffset(0.5), 0.03);
  EXPECT_NEAR(
    odom.getPivotOffset(1.0), 0.03 + (1.0 - 0.5) / (M_PI_2 - 0.5) * (0.05 - 0.03), 1e-15);
  EXPECT_DOUBLE_EQ(odom.getPivotOffset(M_PI_2), 0.05);
  EXPECT_DOUBLE_EQ(odom.getPivotOffset(-M_PI_2), 0.05);
  EXPECT_DOUBLE_EQ(odom.getPivotOffset(2.5), 0.05);  // above the last knot: clamped
  EXPECT_DOUBLE_EQ(odom.getPivotOffset(std::nan("")), 0.05);

  // a single knot is a constant
  Odometry single = make_odometry(0.0, {0.3}, {0.02});
  EXPECT_DOUBLE_EQ(single.getPivotOffset(0.0), 0.02);
  EXPECT_DOUBLE_EQ(single.getPivotOffset(1.0), 0.02);

  // empty table: the constant
  Odometry constant = make_odometry(0.025);
  EXPECT_DOUBLE_EQ(constant.getPivotOffset(0.0), 0.025);
  EXPECT_DOUBLE_EQ(constant.getPivotOffset(1.2), 0.025);
}

TEST(PivotOdometryTest, table_offset_is_used_for_vy_but_not_for_the_yaw_rate)
{
  Odometry table = make_odometry(0.0, {0.0, M_PI_2}, {0.02, 0.06});
  Odometry classic = make_odometry(0.0);
  constexpr double alpha = M_PI_2 / 2.0;  // halfway: c = 0.04
  drive(table, 6.0, alpha);
  drive(classic, 6.0, alpha);
  EXPECT_EQ(table.getAngular(), classic.getAngular());  // L(steering) stays = wheelbase
  EXPECT_EQ(table.getHeading(), classic.getHeading());
  EXPECT_NEAR(table.getLinearY(), 0.04 * table.getAngular(), 1e-12);
}

TEST(PivotOdometryTest, invalid_tables_are_rejected)
{
  Odometry odom(WINDOW);
  EXPECT_THROW(odom.setPivotOffset(0.0, {0.0, 1.0}, {0.02}), std::invalid_argument);
  EXPECT_THROW(odom.setPivotOffset(0.0, {0.5, 0.5}, {0.02, 0.03}), std::invalid_argument);
  EXPECT_THROW(odom.setPivotOffset(0.0, {1.0, 0.5}, {0.02, 0.03}), std::invalid_argument);
  EXPECT_THROW(odom.setPivotOffset(0.0, {-0.1, 0.5}, {0.02, 0.03}), std::invalid_argument);
  EXPECT_THROW(odom.setPivotOffset(0.0, {0.0, 0.5}, {0.02, std::nan("")}), std::invalid_argument);
  EXPECT_THROW(odom.setPivotOffset(std::nan("")), std::invalid_argument);
  EXPECT_NO_THROW(odom.setPivotOffset(-0.02));  // a pivot ahead of base_link (load) is allowed
  EXPECT_DOUBLE_EQ(odom.getPivotOffset(0.3), -0.02);
}
