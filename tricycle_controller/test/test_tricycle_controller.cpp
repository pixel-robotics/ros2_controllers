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

#include <gmock/gmock.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"
#include "tricycle_controller/tricycle_controller.hpp"

using CallbackReturn = controller_interface::CallbackReturn;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using lifecycle_msgs::msg::State;
using testing::SizeIs;
using testing::UnorderedElementsAre;

namespace
{
const char traction_joint_name[] = "traction_joint";
const char steering_joint_name[] = "steering_joint";
}  // namespace

class TestableTricycleController : public tricycle_controller::TricycleController
{
public:
  using TricycleController::TricycleController;
  /// Inject a command directly (no executor spin needed), stamped with the given time.
  void set_command(double linear, double angular, const rclcpp::Time & stamp)
  {
    auto msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = stamp;
    msg->twist.linear.x = linear;
    msg->twist.angular.z = angular;
    received_velocity_msg_ptr_.set(
      [msg](std::shared_ptr<geometry_msgs::msg::TwistStamped> & stored_value)
      { stored_value = msg; });
  }

  bool churn_active() const { return churn_active_; }
  const tricycle_controller::Odometry & odometry() const { return odometry_; }
  /// The inverse kinematics (cmd_vel -> steering angle, traction wheel speed).
  std::tuple<double, double> inverse(double linear, double angular)
  {
    return twist_to_ackermann(linear, angular);
  }
  void set_use_exact_mode(bool on) { use_exact_mode_ = on; }
  /// Call the set_exact_mode service handler directly (no executor needed).
  tricycle_controller_msgs::srv::SetExactMode::Response call_set_exact_mode(
    bool enable, double threshold = 0.0, bool keep_wheel_side = false)
  {
    auto req = std::make_shared<tricycle_controller_msgs::srv::SetExactMode::Request>();
    req->enable = enable;
    req->threshold = threshold;
    req->keep_wheel_side = keep_wheel_side;
    auto res = std::make_shared<tricycle_controller_msgs::srv::SetExactMode::Response>();
    set_exact_mode(nullptr, req, res);
    return *res;
  }

  /// Call the set_steering service handler directly (no executor needed).
  tricycle_controller_msgs::srv::SetSteering::Response call_set_steering(double angle)
  {
    auto req = std::make_shared<tricycle_controller_msgs::srv::SetSteering::Request>();
    req->angle = angle;
    auto res = std::make_shared<tricycle_controller_msgs::srv::SetSteering::Response>();
    set_steering(nullptr, req, res);
    return *res;
  }

  std::shared_ptr<geometry_msgs::msg::TwistStamped> getLastReceivedTwist()
  {
    std::shared_ptr<geometry_msgs::msg::TwistStamped> ret;
    received_velocity_msg_ptr_.get(
      [&ret](const std::shared_ptr<geometry_msgs::msg::TwistStamped> & msg) { ret = msg; });
    return ret;
  }

  /**
   * @brief wait_for_twist block until a new twist is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function
   */
  void wait_for_twist(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds(500))
  {
    auto until = get_node()->get_clock()->now() + timeout;
    while (get_node()->get_clock()->now() < until)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }
};

class TestTricycleController : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  void SetUp() override
  {
    controller_ = std::make_unique<TestableTricycleController>();
    pub_node = std::make_shared<rclcpp::Node>("velocity_publisher");
    velocity_publisher = pub_node->create_publisher<geometry_msgs::msg::TwistStamped>(
      controller_name + "/cmd_vel", rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  /// Publish velocity msgs
  /**
   *  linear - magnitude of the linear command in the geometry_msgs::twist message
   *  angular - the magnitude of the angular command in geometry_msgs::twist message
   */
  void publish(double linear, double angular)
  {
    int wait_count = 0;
    auto topic = velocity_publisher->get_topic_name();
    while (pub_node->count_subscribers(topic) == 0)
    {
      if (wait_count >= 5)
      {
        auto error_msg = std::string("publishing to ") + topic + " but no node subscribes to it";
        throw std::runtime_error(error_msg);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ++wait_count;
    }

    geometry_msgs::msg::TwistStamped velocity_message;
    velocity_message.header.stamp = pub_node->get_clock()->now();
    velocity_message.twist.linear.x = linear;
    velocity_message.twist.angular.z = angular;
    velocity_publisher->publish(velocity_message);
  }

  /// \brief wait for the subscriber and publisher to completely setup
  void waitForSetup()
  {
    constexpr std::chrono::seconds TIMEOUT{2};
    auto clock = pub_node->get_clock();
    auto start = clock->now();
    while (velocity_publisher->get_subscription_count() <= 0)
    {
      if ((clock->now() - start) > TIMEOUT)
      {
        FAIL();
      }
      rclcpp::spin_some(pub_node);
    }
  }

  void assignResources()
  {
    std::vector<LoanedStateInterface> state_ifs;
    state_ifs.emplace_back(steering_joint_pos_state_);
    state_ifs.emplace_back(traction_joint_vel_state_);

    std::vector<LoanedCommandInterface> command_ifs;
    command_ifs.emplace_back(steering_joint_pos_cmd_);
    command_ifs.emplace_back(traction_joint_vel_cmd_);

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  controller_interface::return_type InitController(
    const std::string traction_joint_name_init = traction_joint_name,
    const std::string steering_joint_name_init = steering_joint_name,
    const std::vector<rclcpp::Parameter> & parameters = {})
  {
    auto node_options = rclcpp::NodeOptions();
    std::vector<rclcpp::Parameter> parameter_overrides;

    parameter_overrides.push_back(
      rclcpp::Parameter("traction_joint_name", rclcpp::ParameterValue(traction_joint_name_init)));
    parameter_overrides.push_back(
      rclcpp::Parameter("steering_joint_name", rclcpp::ParameterValue(steering_joint_name_init)));
    // default parameters
    parameter_overrides.push_back(rclcpp::Parameter("wheelbase", rclcpp::ParameterValue(1.)));
    parameter_overrides.push_back(rclcpp::Parameter("wheel_radius", rclcpp::ParameterValue(0.1)));

    parameter_overrides.insert(parameter_overrides.end(), parameters.begin(), parameters.end());
    node_options.parameter_overrides(parameter_overrides);

    return controller_->init(controller_name, urdf_, 0, "", node_options);
  }

  const std::string controller_name = "test_tricycle_controller";
  std::unique_ptr<TestableTricycleController> controller_;

  double position_ = 0.1;
  double velocity_ = 0.2;

  hardware_interface::StateInterface steering_joint_pos_state_{
    steering_joint_name, HW_IF_POSITION, &position_};

  hardware_interface::StateInterface traction_joint_vel_state_{
    traction_joint_name, HW_IF_VELOCITY, &velocity_};

  hardware_interface::CommandInterface steering_joint_pos_cmd_{
    steering_joint_name, HW_IF_POSITION, &position_};

  hardware_interface::CommandInterface traction_joint_vel_cmd_{
    traction_joint_name, HW_IF_VELOCITY, &velocity_};

  rclcpp::Node::SharedPtr pub_node;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher;

  const std::string urdf_ = "";
};

TEST_F(TestTricycleController, init_fails_without_parameters)
{
  const auto ret =
    controller_->init(controller_name, urdf_, 0, "", controller_->define_custom_node_options());
  ASSERT_EQ(ret, controller_interface::return_type::ERROR);
}

TEST_F(TestTricycleController, init_fails_if_only_traction_or_steering_side_defined)
{
  ASSERT_EQ(
    InitController(traction_joint_name, std::string()), controller_interface::return_type::ERROR);

  ASSERT_EQ(
    InitController(std::string(), steering_joint_name), controller_interface::return_type::ERROR);
}

TEST_F(TestTricycleController, configure_succeeds_when_joints_are_specified)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // check interface configuration
  auto cmd_if_conf = controller_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, SizeIs(2lu));
  ASSERT_THAT(
    cmd_if_conf.names, UnorderedElementsAre(
                         std::string(traction_joint_name) + "/velocity",
                         std::string(steering_joint_name) + "/position"));
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  auto state_if_conf = controller_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(2lu));
  ASSERT_THAT(
    state_if_conf.names, UnorderedElementsAre(
                           std::string(traction_joint_name) + "/velocity",
                           std::string(steering_joint_name) + "/position"));
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
}

TEST_F(TestTricycleController, activate_fails_without_resources_assigned)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(TestTricycleController, activate_succeeds_with_resources_assigned)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  // We implicitly test that by default position feedback is required
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  assignResources();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(TestTricycleController, cleanup)
{
  ASSERT_EQ(
    InitController(
      traction_joint_name, steering_joint_name,
      {rclcpp::Parameter("wheelbase", 1.2), rclcpp::Parameter("wheel_radius", 0.12)}),
    controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  auto state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  assignResources();

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  waitForSetup();

  // send msg
  const double linear = 1.0;
  const double angular = 1.0;
  publish(linear, angular);
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  state = controller_->get_node()->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());

  // should be stopped
  EXPECT_EQ(0.0, steering_joint_pos_cmd_.get_value());
  EXPECT_EQ(0.0, traction_joint_vel_cmd_.get_value());

  state = controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());

  // should be stopped
  EXPECT_EQ(0.0, steering_joint_pos_cmd_.get_value());
  EXPECT_EQ(0.0, traction_joint_vel_cmd_.get_value());

  executor.cancel();
}

TEST_F(TestTricycleController, correct_initialization_using_parameters)
{
  ASSERT_EQ(
    InitController(
      traction_joint_name, steering_joint_name,
      {rclcpp::Parameter("wheelbase", 0.4), rclcpp::Parameter("wheel_radius", 1.0)}),
    controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  auto state = controller_->get_node()->configure();
  assignResources();

  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  EXPECT_EQ(position_, steering_joint_pos_cmd_.get_value());
  EXPECT_EQ(velocity_, traction_joint_vel_cmd_.get_value());

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  // send msg
  const double linear = 1.0;
  const double angular = 0.0;
  publish(linear, angular);
  // wait for msg is be published to the system
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_EQ(0.0, steering_joint_pos_cmd_.get_value());
  EXPECT_EQ(1.0, traction_joint_vel_cmd_.get_value());

  // deactivated
  // wait so controller process the second point when deactivated
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  EXPECT_EQ(0.0, steering_joint_pos_cmd_.get_value()) << "Wheels are halted on deactivate()";
  EXPECT_EQ(0.0, traction_joint_vel_cmd_.get_value()) << "Wheels are halted on deactivate()";

  // cleanup
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  EXPECT_EQ(0.0, steering_joint_pos_cmd_.get_value());
  EXPECT_EQ(0.0, traction_joint_vel_cmd_.get_value());

  state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}

namespace
{
rclcpp::Time ros_time(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}
constexpr double DT = 0.02;  // 50 Hz controller update
}  // namespace

class TestTricycleControllerSteering : public TestTricycleController
{
protected:
  // The base fixture feeds every command straight back as joint state. For the
  // churn tests the feedback must stay what a real robot reports while its
  // wheel is being churned: standing still. Decoupled state variables; a test
  // that wants the robot to "move" sets traction_state_ itself.
  double steering_state_ = 0.0;
  double traction_state_ = 0.0;
  hardware_interface::StateInterface decoupled_steering_state_{
    steering_joint_name, HW_IF_POSITION, &steering_state_};
  hardware_interface::StateInterface decoupled_traction_state_{
    traction_joint_name, HW_IF_VELOCITY, &traction_state_};

  void assignDecoupledResources()
  {
    std::vector<LoanedStateInterface> state_ifs;
    state_ifs.emplace_back(decoupled_steering_state_);
    state_ifs.emplace_back(decoupled_traction_state_);
    std::vector<LoanedCommandInterface> command_ifs;
    command_ifs.emplace_back(steering_joint_pos_cmd_);
    command_ifs.emplace_back(traction_joint_vel_cmd_);
    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  // Real-robot-like steering limits: fast normal steering, crawling
  // "stationary" limits that the churn filter falls back to.
  void init_and_activate(const std::vector<rclcpp::Parameter> & extra = {})
  {
    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("steering.max_position", 1.62),
      rclcpp::Parameter("steering.max_velocity", 1.0),
      rclcpp::Parameter("steering_low_speed.max_position", 1.62),
      rclcpp::Parameter("steering_low_speed.max_velocity", 1.0),
      rclcpp::Parameter("steering_stationary.max_position", 1.62),
      rclcpp::Parameter("steering_stationary.max_velocity", 0.2),
      rclcpp::Parameter("traction.max_acceleration", 5.0),
      rclcpp::Parameter("traction.max_deceleration", 8.0),
      rclcpp::Parameter("low_speed_threshold", 0.02),
      rclcpp::Parameter("stationary_time_threshold", 0.0),  // time-based fallback off
    };
    params.insert(params.end(), extra.begin(), extra.end());
    ASSERT_EQ(InitController(traction_joint_name, steering_joint_name, params),
              controller_interface::return_type::OK);
    ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    position_ = 0.0;
    velocity_ = 0.0;
    assignDecoupledResources();
    ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }

  /// One controller cycle at time t with the given twist.
  void step(double linear, double angular, double t)
  {
    controller_->set_command(linear, angular, ros_time(t));
    ASSERT_EQ(
      controller_->update(ros_time(t), rclcpp::Duration::from_seconds(DT)),
      controller_interface::return_type::OK);
  }

  double steering_cmd() const { return steering_joint_pos_cmd_.get_value(); }
  double traction_cmd() const { return traction_joint_vel_cmd_.get_value(); }
};

// wfal-amr4 2026-09-14: the spin's traction started while the wheel was still
// at 70 deg (target 90 deg) because the exact-mode threshold is 30 deg. The
// behavior tree passes a tight threshold in the set_exact_mode request for the
// spin; a request without threshold (0) uses the configured parameter.
TEST_F(TestTricycleControllerSteering, set_exact_mode_request_threshold_overrides_the_parameter)
{
  init_and_activate({rclcpp::Parameter("exact_mode_threshold", 0.5233)});
  double t = 0.0;

  // enabled without threshold: the parameter applies; spin demand (+90 deg),
  // wheel still at 70 deg, inside the 30 deg default
  EXPECT_TRUE(controller_->call_set_exact_mode(true).success);
  steering_state_ = 70.0 * M_PI / 180.0;
  for (int i = 0; i < 5; ++i, t += DT) step(0.0, 0.3, t);
  EXPECT_GT(traction_cmd(), 0.0) << "20 deg of steering error is inside the 30 deg threshold";

  // tightened by the request: 20 deg short of lock releases no traction any more
  EXPECT_TRUE(controller_->call_set_exact_mode(true, 0.05).success);
  for (int i = 0; i < 25; ++i, t += DT) step(0.0, 0.3, t);
  EXPECT_DOUBLE_EQ(traction_cmd(), 0.0);

  // wheel essentially at lock: traction is released
  steering_state_ = M_PI_2 - 0.02;
  for (int i = 0; i < 5; ++i, t += DT) step(0.0, 0.3, t);
  EXPECT_GT(traction_cmd(), 0.0);

  // out-of-range values are rejected and leave mode and threshold untouched
  for (const double invalid : {-0.1, 4.0, std::nan("")})
  {
    const auto res = controller_->call_set_exact_mode(true, invalid);
    EXPECT_FALSE(res.success);
    EXPECT_FALSE(res.message.empty());
  }
  steering_state_ = 70.0 * M_PI / 180.0;
  for (int i = 0; i < 25; ++i, t += DT) step(0.0, 0.3, t);
  EXPECT_DOUBLE_EQ(traction_cmd(), 0.0) << "still exact mode with 0.05 rad";

  // enabled again without threshold: back to the parameter, the 70 deg wheel drives
  EXPECT_TRUE(controller_->call_set_exact_mode(true).success);
  for (int i = 0; i < 5; ++i, t += DT) step(0.0, 0.3, t);
  EXPECT_GT(traction_cmd(), 0.0);

  // disabled: normal driving scales traction with the steering error instead of holding it
  EXPECT_TRUE(controller_->call_set_exact_mode(false).success);
  steering_state_ = 0.0;
  for (int i = 0; i < 5; ++i, t += DT) step(0.0, 0.3, t);
  EXPECT_GT(traction_cmd(), 0.0);
}

TEST_F(TestTricycleControllerSteering, near_zero_twist_holds_the_steering_instead_of_spinning_it)
{
  init_and_activate();
  double t = 0.0;

  // a real spin command steers towards +90 deg
  for (int i = 0; i < 10; ++i, t += DT) step(0.0, 0.5, t);
  const double after_spin = steering_cmd();
  EXPECT_GT(after_spin, 0.15);

  // (v=0, w=1e-4) used to be a "spin" and produced a -90 deg demand; now the
  // steering is held and the wheel does not drive
  for (int i = 0; i < 25; ++i, t += DT) step(0.0, -1e-4, t);
  EXPECT_NEAR(steering_cmd(), after_spin, 1e-6);  // float32 in AckermannDrive
  EXPECT_EQ(traction_cmd(), 0.0);

  // (v=1e-3, w=-1e-3) is also below the thresholds
  for (int i = 0; i < 25; ++i, t += DT) step(1e-3, -1e-3, t);
  EXPECT_NEAR(steering_cmd(), after_spin, 1e-6);  // float32 in AckermannDrive
  EXPECT_EQ(traction_cmd(), 0.0);

  // an exact zero twist still re-centres the wheel (unchanged behaviour)
  for (int i = 0; i < 25; ++i, t += DT) step(0.0, 0.0, t);
  EXPECT_LT(steering_cmd(), after_spin - 0.1);

  EXPECT_FALSE(controller_->churn_active());
}

namespace
{
// Tight creep arcs (wheelbase 1.0 m in the fixture): +-71.6 deg steering demand
constexpr double CREEP_V = 0.05;
constexpr double CREEP_W = 0.15;
const double CREEP_ALPHA = std::atan(CREEP_W * 1.0 / CREEP_V);
}  // namespace

TEST_F(TestTricycleControllerSteering, rapid_demand_reversals_while_stationary_are_debounced)
{
  init_and_activate();
  double t = 0.0;

  // creep arcs with the turning direction alternating at 10 Hz: the +-72 deg
  // steering demand flips every 0.1 s, faster than the wheel can follow.
  double max_abs_steering_before = 0.0;
  bool detected = false;
  double detected_at = -1.0;
  double frozen_at = 0.0;
  double max_abs_deviation_after = 0.0;
  double max_abs_traction_after = 0.0;
  for (int i = 0; i < 150; ++i, t += DT)
  {
    const double w = ((i / 5) % 2 == 0) ? CREEP_W : -CREEP_W;
    step(CREEP_V, w, t);
    if (!detected)
    {
      max_abs_steering_before = std::max(max_abs_steering_before, std::abs(steering_cmd()));
      if (controller_->churn_active())
      {
        detected = true;
        detected_at = t;
        frozen_at = steering_cmd();
      }
    }
    else
    {
      max_abs_deviation_after =
        std::max(max_abs_deviation_after, std::abs(steering_cmd() - frozen_at));
      // the traction limiter (8 rad/s^2 here) needs a few cycles to bring the
      // wheel speed that was commanded before detection down to zero
      if (t > detected_at + 0.1)
      {
        max_abs_traction_after = std::max(max_abs_traction_after, std::abs(traction_cmd()));
      }
    }
  }
  ASSERT_TRUE(detected) << "churn episode was not detected";
  // 4 reversals at 10 Hz -> well within the first second
  EXPECT_LT(detected_at, 1.0);
  // the wheel was actually being churned before detection
  EXPECT_GT(max_abs_steering_before, 0.05);
  // after detection the steering target is frozen and the wheel does not drive
  EXPECT_LT(max_abs_deviation_after, 0.05);
  EXPECT_EQ(max_abs_traction_after, 0.0);

  // upstream stops: exact zero -> re-centre demand, debounced and applied;
  // the episode ends hold_time after the evidence expired
  for (int i = 0; i < 500; ++i, t += DT) step(0.0, 0.0, t);
  EXPECT_FALSE(controller_->churn_active());
  EXPECT_NEAR(steering_cmd(), 0.0, 1e-6);

  // a steady spin request afterwards is NOT slowed down (normal limits, 1 rad/s)
  const double t0 = t;
  for (; t < t0 + 0.5; t += DT) step(0.0, 0.5, t);
  EXPECT_GT(steering_cmd(), 0.4);
}

TEST_F(TestTricycleControllerSteering, settled_demand_during_churn_episode_is_followed_at_low_speed_limits)
{
  init_and_activate();
  double t = 0.0;

  // provoke an episode (same pattern as above)
  for (int i = 0; i < 75; ++i, t += DT)
  {
    const double w = ((i / 5) % 2 == 0) ? CREEP_W : -CREEP_W;
    step(CREEP_V, w, t);
  }
  ASSERT_TRUE(controller_->churn_active());
  const double frozen_at = steering_cmd();
  EXPECT_LT(std::abs(frozen_at), 0.5);

  // upstream settles on one creep arc: after the 0.5 s debounce the wheel must
  // be moved to the demand with the low-speed limits (1 rad/s here), not the
  // stationary 0.2 rad/s, and the wheel must drive once it is there. Before
  // this the settled target was approached at the stationary limits and the
  // robot stood still for the whole episode.
  const double t0 = t;
  for (; t < t0 + 2.0; t += DT) step(CREEP_V, -CREEP_W, t);
  EXPECT_TRUE(controller_->churn_active());  // hold_time 5 s has not passed
  EXPECT_NEAR(steering_cmd(), -CREEP_ALPHA, 0.05);
  EXPECT_GT(std::abs(traction_cmd()), 0.1);
}

TEST_F(TestTricycleControllerSteering, spin_rule_flip_while_stopping_is_not_churn_evidence)
{
  init_and_activate();
  double t = 0.0;

  // Replay of a TEB stop (ros_domain_2_20260911_012047_0, t=22.9..23.5 s):
  // the last reverse arc, then teb_flickering_protection's turning-on-spot
  // state emits (v=0, w=0.11) for 0.35 s -> spin rule -> +90 deg, then a short
  // opposite arc and finally a steady forward creep arc. Counting the +-90 deg
  // jump as demand travel started a churn episode on every stop.
  for (int i = 0; i < 5; ++i, t += DT) step(-0.03, 0.11, t);   // -74.7 deg
  for (int i = 0; i < 18; ++i, t += DT) step(0.0, 0.11, t);    // spin rule: +90 deg
  for (int i = 0; i < 5; ++i, t += DT) step(-0.06, -0.09, t);  // +56.3 deg
  EXPECT_FALSE(controller_->churn_active());

  // steady creep arc: -64.4 deg; the wheel has to travel from +90 deg at the
  // 1 rad/s low-speed limit, so allow 3.5 s
  const double target = std::atan(-0.10 * 1.0 / 0.048);
  const double t0 = t;
  for (; t < t0 + 3.5; t += DT)
  {
    step(0.048, -0.10, t);
    EXPECT_FALSE(controller_->churn_active()) << "at t=" << t;
  }
  EXPECT_NEAR(steering_cmd(), target, 0.05);
  EXPECT_GT(traction_cmd(), 0.1);
}

TEST_F(TestTricycleControllerSteering, collision_monitor_stop_flicker_holds_the_steering)
{
  init_and_activate({rclcpp::Parameter("steering_hold.zero_hold_time", 5.0)});
  double t = 0.0;

  // Replay of amr47 2026-10-11 22:42:10-22:42:41 (rosbags_20261011_224156_6):
  // reversing on the close-putdown arc, the collision monitor STOPs on a wall
  // at the stop field edge and releases to LIMIT for one lidar scan (0.18 s)
  // whenever the field holds 3 instead of 4 points. Its output alternates
  // between (0, 0) and the limited arc; re-centring the wheel on every stop
  // swung the demand 0 <-> -1.1 rad and started a churn episode.
  const double arc_v = -0.019;
  const double arc_w = 0.037;
  const double arc_alpha = std::atan(arc_w * 1.0 / arc_v);
  const std::vector<std::pair<bool, double>> cm_output = {
    {true, 2.26}, {false, 2.28}, {true, 0.18}, {false, 0.52}, {true, 0.18}, {false, 4.06},
    {true, 0.18}, {false, 3.16}, {true, 0.18}, {false, 1.24}, {true, 0.18}, {false, 0.88},
    {true, 0.16}, {false, 1.78}, {true, 0.16}};
  double flicker_start = -1.0;
  for (const auto & [moving, duration] : cm_output)
  {
    const double t0 = t;
    for (; t < t0 + duration; t += DT)
    {
      step(moving ? arc_v : 0.0, moving ? arc_w : 0.0, t);
      if (flicker_start >= 0.0)
      {
        EXPECT_NEAR(steering_cmd(), arc_alpha, 0.05) << "at t=" << t;
      }
      EXPECT_FALSE(controller_->churn_active()) << "at t=" << t;
    }
    if (flicker_start < 0.0)
    {
      ASSERT_NEAR(steering_cmd(), arc_alpha, 0.05) << "wheel did not reach the arc";
      flicker_start = t;
    }
  }

  // the stop that followed lasted 7 s: after zero_hold_time the wheel re-centres
  const double t0 = t;
  for (; t < t0 + 7.0; t += DT) step(0.0, 0.0, t);
  EXPECT_NEAR(steering_cmd(), 0.0, 1e-6);
  EXPECT_FALSE(controller_->churn_active());
}

// wfal-amr2 2026-10-30 12:27:12: after the lift-front spin the wheel stood at
// -90 deg (zero_hold_time keeps it on a stop) and the door drive stood 3 s in
// exact mode until it was straight. set_steering turns it straight while the
// robot stands (marker check), without traction; the next drive releases it.
TEST_F(TestTricycleControllerSteering, set_steering_turns_the_standing_wheel_without_traction)
{
  init_and_activate({rclcpp::Parameter("steering_hold.zero_hold_time", 5.0)});
  double t = 0.0;
  for (const double t_end = t + 2.0; t < t_end; t += DT) step(0.0, -0.11, t);  // spin: -90 deg
  ASSERT_NEAR(steering_cmd(), -M_PI_2, 0.05);
  for (int i = 0; i < 5; ++i, t += DT) step(0.0, 0.0, t);  // stop: the wheel is held

  ASSERT_TRUE(controller_->call_set_steering(0.0).success);
  const double t0 = t;
  for (; t < t0 + 2.5; t += DT)  // 1.57 rad at 1 rad/s
  {
    step(0.0, 0.0, t);
    EXPECT_EQ(traction_cmd(), 0.0) << "at t=" << t;
  }
  EXPECT_NEAR(steering_cmd(), 0.0, 1e-3);
  EXPECT_FALSE(controller_->churn_active());

  // a spin command releases the target: the spin rule applies again
  for (const double t_end = t + 2.0; t < t_end; t += DT) step(0.0, 0.11, t);
  EXPECT_NEAR(steering_cmd(), M_PI_2, 0.05);

  EXPECT_FALSE(controller_->call_set_steering(3.0).success) << "beyond the steering limit";
}

// wfal lift front 2026-10-30: the spin started with the wheel at -40 deg (or
// retried in the other direction) and the spin rule swung it lock to lock
// (3-4.5 s in exact mode). With keep_wheel_side the left turn stays at -90 deg
// and reverses traction; without it the spin rule's +90 deg applies.
TEST_F(TestTricycleControllerSteering, keep_wheel_side_turns_on_the_spot_on_the_wheels_side)
{
  init_and_activate();
  double t = 0.0;
  for (const double t_end = t + 2.0; t < t_end; t += DT) step(0.0, -0.11, t);  // -90 deg
  ASSERT_NEAR(steering_cmd(), -M_PI_2, 0.05);

  ASSERT_TRUE(controller_->call_set_exact_mode(true, 0.08, true).success);
  steering_state_ = -M_PI_2;  // wheel at the lock: exact mode releases traction
  for (const double t_end = t + 0.5; t < t_end; t += DT) step(0.0, 0.11, t);  // turn left
  EXPECT_NEAR(steering_cmd(), -M_PI_2, 1e-6) << "swung the wheel to the other lock";
  EXPECT_LT(traction_cmd(), 0.0) << "traction must reverse for the left turn";
  EXPECT_FALSE(controller_->churn_active());

  ASSERT_TRUE(controller_->call_set_exact_mode(true, 0.08, false).success);
  for (const double t_end = t + 0.5; t < t_end; t += DT) step(0.0, 0.11, t);
  EXPECT_GT(steering_cmd(), -M_PI_2 + 0.3) << "spin rule: the wheel turns towards +90 deg";
}

TEST_F(TestTricycleControllerSteering, churn_monitor_ignores_demands_while_driving)
{
  init_and_activate();
  double t = 0.0;
  traction_state_ = 10.0;  // wheel feedback: 1 m/s with wheel_radius 0.1
  // driving forward at 1 m/s with the steering demand flipping +-0.3 rad at
  // 10 Hz: unpleasant, but the robot moves, so this is not stationary churn
  for (int i = 0; i < 150; ++i, t += DT)
  {
    const double w = ((i / 5) % 2 == 0) ? 0.3 : -0.3;
    step(1.0, w, t);
  }
  EXPECT_FALSE(controller_->churn_active());
}

TEST_F(TestTricycleControllerSteering, pivot_offset_defaults_leave_the_odometry_lateral_velocity_zero)
{
  init_and_activate();
  traction_state_ = 10.0;  // 1 m/s
  steering_state_ = 0.5;   // turning left
  double t = 0.0;
  for (int i = 0; i < 20; ++i, t += DT)
  {
    step(0.0, 0.0, t);
  }
  ASSERT_GT(controller_->odometry().getAngular(), 0.0);
  EXPECT_EQ(controller_->odometry().getLinearY(), 0.0);
  EXPECT_EQ(controller_->odometry().getPivotOffset(0.5), 0.0);
}

TEST_F(TestTricycleControllerSteering, pivot_offset_parameter_reaches_the_odometry)
{
  init_and_activate({rclcpp::Parameter("pivot_offset", 0.05)});
  traction_state_ = 10.0;  // 1 m/s
  steering_state_ = 0.5;   // turning left
  double t = 0.0;
  for (int i = 0; i < 20; ++i, t += DT)
  {
    step(0.0, 0.0, t);
  }
  const auto & odom = controller_->odometry();
  ASSERT_GT(odom.getAngular(), 0.0);
  EXPECT_NEAR(odom.getLinearY(), 0.05 * odom.getAngular(), 1e-12);
  // the robot turns about the pivot: arm wheelbase + c = 1.05
  EXPECT_NEAR(odom.getAngular(), 1.0 * std::sin(0.5) / 1.05, 1e-12);
}

TEST_F(TestTricycleControllerSteering, pivot_model_parameters_reach_the_odometry)
{
  init_and_activate({
    rclcpp::Parameter("pivot_offset", 0.01),
    rclcpp::Parameter("pivot_offset_full_steer", 0.03),
    rclcpp::Parameter("steering_deadband", 0.006),
  });
  const auto & odom = controller_->odometry();
  ASSERT_TRUE(odom.hasPivotModel());
  EXPECT_NEAR(odom.trueSteering(0.5), 0.494, 1e-15);
  EXPECT_NEAR(odom.trueSteering(-0.004), 0.0, 0.0);
  EXPECT_NEAR(odom.getPivotOffset(M_PI_2), 0.04, 1e-15);
  EXPECT_NEAR(odom.yawRateWheelbase(M_PI_2), 1.04, 1e-15);
  EXPECT_NEAR(odom.yawRateWheelbase(0.0), 1.01, 1e-15);
}

TEST_F(TestTricycleController, configure_fails_for_a_negative_steering_deadband)
{
  // rejected by the parameter validation (init) or at the latest by configure
  if (
    InitController(
      traction_joint_name, steering_joint_name,
      {rclcpp::Parameter("steering_deadband", -0.01)}) == controller_interface::return_type::OK)
  {
    EXPECT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  }
}

TEST_F(TestTricycleController, configure_fails_for_a_non_positive_pivot_arm)
{
  ASSERT_EQ(
    InitController(
      traction_joint_name, steering_joint_name, {rclcpp::Parameter("pivot_offset", -1.0)}),
    controller_interface::return_type::OK);
  EXPECT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

// --- inverse kinematics with the pivot model ---------------------------------------------------------

namespace
{
/// twist_to_ackermann before the pivot model (b44c73e / 037e0cb), copied verbatim.
std::tuple<double, double> legacy_twist_to_ackermann(
  double Vx, double theta_dot, double wheelbase, double wheel_radius)
{
  double alpha, Ws;
  if (Vx == 0 && theta_dot != 0)
  {  // is spin action
    alpha = theta_dot > 0 ? M_PI_2 : -M_PI_2;
    Ws = std::abs(theta_dot) * wheelbase / wheel_radius;
    return std::make_tuple(alpha, Ws);
  }
  alpha = (theta_dot == 0 || Vx == 0) ? 0 : std::atan(theta_dot * wheelbase / Vx);
  Ws = Vx / (wheel_radius * std::cos(alpha));
  return std::make_tuple(alpha, Ws);
}

uint64_t bits(double v)
{
  uint64_t b;
  std::memcpy(&b, &v, sizeof(b));
  return b;
}

std::vector<double> grid(double lo, double hi, int n)
{
  std::vector<double> out;
  for (int i = 0; i <= n; ++i)
  {
    out.push_back(lo + (hi - lo) * i / n);
  }
  return out;
}

constexpr double WB = 1.0;  // fixture wheelbase
constexpr double R = 0.1;   // fixture wheel_radius
}  // namespace

TEST_F(TestTricycleControllerSteering, default_inverse_kinematics_is_bit_identical_to_the_previous)
{
  init_and_activate();
  ASSERT_FALSE(controller_->odometry().hasPivotModel());
  auto vs = grid(-1.5, 1.5, 30);
  auto ws = grid(-1.0, 1.0, 20);
  vs.insert(vs.end(), {0.0, -0.0, 1e-9, -1e-9});
  ws.insert(ws.end(), {0.0, -0.0, 1e-9});
  for (double v : vs)
  {
    for (double w : ws)
    {
      const auto [alpha, Ws] = controller_->inverse(v, w);
      const auto [alpha_ref, Ws_ref] = legacy_twist_to_ackermann(v, w, WB, R);
      ASSERT_EQ(bits(alpha), bits(alpha_ref)) << v << " " << w;
      ASSERT_EQ(bits(Ws), bits(Ws_ref)) << v << " " << w;
    }
  }
}

TEST_F(TestTricycleControllerSteering, inverse_round_trip_with_pivot_model_in_all_quadrants)
{
  constexpr double c0 = 0.01, c1 = 0.04;
  init_and_activate({
    rclcpp::Parameter("pivot_offset", c0),
    rclcpp::Parameter("pivot_offset_full_steer", c1),
  });
  const auto & odom = controller_->odometry();
  int checked = 0;
  for (double v : grid(-1.5, 1.5, 60))
  {
    for (double w : grid(-1.0, 1.0, 40))
    {
      const auto [alpha, Ws] = controller_->inverse(v, w);
      ASSERT_LE(std::abs(alpha), M_PI_2) << v << " " << w;  // atan, also in reverse
      double vx, vy, wz;
      odom.wheelToTwist(Ws, alpha, vx, vy, wz);
      // one fixed-point step: residual ~ (c1 / 2L) * |delta1 - delta0|, < 5e-4 here
      EXPECT_NEAR(vx, v, 5e-4) << v << " " << w;
      EXPECT_NEAR(wz, w, 5e-4) << v << " " << w;
      // the driving direction follows v (reverse drives the wheel backwards)
      if (v != 0.0)
      {
        EXPECT_EQ(std::signbit(Ws), std::signbit(v)) << v << " " << w;
      }
      const double s = std::sin(alpha);
      EXPECT_NEAR(vy, (c0 + c1 * s * s) * wz, 1e-15);
      ++checked;
    }
  }
  EXPECT_EQ(checked, 61 * 41);
}

TEST_F(TestTricycleControllerSteering, inverse_with_constant_pivot_is_exact)
{
  // c1 = 0: L is constant, the fixed-point step is exact
  init_and_activate({rclcpp::Parameter("pivot_offset", 0.03)});
  const auto & odom = controller_->odometry();
  for (double v : grid(-1.5, 1.5, 12))
  {
    for (double w : grid(-1.0, 1.0, 8))
    {
      const auto [alpha, Ws] = controller_->inverse(v, w);
      double vx, vy, wz;
      odom.wheelToTwist(Ws, alpha, vx, vy, wz);
      EXPECT_NEAR(vx, v, 1e-12);
      EXPECT_NEAR(wz, w, 1e-12);
      // same steering as the classic model with wheelbase + c
      if (v != 0.0)
      {
        EXPECT_NEAR(alpha, std::get<0>(legacy_twist_to_ackermann(v, w, WB + 0.03, R)), 1e-12);
      }
    }
  }
}

TEST_F(TestTricycleControllerSteering, inverse_spin_drives_the_arm_at_full_steer)
{
  constexpr double c0 = 0.01, c1 = 0.04;
  init_and_activate({
    rclcpp::Parameter("pivot_offset", c0),
    rclcpp::Parameter("pivot_offset_full_steer", c1),
  });
  const auto & odom = controller_->odometry();
  for (double w : {0.3, -0.3, 1e-3})
  {
    const auto [alpha, Ws] = controller_->inverse(0.0, w);
    EXPECT_EQ(alpha, std::copysign(M_PI_2, w));
    EXPECT_NEAR(Ws, std::abs(w) * (WB + c0 + c1) / R, 1e-12);
    double vx, vy, wz;
    odom.wheelToTwist(Ws, alpha, vx, vy, wz);
    EXPECT_NEAR(wz, w, 1e-12);
    EXPECT_NEAR(vx, 0.0, 1e-15);
    // keep_wheel_side's flip (wheel on the other lock, traction reversed) is the same motion
    odom.wheelToTwist(-Ws, -alpha, vx, vy, wz);
    EXPECT_NEAR(wz, w, 1e-12);
    EXPECT_NEAR(vy, (c0 + c1) * w, 1e-12);
  }
  const auto [alpha0, Ws0] = controller_->inverse(0.0, 0.0);
  EXPECT_EQ(alpha0, 0.0);
  EXPECT_EQ(Ws0, 0.0);
}

TEST_F(TestTricycleControllerSteering, inverse_is_continuous_as_v_goes_to_zero)
{
  constexpr double c0 = 0.01, c1 = 0.04;
  init_and_activate({
    rclcpp::Parameter("pivot_offset", c0),
    rclcpp::Parameter("pivot_offset_full_steer", c1),
  });
  const auto & odom = controller_->odometry();
  for (double w : {0.3, -0.3})
  {
    const auto [alpha_spin, Ws_spin] = controller_->inverse(0.0, w);
    for (double eps : {1e-3, 1e-6, 1e-9})
    {
      // v -> 0+: converges to the spin command
      const auto [alpha_p, Ws_p] = controller_->inverse(eps, w);
      EXPECT_NEAR(alpha_p, alpha_spin, 2.0 * eps / (std::abs(w) * WB)) << eps;
      EXPECT_NEAR(Ws_p, Ws_spin, 1e-3 * Ws_spin + eps / R) << eps;
      // v -> 0-: converges to the keep_wheel_side form of the spin (other lock, reversed)
      const auto [alpha_m, Ws_m] = controller_->inverse(-eps, w);
      EXPECT_NEAR(alpha_m, -alpha_spin, 2.0 * eps / (std::abs(w) * WB)) << eps;
      EXPECT_NEAR(Ws_m, -Ws_spin, 1e-3 * Ws_spin + eps / R) << eps;
      // both are the same motion
      double vx_p, vy_p, w_p, vx_m, vy_m, w_m;
      odom.wheelToTwist(Ws_p, alpha_p, vx_p, vy_p, w_p);
      odom.wheelToTwist(Ws_m, alpha_m, vx_m, vy_m, w_m);
      EXPECT_NEAR(w_p, w, 5e-4);
      EXPECT_NEAR(w_m, w, 5e-4);
      EXPECT_NEAR(vx_p, eps, 1e-9);
      EXPECT_NEAR(vx_m, -eps, 1e-9);
    }
  }
}

TEST_F(TestTricycleControllerSteering, inverse_does_not_compensate_the_steering_deadband)
{
  // A deadband alone switches to the pivot code path but must not change the command: +-b at
  // delta ~ 0 would make the steering jump by 2b on every sign change (chatter).
  init_and_activate({rclcpp::Parameter("steering_deadband", 0.006)});
  ASSERT_TRUE(controller_->odometry().hasPivotModel());
  for (double v : grid(-1.5, 1.5, 12))
  {
    for (double w : grid(-1.0, 1.0, 8))
    {
      const auto [alpha, Ws] = controller_->inverse(v, w);
      const auto [alpha_ref, Ws_ref] = legacy_twist_to_ackermann(v, w, WB, R);
      EXPECT_NEAR(alpha, alpha_ref, 1e-12) << v << " " << w;
      EXPECT_NEAR(Ws, Ws_ref, 1e-12) << v << " " << w;
    }
  }
  // tiny heading corrections give tiny steering angles of either sign, no jump to +-b
  EXPECT_NEAR(std::get<0>(controller_->inverse(1.0, 1e-4)), 1e-4, 1e-9);
  EXPECT_NEAR(std::get<0>(controller_->inverse(1.0, -1e-4)), -1e-4, 1e-9);
}

TEST_F(TestTricycleControllerSteering, keep_wheel_side_spin_with_pivot_model)
{
  constexpr double c0 = 0.01, c1 = 0.04;
  init_and_activate({
    rclcpp::Parameter("pivot_offset", c0),
    rclcpp::Parameter("pivot_offset_full_steer", c1),
  });
  double t = 0.0;
  for (const double t_end = t + 2.0; t < t_end; t += DT) step(0.0, -0.11, t);  // -90 deg
  ASSERT_NEAR(steering_cmd(), -M_PI_2, 0.05);

  ASSERT_TRUE(controller_->call_set_exact_mode(true, 0.08, true).success);
  steering_state_ = -M_PI_2;  // wheel at the lock: exact mode releases traction
  for (const double t_end = t + 1.0; t < t_end; t += DT) step(0.0, 0.11, t);  // turn left
  EXPECT_NEAR(steering_cmd(), -M_PI_2, 1e-6) << "swung the wheel to the other lock";
  // reversed traction driving the arm L(pi/2) = wheelbase + c0 + c1
  EXPECT_NEAR(traction_cmd(), -0.11 * (WB + c0 + c1) / R, 1e-6);
}

TEST_F(TestTricycleControllerSteering, open_loop_odometry_uses_the_pivot_model)
{
  constexpr double c0 = 0.0, c1 = 0.04, b = 0.006;
  init_and_activate({
    rclcpp::Parameter("open_loop", true),
    rclcpp::Parameter("pivot_offset", c0),
    rclcpp::Parameter("pivot_offset_full_steer", c1),
    rclcpp::Parameter("steering_deadband", b),
  });
  double t = 0.0;
  for (const double t_end = t + 3.0; t < t_end; t += DT) step(0.5, 0.4, t);
  const auto & odom = controller_->odometry();
  double vx, vy, wz;
  odom.wheelToTwist(traction_cmd(), steering_cmd(), vx, vy, wz);
  EXPECT_NEAR(odom.getAngular(), wz, 1e-9);
  EXPECT_NEAR(odom.getLinear(), vx, 1e-9);
  EXPECT_NEAR(odom.getLinearY(), vy, 1e-9);
  const double delta_t = steering_cmd() - b;
  EXPECT_NEAR(vy, c1 * std::sin(delta_t) * std::sin(delta_t) * wz, 1e-12);
  // the uncompensated deadband: the wheel's true angle is the command minus b
  const double v_d = traction_cmd() * R;
  const double s = std::sin(delta_t);
  EXPECT_NEAR(wz, v_d * s / (WB + c1 * s * s), 1e-12);
  const double s_cmd = std::sin(steering_cmd());
  EXPECT_LT(wz, v_d * s_cmd / (WB + c1 * s_cmd * s_cmd));
}
