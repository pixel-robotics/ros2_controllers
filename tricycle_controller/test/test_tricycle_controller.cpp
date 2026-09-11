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
#include <memory>
#include <string>
#include <thread>
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
