// Copyright (c) 2021 ros2_control Development Team
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

#ifndef JOINT_TRAJECTORY_CONTROLLER__JOINT_TRAJECTORY_CONTROLLER_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__JOINT_TRAJECTORY_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/single_joint_position.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "control_toolbox/pid.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "velocity_controllers/visibility_control.h"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/types.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_server_goal_handle.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace rclcpp_action
{
template <typename ActionT>
class ServerGoalHandle;
}  // namespace rclcpp_action
namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace joint_position_controller
{
class Trajectory;

class JointPositionController : public controller_interface::ControllerInterface
{
public:
  VELOCITY_CONTROLLERS_PUBLIC
  JointPositionController();

  /**
   * @brief command_interface_configuration This controller requires the position command
   * interfaces for the controlled joints
   */
  VELOCITY_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief command_interface_configuration This controller requires the position and velocity
   * state interfaces for the controlled joints
   */
  VELOCITY_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  VELOCITY_CONTROLLERS_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  VELOCITY_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  VELOCITY_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  VELOCITY_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // VELOCITY_CONTROLLERS_PUBLIC
  // controller_interface::CallbackReturn on_deactivate(
  //   const rclcpp_lifecycle::State & previous_state) override;

  // VELOCITY_CONTROLLERS_PUBLIC
  // controller_interface::CallbackReturn on_cleanup(
  //   const rclcpp_lifecycle::State & previous_state) override;

  // VELOCITY_CONTROLLERS_PUBLIC
  // controller_interface::CallbackReturn on_error(
  //   const rclcpp_lifecycle::State & previous_state) override;

  // VELOCITY_CONTROLLERS_PUBLIC
  // controller_interface::CallbackReturn on_shutdown(
  //   const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::string joint_name_;

  // Preallocate variables used in the realtime update() function
  trajectory_msgs::msg::JointTrajectoryPoint state_current_;
  trajectory_msgs::msg::JointTrajectoryPoint state_desired_;
  trajectory_msgs::msg::JointTrajectoryPoint state_error_;

  // Parameters for some special cases, e.g. hydraulics powered robots
  // Run the controller in open-loop, i.e., read hardware states only when starting controller.
  // This is useful when robot is not exactly following the commanded trajectory.
  bool open_loop_control_ = false;

  double state_publish_rate_;
  double action_monitor_rate_;


  struct JointHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command;
  };

  std::vector<JointHandle> joint_;
  controller_interface::CallbackReturn get_joint(
    const std::string & joint_name, std::vector<JointHandle> & joint);

  using ControllerStateMsg = control_msgs::msg::JointTrajectoryControllerState;
  using StatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
  using StatePublisherPtr = std::unique_ptr<StatePublisher>;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr publisher_;
  StatePublisherPtr state_publisher_;

  rclcpp::Duration state_publisher_period_ = rclcpp::Duration(20ms);
  rclcpp::Time last_state_publish_time_;

  using SingleJointPositionAction = control_msgs::action::SingleJointPosition;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<SingleJointPositionAction>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr>;

  rclcpp_action::Server<SingleJointPositionAction>::SharedPtr action_server_;
  RealtimeGoalHandleBuffer rt_active_goal_;  ///< Currently active action goal, if any.
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
  rclcpp::Duration action_monitor_period_ = rclcpp::Duration(50ms);

  // callbacks for action_server_
  VELOCITY_CONTROLLERS_PUBLIC
  rclcpp_action::GoalResponse goal_received_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SingleJointPositionAction::Goal> goal);
  VELOCITY_CONTROLLERS_PUBLIC
  rclcpp_action::CancelResponse goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SingleJointPositionAction>> goal_handle);
  VELOCITY_CONTROLLERS_PUBLIC
  void goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<SingleJointPositionAction>> goal_handle);

  // VELOCITY_CONTROLLERS_PUBLIC
  // void preempt_active_goal();

  VELOCITY_CONTROLLERS_PUBLIC
  bool reset();

  using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
  VELOCITY_CONTROLLERS_PUBLIC
  void publish_state(
    const JointTrajectoryPoint & desired_state, const JointTrajectoryPoint & current_state,
    const JointTrajectoryPoint & state_error);

//private:

};

}  // namespace joint_position_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__JOINT_TRAJECTORY_CONTROLLER_HPP_
