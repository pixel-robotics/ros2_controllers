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

#include "velocity_controllers/joint_position_controller.hpp"

#include <stddef.h>
#include <chrono>
#include <functional>
#include <memory>
#include <ostream>
#include <ratio>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/header.hpp"

namespace joint_position_controller
{

using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;


JointPositionController::JointPositionController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn JointPositionController::on_init()
{
  try
  {
    // with the lifecycle node being initialized, we can declare parameters
    joint_name_ = auto_declare<std::string>("joint", joint_name_);
    open_loop_control_ = auto_declare<bool>("open_loop_control", open_loop_control_);
    state_publish_rate_ = auto_declare<double>("state_publish_rate", 50.0);
    action_monitor_rate_ = auto_declare<double>("action_monitor_rate", 20.0);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.push_back(joint_name_ + "/" + HW_IF_VELOCITY);
  return conf;
}


controller_interface::InterfaceConfiguration
JointPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.push_back(joint_name_ + "/" + HW_IF_POSITION);
  return conf;
}

controller_interface::return_type JointPositionController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  return controller_interface::return_type::OK;
}


controller_interface::CallbackReturn JointPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  const auto logger = get_node()->get_logger();

  // update parameters
  joint_name_ = get_node()->get_parameter("joint").as_string();
  if (joint_name_.empty())
  {
    RCLCPP_ERROR(logger, "'joint_name' parameter was empty");
    return CallbackReturn::ERROR;
  }
  // Read parameters customizing controller for special cases
  open_loop_control_ = get_node()->get_parameter("open_loop_control").get_value<bool>();

  // State publisher
  state_publish_rate_ = get_node()->get_parameter("state_publish_rate").get_value<double>();
  RCLCPP_INFO(logger, "Controller state will be published at %.2f Hz.", state_publish_rate_);
  if (state_publish_rate_ > 0.0)
  {
    state_publisher_period_ = rclcpp::Duration::from_seconds(1.0 / state_publish_rate_);
  }
  else
  {
    state_publisher_period_ = rclcpp::Duration::from_seconds(0.0);
  }

  publisher_ =
    get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<StatePublisher>(publisher_);

//   state_publisher_->lock();
//   state_publisher_->msg_.joint_names = command_joint_name_;
//   state_publisher_->msg_.desired.positions.resize(dof_);
//   state_publisher_->msg_.desired.velocities.resize(dof_);
//   state_publisher_->msg_.desired.accelerations.resize(dof_);
//   state_publisher_->msg_.actual.positions.resize(dof_);
//   state_publisher_->msg_.error.positions.resize(dof_);
//   state_publisher_->msg_.actual.velocities.resize(dof_);
//   state_publisher_->msg_.error.velocities.resize(dof_);    
//   state_publisher_->msg_.actual.accelerations.resize(dof_);
//   state_publisher_->msg_.error.accelerations.resize(dof_);

//   state_publisher_->unlock();

  last_state_publish_time_ = get_node()->now();

  // action server configuration

  action_monitor_rate_ = get_node()->get_parameter("action_monitor_rate").get_value<double>();
  RCLCPP_INFO(logger, "Action status changes will be monitored at %.2f Hz.", action_monitor_rate_);
  action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / action_monitor_rate_);

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<SingleJointPositionAction>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/follow_joint_trajectory",
    std::bind(&JointPositionController::goal_received_callback, this, _1, _2),
    std::bind(&JointPositionController::goal_cancelled_callback, this, _1),
    std::bind(&JointPositionController::goal_accepted_callback, this, _1));

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPositionController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (get_joint(joint_name_, joint_) == CallbackReturn::ERROR)
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// controller_interface::CallbackReturn JointPositionController::on_deactivate(
//   const rclcpp_lifecycle::State &)
// {
//   // TODO(anyone): How to halt when using effort commands?
//   for (size_t index = 0; index < dof_; ++index)
//   {
//     if (has_position_command_interface_)
//     {
//       joint_command_interface_[0][index].get().set_value(
//         joint_command_interface_[0][index].get().get_value());
//     }

//     if (has_velocity_command_interface_)
//     {
//       joint_command_interface_[1][index].get().set_value(0.0);
//     }

//     if (has_effort_command_interface_)
//     {
//       joint_command_interface_[3][index].get().set_value(0.0);
//     }
//   }

//   for (size_t index = 0; index < allowed_interface_types_.size(); ++index)
//   {
//     joint_command_interface_[index].clear();
//     joint_state_interface_[index].clear();
//   }
//   release_interfaces();

//   return CallbackReturn::SUCCESS;
// }

// controller_interface::CallbackReturn JointPositionController::on_cleanup(
//   const rclcpp_lifecycle::State &)
// {
//   // go home
//   traj_home_point_ptr_->update(traj_msg_home_ptr_);
//   traj_point_active_ptr_ = &traj_home_point_ptr_;

//   return CallbackReturn::SUCCESS;
// }

// controller_interface::CallbackReturn JointPositionController::on_error(
//   const rclcpp_lifecycle::State &)
// {
//   if (!reset())
//   {
//     return CallbackReturn::ERROR;
//   }
//   return CallbackReturn::SUCCESS;
// }

// bool JointPositionController::reset()
// {
//   for (const auto & pid : pids_)
//   {
//     pid->reset();
//   }

//   // iterator has no default value
//   // prev_traj_point_ptr_;
//   traj_point_active_ptr_ = nullptr;
//   traj_external_point_ptr_.reset();
//   traj_home_point_ptr_.reset();
//   traj_msg_home_ptr_.reset();

//   // reset pids
//   for (const auto & pid : pids_)
//   {
//     pid->reset();
//   }

//   return true;
// }

// controller_interface::CallbackReturn JointPositionController::on_shutdown(
//   const rclcpp_lifecycle::State &)
// {
//   // TODO(karsten1987): what to do?

//   return CallbackReturn::SUCCESS;
// }

void JointPositionController::publish_state(
  const JointTrajectoryPoint & desired_state, const JointTrajectoryPoint & current_state,
  const JointTrajectoryPoint & state_error)
{
  if (state_publisher_period_.seconds() <= 0.0)
  {
    return;
  }

  if (get_node()->now() < (last_state_publish_time_ + state_publisher_period_))
  {
    return;
  }

  if (state_publisher_ && state_publisher_->trylock())
  {
    last_state_publish_time_ = get_node()->now();
    state_publisher_->msg_.header.stamp = last_state_publish_time_;
    state_publisher_->msg_.desired.positions = desired_state.positions;
    state_publisher_->msg_.desired.velocities = desired_state.velocities;
    state_publisher_->msg_.desired.accelerations = desired_state.accelerations;
    state_publisher_->msg_.actual.positions = current_state.positions;
    state_publisher_->msg_.error.positions = state_error.positions;
    {
      state_publisher_->msg_.actual.accelerations = current_state.accelerations;
      state_publisher_->msg_.error.accelerations = state_error.accelerations;
    }

    state_publisher_->unlockAndPublish();
  }
}

rclcpp_action::GoalResponse JointPositionController::goal_received_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const SingleJointPositionAction::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received new action goal");

  // Precondition: Running controller
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Can't accept new action goals. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointPositionController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<SingleJointPositionAction>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

  // Check that cancel request refers to currently active goal (if any)
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle)
  {
    // Controller uptime
    // Enter hold current position mode
    // set_hold_position();

    RCLCPP_DEBUG(
      get_node()->get_logger(), "Canceling active action goal because cancel callback received.");

    // Mark the current goal as canceled
    auto action_res = std::make_shared<SingleJointPositionAction::Result>();
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointPositionController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<SingleJointPositionAction>> goal_handle)
{
  // Update new trajectory
  {
    // preempt_active_goal();
    auto traj_msg =
      std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal_handle->get_goal()->trajectory);

    //add_new_trajectory_msg(traj_msg);
  }

  // Update the active goal
  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
//   rt_goal->preallocated_feedback_->joint_names = joint_name_;
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  // Setup goal status checking timer
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::seconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}


controller_interface::CallbackReturn JointPositionController::get_joint(
  const std::string & joint_name, std::vector<JointHandle> & joint)
{
  RCLCPP_INFO(get_node()->get_logger(), "Get Wheel Joint Instance");

  // Lookup the position state interface
  const auto state_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(),
    [&joint_name](const auto & interface)
    {
      return interface.get_prefix_name() == joint_name &&
             interface.get_interface_name() == HW_IF_POSITION;
    });
  if (state_handle == state_interfaces_.cend())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to obtain joint state handle for %s",
      joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  // Lookup the velocity command interface
  const auto command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&joint_name](const auto & interface)
    {
      return interface.get_prefix_name() == joint_name &&
             interface.get_interface_name() == HW_IF_VELOCITY;
    });
  if (command_handle == command_interfaces_.end())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to obtain joint command handle for %s",
      joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  // Create the traction joint instance
  joint.emplace_back(JointHandle{std::ref(*state_handle), std::ref(*command_handle)});
  return CallbackReturn::SUCCESS;
}

}  // namespace joint_position_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_position_controller::JointPositionController, controller_interface::ControllerInterface)
