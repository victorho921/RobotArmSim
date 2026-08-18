// Copyright 2026, victorho921
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

#include "include/robot_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
// #include "rclcpp/logging.hpp"

// #include "rclcpp/qos.hpp"
// #include "rclcpp/time.hpp"
// #include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
// #include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace Custom_Franka_Controller
{
HybridFTController::HybridFTController() : controller_interface::ControllerInterface() {}


// Read out joint name, command and state interface
controller_interface::CallbackReturn HybridFTController::on_init()
{
  // should have error handling
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ =
    auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ =
    auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  // Set the initial point of trajectory to be 0 (both joint position and joint velocity)
  point_interp_.positions.assign(joint_names_.size(), 0);
  point_interp_.velocities.assign(joint_names_.size(), 0);

  // Set default virtual stiffness and damping gains for end effector
  const double default_kp = 10.0;  // N/m
  const double default_kd = 2.0;   // N·s/m
  auto_declare<double>("stiffness.trans_x", default_kp);
  auto_declare<double>("stiffness.trans_y", default_kp);
  auto_declare<double>("stiffness.trans_z", default_kp);
  auto_declare<double>("stiffness.rot_x", default_kp);
  auto_declare<double>("stiffness.rot_y", default_kp);
  auto_declare<double>("stiffness.rot_z", default_kp);
  return CallbackReturn::SUCCESS;
}


// Both command & state interface method is to declare variable (similar) of every joint with command or state
// e.g: joint1/command, joint2/state
controller_interface::InterfaceConfiguration HybridFTController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : command_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration HybridFTController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

// defining a callback 
controller_interface::CallbackReturn HybridFTController::on_configure(const rclcpp_lifecycle::State &)
{
  // Get list of joints from parameters
  auto joints = get_node()->get_parameter("joints").as_string_array();
  num_joints_ = joints.size();

  q_.resize(num_joints_);
  dq_.resize(num_joints_);
  vel_cmd.resize(num_joints_, 0.0);

  // Initialize control gains and positions
  kp_.resize(num_joints_, 1.0);
  kd_.resize(num_joints_, 0.1);
  kp_hold_.resize(num_joints_, 200.0);
  target_pos_.resize(num_joints_, 0.0);
  hold_pos_.resize(num_joints_, 0.0);

  // Initialize sensor vectors
  forces_.resize(3, 0.0);
  torques_.resize(3, 0.0);

  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

  // create subscription to the trajectory node
  // pass the message from the subscription to the control loop

  auto callback =
    [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) -> void
  {
    traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
    new_msg_ = true;
  };
  joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);


  auto wrench_callback = 
    [this](const std::shared_ptr<geometry_msgs::msg::WrenchStamped> wrench) -> void
  {
    wrench_external_point_ptr_.writeFromNonRT(wrench);
    new_wrench_ = true;
  };
  ft_sensor_subscriber_ = 
    get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/force_torque_joint6", rclcpp::SystemDefaultsQoS(), wrench_callback);
  
  // Create a name to index map
  joint_name_to_index_.clear();
  for (size_t i = 0; i < joints.size(); ++i)
      joint_name_to_index_[joints[i]] = i;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HybridFTController::on_activate(const rclcpp_lifecycle::State &)
{
  // clear out vectors in case of restart
  joint_velocity_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

  // Assign command interfaces
  for (auto & interface : command_interfaces_)
    {
        const std::string interface_type = interface.get_interface_name();  // "position", "velocity", etc.
        auto it = command_interface_map_.find(interface_type);
        if (it != command_interface_map_.end())
        {
            it->second->push_back(interface);   // push into the correct vector
        }
    }
  // Assign state interfaces
  for (auto & interface : state_interfaces_)
  {
      const std::string interface_name = interface.get_name();
      const std::string interface_type = interface.get_interface_name();  // position/velocity/effort
      auto it = state_interface_map_.find(interface_type);
      // state_interfaces_[interface.get_interface_name()] = std::ref(interface);
      if (it != state_interface_map_.end())
      {
          it->second->push_back(interface);   // push into the correct vector
      }
  }

  // Safty check for the interfaces
  // if (joint_position_command_interface_.empty() && joint_position_state_interface_.empty())
  // {
  //     RCLCPP_ERROR(get_node()->get_logger(), "No position interfaces found!");
  //     return CallbackReturn::ERROR;
  // }

  // Hold current position at the start of the controller
  for (size_t i = 0; i < joint_position_state_interface_.size(); ++i)
  {
    joint_velocity_command_interface_[i].get().set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(), "Controller activated — holding startup position");

  return CallbackReturn::SUCCESS;
}
// Linear interpolation for trajectory following
void HybridFTController::linear_interpolate_point(
  const trajectory_msgs::msg::JointTrajectoryPoint & point_1,
  const trajectory_msgs::msg::JointTrajectoryPoint & point_2,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp, 
  double tau)
{
  size_t num_joints = point_1.positions.size();

  for (size_t i = 0; i < num_joints; i++)
  {
    point_interp.positions[i] = tau * point_1.positions[i] + (1.0 - tau) * point_2.positions[i];
  }
  for (size_t i = 0; i < num_joints; i++)
  {
    point_interp.velocities[i] = tau * point_1.velocities[i] + (1.0 - tau) * point_2.velocities[i];
  }
}

// Cubic Interpolation for 
void HybridFTController::cubic_interpolate_point(
  const trajectory_msgs::msg::JointTrajectoryPoint & point_1,
  const trajectory_msgs::msg::JointTrajectoryPoint & point_2,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp,
  double T,
  double tau) // The time duration of this specific segment in seconds
{
  size_t num_joints = point_1.positions.size();
  
  for (size_t i = 0; i < num_joints; ++i)
  {
    double q0 = point_1.positions[i];
    double q1 = point_2.positions[i];
    
    // Fallback protection: if velocities aren't provided in the message, assume 0.0
    double v0 = (point_1.velocities.size() > i) ? point_1.velocities[i] : 0.0;
    double v1 = (point_2.velocities.size() > i) ? point_2.velocities[i] : 0.0;

    // Cubic Spline Blend Coefficients
    double a0 = q0;
    double a1 = v0;
    double a2 = (3.0 * (q1 - q0) / (T * T)) - ((2.0 * v0 + v1) / T);
    double a3 = (-2.0 * (q1 - q0) / (T * T * T)) + ((v0 + v1) / (T * T));

    point_interp.positions[i] = a0 + a1 * tau + a2 * tau * tau + a3 * tau * tau * tau;
    point_interp.velocities[i] = a1 + 2.0 * a2 * tau + 3.0 * a3 * tau * tau;
    // RCLCPP_INFO(get_node()->get_logger(), "Cubic interpolation for joint %zu: pos=%.2f, vel=%.2f", i, point_interp.positions[i], point_interp.velocities[i]);
  }
}

void HybridFTController::interpolate_trajectory_point(
  const trajectory_msgs::msg::JointTrajectory & traj_msg, const rclcpp::Duration & cur_time, 
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp)
{
  // Varibale explained:
  // cur_time is the time elapsed since the start of the trajectory execution
  // tau is the local time within each waypoint

  size_t traj_len = traj_msg.points.size();
  auto last_time = traj_msg.points[traj_len - 1].time_from_start;
  double total_time = last_time.sec + last_time.nanosec * 1E-9;

  // Get the duration from the trajectory message (if provided) or default to 1 second
  double T = (traj_msg.points[0].time_from_start.sec + traj_msg.points[0].time_from_start.nanosec * 1E-9);
  if (T <= 0.0) {
    RCLCPP_WARN(rclcpp::get_logger("HybridFTController"), "Trajectory point has non-positive time_from_start. Defaulting to 1 second.");
    T = 1.0; 
  }

  // One point Trajectory
  if (traj_len == 1)  
  {
    // RCLCPP_INFO(rclcpp::get_logger("HybridFTController"), "ONE POINT");
    // Get the current point and velocity from state interface
    trajectory_msgs::msg::JointTrajectoryPoint current_point;
    for (size_t i = 0; i < num_joints_; ++i)
    {
      current_point.positions.push_back(joint_position_state_interface_[i].get().get_value());
      current_point.velocities.push_back(joint_velocity_state_interface_[i].get().get_value());
    }
    double tau = std::min(cur_time.seconds()/T, 1.0); 
    // cubic_interpolate_point(current_point, traj_msg.points[0], point_interp, T, tau);
    // linear_interpolate_point(current_point, traj_msg.points[0], point_interp, tau);
    point_interp = traj_msg.points[0];
    return;
  }
  // Multiple point Trajectory
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("HybridFTController"), "MULTIPLE POINT");
    // the section number of the current point in the trajectory after it being sliced
    size_t ind = cur_time.seconds() * (traj_len / total_time); 
    // ind = std::min(static_cast<double>(ind), traj_len - 2);
    ind = std::min(ind, traj_len - 2);
    double tau = cur_time.seconds() - ind * (total_time / traj_len);
    // linear_interpolate_point(traj_msg.points[ind], traj_msg.points[ind + 1], point_interp, tau);
    cubic_interpolate_point(traj_msg.points[ind], traj_msg.points[ind + 1], point_interp, T , tau );
  }
}

void HybridFTController::PDControl()
{
  for (size_t i = 0; i < num_joints_; ++i) {
    double q_des = point_interp_.positions[i];
    double dq_des = point_interp_.velocities[i];

    double q_curr = joint_position_state_interface_[i].get().get_value();
    double dq_curr = joint_velocity_state_interface_[i].get().get_value();

    double pos_err = q_des - q_curr;
    double vel_err = dq_des - dq_curr;

    vel_cmd[i] = kp_[i] * pos_err + kd_[i] * vel_err;
  }
}

controller_interface::return_type HybridFTController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (emergency_stop_)
    {
        for (size_t i = 0; i < num_joints_; ++i)
        {
            joint_velocity_command_interface_[i].get().set_value(0.0);
        }
        return controller_interface::return_type::OK;
    }

  // Filter the z-force data
  wrench_ = *wrench_external_point_ptr_.readFromRT();
  if (wrench_ == nullptr) {
    triggerEStop("Wrench message is null");
    return controller_interface::return_type::OK;
  }
  
  // if (new_wrench_)
  // {
      // forces_[0] = wrench_->wrench.force.x;
      // forces_[1] = wrench_->wrench.force.y;
      // forces_[2] = wrench_->wrench.force.z;

      // torques_[0] = wrench_->wrench.torque.x;
      // torques_[1] = wrench_->wrench.torque.y;
      // torques_[2] = wrench_->wrench.torque.z;

  //     RCLCPP_INFO(get_node()->get_logger(), "Latest wrench - Force: [%.2f, %.2f, %.2f], Torque: [%.2f, %.2f, %.2f]", 
  //                  forces_[0], forces_[1], forces_[2], torques_[0], torques_[1], torques_[2]);
  // // }
  double z_force_ = wrench_->wrench.force.z;
   filtered_force_z_ = 0.1 * std::abs(z_force_) + 0.9 * filtered_force_z_; // Simple low-pass filter
  // double force_threshold_ = 50.0; // Threshold for contact detection (adjust as needed)
  
  if (filtered_force_z_ > force_threshold_) 
  {
    // Only capture hold position ONCE when contact first occurs
    if (!in_contact_)
    {
      RCLCPP_WARN(get_node()->get_logger(),
        "Contact detected! Force: %.2f N — holding position", filtered_force_z_);

      for (size_t i = 0; i < joint_position_state_interface_.size(); ++i)
      {
        hold_pos_[i] = joint_position_state_interface_[i].get().get_value();
      }
      in_contact_ = true;
    }
    
    for (size_t i = 0; i < joint_position_state_interface_.size(); ++i) 
    {
      joint_velocity_command_interface_[i].get().set_value(0.0); // Set velocity command to zero to hold position
    }
    return controller_interface::return_type::OK; // Skip the rest of the update loop to maintain hold
  }
  else
  {
    if (in_contact_) {
      RCLCPP_INFO(get_node()->get_logger(), "Contact lost. Resuming trajectory execution.");
      in_contact_ = false; // Reset contact flag when force drops below threshold
      trajectory_msg_ = nullptr; // Clear the trajectory to allow new commands to be processed
    }
  }

  // The controller will keep sending the trajectory command so its normally to see the message looping
  // Trajectory interpolation logic
  if (new_msg_)
  {

    RCLCPP_INFO(get_node()->get_logger(), "Received new trajectory message, starting interpolation.");
    
    auto incoming = *traj_msg_external_point_ptr_.readFromRT();
    // Remap positions/velocities by joint name
    for (auto & pt : incoming->points)
    {
        trajectory_msgs::msg::JointTrajectoryPoint remapped;
        for (size_t i = 0; i < num_joints_; ++i)
        {
            remapped.positions.push_back(joint_position_state_interface_[i].get().get_value());
            remapped.velocities.push_back(0.0);
        }
        remapped.time_from_start = pt.time_from_start;

        for (size_t j = 0; j < incoming->joint_names.size(); ++j)
        {
            auto it = joint_name_to_index_.find(incoming->joint_names[j]);
            if (it != joint_name_to_index_.end())
            {
                remapped.positions[it->second] = pt.positions[j];
                if (j < pt.velocities.size())
                    remapped.velocities[it->second] = pt.velocities[j];
            }
        }
        pt = remapped;
    }
    
    trajectory_msg_ = incoming;
    start_time_ = time;
    new_msg_ = false;
  }
  if (trajectory_msg_ != nullptr)
  {
    rclcpp::Duration elapsed = time - start_time_;
    auto & last_point = trajectory_msg_->points.back();
    double total_time = last_point.time_from_start.sec + 
                        last_point.time_from_start.nanosec * 1e-9;

    // RCLCPP_INFO(get_node()->get_logger(), "elapsed: %.2f", elapsed.seconds());
    // RCLCPP_INFO(get_node()->get_logger(), "point_interp pos[0]: %.4f", point_interp_.positions[0]);
    // RCLCPP_INFO(get_node()->get_logger(), "joint 0 current pos: %.4f", 
    //     joint_position_state_interface_[0].get().get_value());
    

    // Clamp elapsed time to total trajectory duration
    double clamped_elapsed = std::min(elapsed.seconds(), total_time);

    if (elapsed.seconds() >= total_time)
    {
        // Hold final waypoint position
        for (size_t i = 0; i < num_joints_; ++i)
        {
          joint_velocity_command_interface_[i].get().set_value(0.0); 
        }
        trajectory_msg_ = nullptr;
        return controller_interface::return_type::OK;
    }

    interpolate_trajectory_point(*trajectory_msg_, 
        rclcpp::Duration::from_seconds(clamped_elapsed), point_interp_);
    
    PDControl();

    for (size_t i = 0; i < joint_velocity_command_interface_.size(); i++)
    {
      // RCLCPP_INFO(get_node()->get_logger(), "Setting joint %zu velocity command to %.2f", i, point_interp_.velocities[i]);

      // const double debug_vel = 1.0; // Set a constant velocity for debugging
      // joint_velocity_command_interface_[i].get().set_value(debug_vel);

      joint_velocity_command_interface_[i].get().set_value(vel_cmd[i]);
    }
  }

  return controller_interface::return_type::OK;
}

void HybridFTController::triggerEStop(const std::string & reason)
{
    if (emergency_stop_) return; // already in estop, dont repeat

    RCLCPP_ERROR(get_node()->get_logger(), "EMERGENCY STOP: %s", reason.c_str());

    // Capture current position
    estop_hold_pos_.resize(num_joints_);
    for (size_t i = 0; i < joint_position_state_interface_.size(); ++i)
    {
        estop_hold_pos_[i] = joint_position_state_interface_[i].get().get_value();
    }

    // Clear any active trajectory
    trajectory_msg_ = nullptr;
    in_contact_ = false;
    emergency_stop_ = true;
}

controller_interface::CallbackReturn HybridFTController::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Hold current position before deactivating
  for (size_t i = 0; i < num_joints_; ++i)
  {
    double current_pos = joint_position_state_interface_[i].get().get_value();
    joint_velocity_command_interface_[i].get().set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(), "Controller deactivated — holding position");
  return controller_interface::CallbackReturn::SUCCESS;
  // release_interfaces();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HybridFTController::on_cleanup(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HybridFTController::on_error(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HybridFTController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}




// void JointImpedanceExampleController::updateJointStates()
// {
//   for (size_t i = 0; i < num_joints_; ++i)
//   {
//     q_(i)  = joint_position_interfaces_[i].get().get_value();
//     dq_(i) = joint_velocity_interfaces_[i].get().get_value();
//   }
// }


}  // namespace Custom_Franka_Controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  Custom_Franka_Controller::HybridFTController, controller_interface::ControllerInterface)
  