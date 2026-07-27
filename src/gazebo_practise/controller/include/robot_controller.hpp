// Copyright 2023 ros2_control Development Team
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

#ifndef CUSTOM_FRANKA_CONTROLLER_HPP_
#define CUSTOM_FRANKA_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <Eigen/Eigen>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
// #include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace Custom_Franka_Controller
{
class HybridFTController : public controller_interface::ControllerInterface
{
public:
  HybridFTController();

  // Mandatory for Controller Interface
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Optional functions for robot lifecycle 
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  size_t num_joints_ = 0;
  // State and Command vectors
  std::vector<double> q_;                    // joint positions
  std::vector<double> dq_;                   // joint velocities

  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_subscriber_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>>
    traj_msg_external_point_ptr_;
  bool new_msg_ = false;

  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::WrenchStamped>> 
    wrench_external_point_ptr_;
  bool new_wrench_ = false;

  rclcpp::Time start_time_;

  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg_;
  std::shared_ptr<geometry_msgs::msg::WrenchStamped> wrench_;

  trajectory_msgs::msg::JointTrajectoryPoint point_interp_;

  // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  //   joint_position_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_velocity_command_interface_;
  // Added effort command interface for 
  // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  //   joint_effort_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_velocity_state_interface_;
  // Added effort state interface for force feedback
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_effort_state_interface_;

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
    command_interface_map_ = {
      {"velocity", &joint_velocity_command_interface_}};

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
    state_interface_map_ = {
      {"position", &joint_position_state_interface_},
      {"velocity", &joint_velocity_state_interface_},
      {"effort", &joint_effort_state_interface_}};
  
  // Used to subscribe force torque sensor topic
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_sensor_subscriber_;
  double wrench_filtered_[6] = {0.0}; // [force_x, force_y, force_z, torque_x, torque_y, torque_z]
  double force_threshold_ = 500.0;
  double filtered_force_z_ = 0.0;
  bool in_contact_ = false; // Flag to indicate if the robot is in contact with the environment
  
  
  // Controller state for hybrid control
  enum class State { MOVING, CONTACT };
  State current_state_ = State::MOVING;
  
  // Control gains
  std::vector<double> kp_, kd_, kp_hold_;
  
  // Target and hold positions
  std::vector<double> target_pos_, hold_pos_;

  // Output Command
  std::vector<double> vel_cmd;
  
  // Sensor readings
  std::vector<double> forces_;  // [force_x, force_y, force_z]
  std::vector<double> torques_; // [torque_x, torque_y, torque_z]

  std::unordered_map<std::string, size_t> joint_name_to_index_;

  bool emergency_stop_ = false;
  std::vector<double> estop_hold_pos_;

  // Functions
  void interpolate_trajectory_point(
    const trajectory_msgs::msg::JointTrajectory & traj_msg,
    const rclcpp::Duration & delta,
    trajectory_msgs::msg::JointTrajectoryPoint & point_interp
  );

  void linear_interpolate_point(
    const trajectory_msgs::msg::JointTrajectoryPoint & point_1,
    const trajectory_msgs::msg::JointTrajectoryPoint & point_2,
    trajectory_msgs::msg::JointTrajectoryPoint & point_interp, 
    double tau
  );

  void cubic_interpolate_point(
    const trajectory_msgs::msg::JointTrajectoryPoint & point_1,
    const trajectory_msgs::msg::JointTrajectoryPoint & point_2,
    trajectory_msgs::msg::JointTrajectoryPoint & point_interp, 
    double T,
    double tau
  );

  void PDControl();
  
  void triggerEStop(const std::string & reason);
};

}  // namespace Custom_Franka_Controller

#endif  