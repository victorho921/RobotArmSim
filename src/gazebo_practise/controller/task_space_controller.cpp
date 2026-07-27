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

#include "include/task_space_controller.hpp"

namespace Task_Space_Controller
{
TaskSpaceController::TaskSpaceController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn TaskSpaceController::on_init()
{
    if (!get_node()->has_parameter("robot_description"))
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'robot_description' not found.");
        return controller_interface::CallbackReturn::ERROR;
    }
    std::string urdf_string;
    if (!get_node()->get_parameter("robot_description", urdf_string) || urdf_string.empty()) 
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to retrieve 'robot_description' parameter.");
        return controller_interface::CallbackReturn::ERROR;
    }

    try 
    {
        pinocchio::urdf::buildModelFromXML(urdf_string, model_);
    } 
    catch (const std::exception & e) 
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Pinocchio failed to parse URDF: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }


    // Declare and get the end-effector frame name parameter
    if (!get_node()->has_parameter("end_effector_frame"))
    {
        get_node()->declare_parameter<std::string>("end_effector_frame", "");
    }
    ee_frame_name_ = get_node()->get_parameter("end_effector_frame").as_string();

    if (ee_frame_name_.empty())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'end_effector_frame' not set.");
        return controller_interface::CallbackReturn::ERROR;
    }

    if (!model_.existFrame(ee_frame_name_))
    {
        RCLCPP_ERROR(get_node()->get_logger(),
            "Frame '%s' does not exist in the URDF model.", ee_frame_name_.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    ee_frame_id_ = model_.getFrameId(ee_frame_name_);

    // 4. Initialize Pinocchio Data structures
    data_ = pinocchio::Data(model_);
    
    // Resize your internal Eigen vectors
    q_pin_.setZero(model_.nq);
    dq_pin_.setZero(model_.nv);
    J_.setZero(6, model_.nv);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TaskSpaceController::on_configure(
    const rclcpp_lifecycle::State & previous_state) 
{
    // Initialize target position and orientation
    target_pos_.setZero();
    target_quat_.setIdentity();
    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();
    joint_velocity_command_interface_.clear();

    auto callback = [this](const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) -> void
    {
        target_pose_buffer_.writeFromNonRT(msg);
        new_target_pose_ = true;
    };

    target_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/target_pose", rclcpp::SystemDefaultsQoS(), callback);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TaskSpaceController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    auto logger = get_node()->get_logger();

    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();
    joint_velocity_command_interface_.clear();

    //  // Assign command interfaces
    // for (auto & interface : command_interfaces_)
    // {
    //     const std::string interface_type = interface.get_interface_name();  // "position", "velocity", etc.
    //     auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
    //         [&](const hardware_interface::LoanedCommandInterface & interface) 
    //         { return interface.get_interface_name() == interface_type; });
    //     if (it != command_interfaces_.end())
    //     {
    //         it->second->push_back(interface);   // push into the correct vector
    //     }
    //     else
    //     {
    //         RCLCPP_ERROR(logger, "Command interface type '%s' not found in command_interfaces_ map.", interface_type.c_str());
    //     }
    // }
    // // Assign state interfaces
    // for (auto & interface : state_interfaces_)
    // {
    //     const std::string interface_name = interface.get_name();
    //     const std::string interface_type = interface.get_interface_name();  // position/velocity/effort
    //     auto it = state_interfaces_.find(interface_type);
    //     if (it != state_interfaces_.end())
    //     {
    //         it->second->push_back(interface);   // push into the correct vector
    //     }
    // }


    // 1. Clear interfaces and assign them securely
    // 2. Read current joint states immediately
    updateCurrentPose();

    // 3. Run forward kinematics to find out where the hand actually is right now
    pinocchio::forwardKinematics(model_, data_, q_pin_);
    pinocchio::updateFramePlacements(model_, data_);
    
    // 4. Set your initial target to the CURRENT position so it holds steady
    const pinocchio::SE3 & ee_pose = data_.oMf[ee_frame_id_];
    target_pos_ = ee_pose.translation();
    target_quat_ = Eigen::Quaterniond(ee_pose.rotation());

    // Seed the realtime buffer with this initial pose
    auto initial_pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    initial_pose_msg->pose.position.x = target_pos_.x();
    initial_pose_msg->pose.position.y = target_pos_.y();
    initial_pose_msg->pose.position.z = target_pos_.z();
    initial_pose_msg->pose.orientation.w = target_quat_.w();
    initial_pose_msg->pose.orientation.x = target_quat_.x();
    initial_pose_msg->pose.orientation.y = target_quat_.y();
    initial_pose_msg->pose.orientation.z = target_quat_.z();
    target_pose_buffer_.initRT(initial_pose_msg);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TaskSpaceController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    return  controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskSpaceController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // Read latest message from the real-time buffer
    auto latest_target_msg = *(target_pose_buffer_.readFromRT());
    
    if (latest_target_msg) 
    {
        target_pos_ << latest_target_msg->pose.position.x,
                    latest_target_msg->pose.position.y,
                    latest_target_msg->pose.position.z;
                    
        target_quat_ = Eigen::Quaterniond(
                    latest_target_msg->pose.orientation.w,
                    latest_target_msg->pose.orientation.x,
                    latest_target_msg->pose.orientation.y,
                    latest_target_msg->pose.orientation.z);
    } 
    else 
    {
        writeVelocityCommands(Eigen::VectorXd::Zero(model_.nv));
        return controller_interface::return_type::OK;
    }

    // 2. Update Pinocchio Kinematics
    pinocchio::forwardKinematics(model_, data_, q_pin_, dq_pin_);
    pinocchio::updateFramePlacements(model_, data_);

    // 3. Get Current End Effector Pose
    const pinocchio::SE3 & ee_pose = data_.oMf[ee_frame_id_];
    Eigen::Vector3d current_pos = ee_pose.translation();
    Eigen::Quaterniond current_quat(ee_pose.rotation());

    // 4. Calculate error
    Eigen::VectorXd error(6);
    computeTaskSpaceError(current_pos, current_quat, error);

    // 5. Generate Desired Task Space Velocity (twist) using proportional gain
    // x_dot_des = Kp * error
    Eigen::VectorXd twist_des = kp_task_ * error; 

    // 6. Compute Jacobian
    computeJacobian();

    // 7. Solve Inverse Kinematics using Pseudo-inverse (or Damped Least Squares)
    // J * dq = twist_des  =>  dq = J^+ * twist_des
    // Using Damped Least Squares (DLS) to safely handle singularities: J^T * (J * J^T + lambda^2 * I)^-1
    double lambda = 0.01; // damping factor
    Eigen::MatrixXd JJT = J_ * J_.transpose();
    JJT.diagonal().array() += lambda * lambda;
    Eigen::VectorXd dq_cmd = J_.transpose() * JJT.ldlt().solve(twist_des);

    // Write the command 
    writeVelocityCommands(dq_cmd);

    return controller_interface::return_type::OK;
}

/*Helper Functions*/
// Update the current joint positions and velocities 
void TaskSpaceController::updateCurrentPose()
{
    for (size_t i = 0; i < num_joints_; ++i)
    {
        q_pin_[i] = joint_position_state_interface_[i].get().get_value();
        dq_pin_[i] = joint_velocity_state_interface_[i].get().get_value();
    }
}

// Write the velocity commands to the command interfaces
void TaskSpaceController::writeVelocityCommands(const Eigen::VectorXd & dq_cmd)
{
    for (size_t i = 0; i < num_joints_; ++i)
    {
        joint_velocity_command_interface_[i].get().set_value(dq_cmd[i]);
    }
}
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  Task_Space_Controller::TaskSpaceController, controller_interface::ControllerInterface)