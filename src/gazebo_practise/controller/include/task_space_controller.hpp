#ifndef TASK_SPACE_CONTROLLER_HPP
#define TASK_SPACE_CONTROLLER_HPP

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <Eigen/Dense>

#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/logging.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace Task_Space_Controller
{
class TaskSpaceController : public controller_interface::ControllerInterface
{
public:
    // Constructor;
    TaskSpaceController();

    // Destructor;;
    ~TaskSpaceController() = default;

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
    // Members
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_frame_id_;
    std::string ee_frame_name_;
    Eigen::MatrixXd J_;         // 6 x num_joints Jacobian
    Eigen::VectorXd q_pin_;     // joint positions for pinocchio
    Eigen::VectorXd dq_pin_;    // joint velocities for pinocchio

    // Task space target
    Eigen::Vector3d target_pos_;
    Eigen::Quaterniond target_quat_;

    size_t num_joints_ = 0;

    // Task space gains
    double kp_task_ = 1.0;
    double kd_task_ = 0.1;

private:
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_velocity_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;
    std::shared_ptr<geometry_msgs::msg::WrenchStamped> wrench_;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> target_pose_;
    bool new_target_pose_ = false;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscriber_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::PoseStamped>> target_pose_buffer_;
    // Helper functions
    void computeJacobian();
    void computeTaskSpaceError(const Eigen::Vector3d & current_pos, const Eigen::Quaterniond & current_quat,
                               Eigen::VectorXd & error);
    
    void updateCurrentPose();
    void writeVelocityCommands(const Eigen::VectorXd & dq_cmd);
};
}

#endif // TASK_SPACE_CONTROLLER_HPP
