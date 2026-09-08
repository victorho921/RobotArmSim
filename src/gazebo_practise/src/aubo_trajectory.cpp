#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <visualization_msgs/msg/marker.hpp>
#include <thread> 
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parametrization.h>

class TrajectoryVisualizer
{
public:
  TrajectoryVisualizer(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    rclcpp::QoS qos(1);
    qos.transient_local();
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "trajectory_marker", qos);
  }

  void setTrace(const std::vector<geometry_msgs::msg::Pose> & poses,
                const std::string & frame_id = "fr3_link0")
  {
    marker_.header.frame_id = frame_id;
    marker_.ns = "trajectory";
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.scale.x = 0.005;
    marker_.color.r = 0.0;
    marker_.color.g = 0.8;
    marker_.color.b = 1.0;
    marker_.color.a = 1.0;
    marker_.pose.orientation.w = 1.0;
    marker_.points.clear();
    for (const auto & pose : poses)
      marker_.points.push_back(pose.position);

    timer_ = node_->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
          marker_.header.stamp = node_->now();
          marker_pub_->publish(marker_);
        });
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  visualization_msgs::msg::Marker marker_;
};

std::vector<geometry_msgs::msg::Pose> loadPosesFromJson(
    const std::string & filepath,
    const std::string & prefix,
    const geometry_msgs::msg::Quaternion & default_orientation,
    double scale = 0.1,
    double x_offset = 0.4,
    double y_offset = 0.0,
    double z_offset = 0.4)
{
  std::ifstream file(filepath);
  nlohmann::json data;
  file >> data;

  std::vector<double> xs = data.at(prefix + "_x").get<std::vector<double>>();
  std::vector<double> ys = data.at(prefix + "_y").get<std::vector<double>>();

  bool has_z = data.contains(prefix + "_z");
  std::vector<double> zs;
  if (has_z)
    zs = data.at(prefix + "_z").get<std::vector<double>>();

  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(xs.size());

  for (size_t i = 0; i < xs.size(); ++i)
  {
      geometry_msgs::msg::Pose pose;
      pose.position.x = xs[i] * scale + x_offset;
      pose.position.y = ys[i] * scale + y_offset;
      pose.position.z = has_z ? (zs[i] * scale + z_offset) : z_offset;
      pose.orientation = default_orientation;

      poses.push_back(pose);
  }

  return poses;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "move_to_pose_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group(node, "manipulator");

  if (!move_group.getCurrentState(5.0))  // wait up to 5s, returns nullptr on timeout
  {
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for current robot state.");
    rclcpp::shutdown();
    return 1;
  }

  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(5);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  // 1. Get current pose and orientation
  geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
  if (current_pose.orientation.w == 0.0 && current_pose.orientation.x == 0.0 &&
    current_pose.orientation.y == 0.0 && current_pose.orientation.z == 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "Got invalid (zero) current pose orientation!");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Current pose: [%.3f, %.3f, %.3f] orientation [%.3f, %.3f, %.3f, %.3f]",
    current_pose.position.x, current_pose.position.y, current_pose.position.z,
    current_pose.orientation.x, current_pose.orientation.y,
    current_pose.orientation.z, current_pose.orientation.w);

  // 2. Load JSON poses using current orientation
  auto poses = loadPosesFromJson(
      "/home/victor/RobotArmSim/src/trajectory/eight_3d_temporal_100_d2.json", 
      "l2_temporal", 
      current_pose.orientation);

  // 3. Insert current pose as the explicit start point for Cartesian planning
  poses.insert(poses.begin(), current_pose);

  // 4. Publish path marker in RViz
  TrajectoryVisualizer viz(node);
  viz.setTrace(poses, "fr3_link0");

  // 5. Compute Cartesian trajectory
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.001; 

  double fraction = move_group.computeCartesianPath(poses, eef_step, jump_threshold, trajectory);

  if (fraction > 0.90) // Require at least 90% reachability
  {
    RCLCPP_INFO(node->get_logger(), "Cartesian path coverage: %.2f%%. Executing...", fraction * 100.0);
    move_group.execute(trajectory);
  } 
  else 
  {
    RCLCPP_ERROR(node->get_logger(), "Cartesian path planning failed. Only %.2f%% of path computed.", fraction * 100.0);
  }

  // 6. Safe shutdown
  executor.cancel();
  rclcpp::shutdown();
  if (spinner.joinable()) {
    spinner.join();
  }
  return 0;
}