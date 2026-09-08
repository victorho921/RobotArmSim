import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Load MoveIt Configuration (Replace with your moveit_config package name)
    moveit_config = (
        MoveItConfigsBuilder("fr3")
        .robot_description(file_path="config/fr3.urdf.xacro")
        .robot_description_semantic(file_path="config/fr3.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # 2. Path to Servo configuration YAML
    servo_config_path = os.path.join(
        get_package_share_directory("gazebo_practise"), # Your ROS 2 package name
        "config",
        "servo_config.yaml"
    )

    # 3. Create the MoveIt Servo Node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_config_path,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([servo_node])