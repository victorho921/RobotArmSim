from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_aubo_robot = get_package_share_directory('aubo_description_modify')
    xacro_file = os.path.join(pkg_aubo_robot, 'urdf', 'aubo_i5_calibrated.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="aubo_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_semantic(file_path="config/my_robot.srdf")
        .to_moveit_configs()
    )

    move_to_pose_node = Node(
        package='gazebo_practise',
        executable='aubo_trajectory',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
    )

    py_move_to_pose_node = Node(
        package='gazebo_practise',
        executable='aubo_trajectory_python.py',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([move_to_pose_node])
    # return LaunchDescription([py_move_to_pose_node])