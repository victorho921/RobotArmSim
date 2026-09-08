from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro
from ament_index_python.packages import get_package_share_directory
import os
from xml.etree.ElementTree import fromstring, ParseError
import re

def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    pkg_franka_robot = get_package_share_directory('franka_description')
    xacro_file = os.path.join(pkg_franka_robot, 'robots', 'fr3', 'fr3.urdf.xacro')
    
    robot_description = xacro.process_file(
        xacro_file, 
        mappings={'ros2_control': 'true', 'gazebo': 'true'}
    ).toxml()
    robot_description = remove_comments(robot_description)

    moveit_config = (
        MoveItConfigsBuilder("fr3", package_name="franka_moveit_config")
        # .robot_description(file_path=xacro_file)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_semantic(file_path="config/fr3.srdf")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
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

    return LaunchDescription([move_to_pose_node])