import os
import launch
import xacro
import re
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
# from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

from xml.etree.ElementTree import fromstring, ParseError
import sys


def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    # Get packet form share directory
    pkg_gazebo = get_package_share_directory('ros_gz_sim')
    pkg_robot = get_package_share_directory('gazebo_practise')
    pkg_franka_robot = get_package_share_directory('franka_description')

    # Get controller yaml file
    controller_yaml_file = os.path.join(pkg_robot, 'controller_config', 'controller.yaml')

    # Get the urdf.xacro file
    xacro_file = os.path.join(pkg_franka_robot, 'robots', 'fr3', 'fr3.urdf.xacro')
    
    robot_description = xacro.process_file(
        xacro_file, 
        mappings={'ros2_control': 'true', 'gazebo': 'true'}
    ).toxml()
    robot_description = remove_comments(robot_description)
    
    output_file = os.path.join(os.getcwd(), "franka_robot.urdf")
    
    with open(output_file, "w") as f:
        f.write(robot_description)  # write the string  
    # print(f"URDF written to: {output_file}")

    # robot_description = ParameterValue(robot_description, value_type=None)

    # try:
    #     with open(urdf_file, 'r') as f:
    #          robot_description = f.read()
    # except FileNotFoundError:
    #     raise RuntimeError(f"URDF file not found at: {urdf_file}")

    # Get Rviz config file path
    rviz_config_path = os.path.join(pkg_robot, 'config', 'my_robot_config.rviz')

    # For publishing robot information
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        
    )

    # # For publishing joint information
    # Only when using Rviz
    joint_state_publisher = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name="joint_state_publisher_gui",
    )

    # Get the sdf file
    world_path = os.path.join(pkg_robot,'worlds','world.sdf')

    # For launching gazebo
    launch_gazebo = ExecuteProcess(
            cmd=['ign', 'gazebo','-v', '4','-r',world_path],    
            output='screen'
    )

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

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            # moveit_config.to_dict(),
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.trajectory_execution,
            {'use_sim_time': True},
        ],
    )

    # For launching Rviz
    launch_rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True},
                        moveit_config.robot_description,
                        moveit_config.robot_description_semantic,
                        moveit_config.robot_description_kinematics,
                        moveit_config.planning_pipelines,],
    )

    # For spawning robot node in gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=['-topic', '/robot_description'],
        # arguments=['-string', sdf_path, '-name', 'fr3'],
        output='screen',
    )

    ign_bridge_config = os.path.join(pkg_robot, 'config', 'ign_bridge.yaml')
    ft_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ft_sensor_bridge',
        output='screen',
        parameters=[{'config_file': ign_bridge_config}],
    )


    # ------------------------------------ Controller ------------------------------------ #
    # For launching the joint state broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # load_joint_trajectory_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_trajectory_controller','--inactive'],
    #     output='screen'
    # )

    # For launching custom joint space controller
    load_HybridFT_controller = Node(
        package='controller_manager',
        executable='spawner',
        # arguments=['robot_controller','--inactive'],
        arguments=['robot_controller'],
        output='screen'
    )

    # For launching custom task space controller
    load_TaskSpace_controller = Node(
        package='controller_manager',
        executable='spawner',
        # arguments=['task_space_controller', '--param-file', controller_yaml_file],
        arguments=['task_space_controller'],
        output='screen'
    )

    # For launching moveit arm controller
    load_moveit_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                    '--controller-manager', '/controller_manager',],
        output='screen'
    ) 
    # For launching moveit gripper controller 
    load_moveit_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller',
                    '--controller-manager', '/controller_manager',],
        output='screen'
    )

    
    return LaunchDescription([
        robot_state_publisher,
        move_group_node,
        # joint_state_publisher,
        launch_gazebo,
        spawn_robot,
        launch_rviz,
        ft_bridge,

        # load_HybridFT_controller,
        # load_joint_trajectory_controller,
        load_joint_state_broadcaster,

        load_moveit_arm_controller,
        load_moveit_gripper_controller
    ])


