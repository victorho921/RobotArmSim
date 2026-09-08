#!/usr/bin/env python3

# Wait until Moveit2 release binary build for Moveit_py


import rclpy
from rclpy.node import Node
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose, PoseStamped

def plan_and_execute_waypoints():
    # 1. Initialize ROS 2 client library
    rclpy.init()

    # 2. Instantiate MoveItPy 
    # This automatically reads configuration from your launched node parameters
    moveit = MoveItPy(node_name="moveit_py_waypoint_node")
    
    # Change 'arm' to your specific move group planning component
    arm = moveit.get_planning_component("arm") 

    # 3. Create your list of target poses
    # In ROS 2, paths rely heavily on PoseStamped messages containing frame details
    waypoints = []

    # Target 1
    pose1 = PoseStamped()
    pose1.header.frame_id = "base_link" # Set to your robot's reference frame
    pose1.pose.position.x = 0.4
    pose1.pose.position.y = 0.1
    pose1.pose.position.z = 0.5
    pose1.pose.orientation.w = 1.0 # Maintain simple orientation
    waypoints.append(pose1)

    # Target 2
    pose2 = PoseStamped()
    pose2.header.frame_id = "base_link"
    pose2.pose.position.x = 0.4
    pose2.pose.position.y = -0.1
    pose2.pose.position.z = 0.5
    pose2.pose.orientation.w = 1.0
    waypoints.append(pose2)

    # 4. Sequentially Plan and Execute through the list
    print(f"Starting execution of {len(waypoints)} waypoints...")
    
    for i, target_pose in enumerate(waypoints):
        print(f"Planning to Waypoint {i+1}...")
        
        # Set the current goal state
        arm.set_start_state_to_current_state()
        arm.set_goal_state(pose_stamped_msg=target_pose, plan_component_name="arm")
        
        # Compute the plan
        plan_result = arm.plan()
        
        # Check if planning was successful and execute
        if plan_result:
            print(f"Executing Waypoint {i+1}...")
            # False tells it to wait until the execution is finished before continuing the loop
            moveit.execute(plan_result.trajectory, blocking=False) 
        else:
            print(f"Failed to plan to Waypoint {i+1}. Aborting sequence.")
            break

    # 5. Clean up
    rclpy.shutdown()

if __name__ == "__main__":
    plan_and_execute_waypoints()

