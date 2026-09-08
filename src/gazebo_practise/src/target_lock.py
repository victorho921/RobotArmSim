#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import (
    MoveItErrorCodes,
    Constraints,
    OrientationConstraint,
    PositionConstraint,
    BoundingVolume
)
from shape_msgs.msg import SolidPrimitive
from pilz_industrial_motion_planner_testutils import *   # optional helper
# Alternative pure MoveIt way below

from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
import moveit_commander
import sys
import math
from copy import deepcopy
from tf_transformations import quaternion_from_euler, quaternion_multiply

class CameraLockMover(Node):
    def __init__(self):
        super().__init__('camera_lock_mover')
        moveit_commander.roscpp_initialize(sys.argv)

        self.arm = MoveGroupCommander("manipulator")   # change if needed
        self.arm.set_planning_time(10.0)
        self.arm.set_num_planning_attempts(15)

        # Important frames
        self.ee_link = self.arm.get_end_effector_link()
        self.camera_link = "fr3_camera_link"     # ← CHANGE to your real camera frame
        self.base_frame = "fr3_link0"

        self.get_logger().info(f"End effector: {self.ee_link}")
        self.get_logger().info(f"Camera frame: {self.camera_link}")

    def create_look_at_constraint(self, target_point, weight=1.0):
        """
        Create an orientation constraint that tries to keep the camera
        pointing toward a fixed target point.
        """
        # This is a simplified version.
        # A true look-at needs dynamic orientation calculation per waypoint.
        # For better results we compute Cartesian path + force orientation.

        constraint = OrientationConstraint()
        constraint.link_name = self.camera_link
        constraint.header.frame_id = self.base_frame
        constraint.orientation.w = 1.0          # will be overwritten per waypoint
        constraint.absolute_x_axis_tolerance = 0.2
        constraint.absolute_y_axis_tolerance = 0.2
        constraint.absolute_z_axis_tolerance = 0.2
        constraint.weight = weight

        return constraint

    def compute_look_at_orientation(self, camera_position, target_point):
        """
        Compute quaternion so that the camera optical axis (-Z or +Z)
        points toward the target.
        Adjust the axis according to your camera optical frame convention.
        """
        dx = target_point.x - camera_position.x
        dy = target_point.y - camera_position.y
        dz = target_point.z - camera_position.z

        # Simple look-at (camera looking along -Z is common for optical frames)
        # You may need to change the rotation order depending on your camera frame
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(-dz, math.sqrt(dx*dx + dy*dy))

        q = quaternion_from_euler(0, pitch + math.pi/2, yaw)  # adjust offset as needed
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def move_cartesian_with_look_at(self, waypoints, target_point, 
                                    velocity_scale=0.15, eef_step=0.01):
        """
        Move through Cartesian waypoints while trying to keep camera looking at target.
        """
        self.arm.set_max_velocity_scaling_factor(velocity_scale)
        self.arm.set_max_acceleration_scaling_factor(velocity_scale)

        # Compute Cartesian path
        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints,
            eef_step,
            0.0  # jump threshold
        )

        if fraction < 0.90:
            self.get_logger().warn(f"Only {fraction*100:.1f}% of the path was planned")
            return False

        self.get_logger().info(f"Cartesian path planned ({fraction*100:.1f}%)")
        self.arm.execute(plan, wait=True)
        return True

    def move_circular_with_look_at(self, start_pose, end_pose, via_point, target_point,
                                   velocity_scale=0.12):
        """
        Approximate a circular motion with multiple Cartesian waypoints
        while keeping the camera roughly pointed at the target.
        """
        self.get_logger().info("Generating circular waypoints...")

        waypoints = []
        steps = 20

        for i in range(steps + 1):
            t = i / steps
            # Simple circular interpolation (better methods exist)
            pose = Pose()
            pose.position.x = (1-t)**2 * start_pose.position.x + \
                              2*(1-t)*t * via_point.x + \
                              t**2 * end_pose.position.x
            pose.position.y = (1-t)**2 * start_pose.position.y + \
                              2*(1-t)*t * via_point.y + \
                              t**2 * end_pose.position.y
            pose.position.z = (1-t)**2 * start_pose.position.z + \
                              2*(1-t)*t * via_point.z + \
                              t**2 * end_pose.position.z

            # Compute orientation that looks at the target
            pose.orientation = self.compute_look_at_orientation(pose.position, target_point)
            waypoints.append(pose)

        return self.move_cartesian_with_look_at(waypoints, target_point, velocity_scale)

    def move_straight_with_look_at(self, start_pose, end_pose, target_point,
                                   velocity_scale=0.15):
        """Straight line while looking at target"""
        waypoints = []
        steps = 15

        for i in range(steps + 1):
            t = i / steps
            pose = Pose()
            pose.position.x = (1-t) * start_pose.position.x + t * end_pose.position.x
            pose.position.y = (1-t) * start_pose.position.y + t * end_pose.position.y
            pose.position.z = (1-t) * start_pose.position.z + t * end_pose.position.z
            pose.orientation = self.compute_look_at_orientation(pose.position, target_point)
            waypoints.append(pose)

        return self.move_cartesian_with_look_at(waypoints, target_point, velocity_scale)


def main():
    rclpy.init()
    node = CameraLockMover()

    # ========== USER CONFIGURATION ==========
    # Target object position (the point the camera should look at)
    target = Point(x=0.5, y=0.0, z=0.15)

    # Current / start pose of the end-effector
    start_pose = node.arm.get_current_pose().pose

    # Goal pose (example)
    end_pose = deepcopy(start_pose)
    end_pose.position.x = 0.45
    end_pose.position.y = 0.25
    end_pose.position.z = 0.35

    # Via point for circular motion
    via = Point(x=0.50, y=0.15, z=0.40)

    # ========== EXECUTE ==========
    print("\n=== 1. Straight line while looking at target ===")
    node.move_straight_with_look_at(start_pose, end_pose, target, velocity_scale=0.12)

    print("\n=== 2. Circular motion while looking at target ===")
    node.move_circular_with_look_at(start_pose, end_pose, via, target, velocity_scale=0.10)

    rclpy.shutdown()


if __name__ == '__main__':
    main()