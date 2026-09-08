#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class PathAndTrackController(Node):
    def __init__(self):
        super().__init__('path_and_track_controller')

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        self.bridge = CvBridge()
        
        # 1. Image Subscriber
        self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        
        # 2. Twist Publisher targeting MoveIt Servo input topic
        self.twist_pub = self.create_publisher(
            TwistStamped, 
            '/servo_node/delta_twist_cmds', 
            10
        )
        
        # Proportional controller gains for rotation
        # self.kp_yaw = 0.0015
        self.kp_yaw = 0.009
        # self.kp_pitch = 0.0015
        self.kp_pitch = 0.009

        # Path Generation State (Example: Side-to-side sweeping motion along X-axis)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.latest_error_x = 0.0
        self.latest_error_y = 0.0
        self.target_visible = False

        # 50 Hz Control Loop Timer
        self.create_timer(0.02, self.control_loop)

        # HSV Thresholds for Red Box
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = cv_image.shape
        img_cx, img_cy = w // 2, h // 2

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv_image, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv_image, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 500:
                x, y, box_w, box_h = cv2.boundingRect(largest_contour)
                cx, cy = x + box_w // 2, y + box_h // 2
                
                # Compute pixel error
                self.latest_error_x = float(cx - img_cx)
                self.latest_error_y = float(cy - img_cy)
                self.target_visible = True
                # self.get_logger().info(f"Target detected at pixel ({cx}, {cy}) with error (X: {self.latest_error_x}, Y: {self.latest_error_y})")
                return

        self.target_visible = False
        # self.get_logger().info(f"Target not detected. No contours found or area too small.")

    def control_loop(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        # Set frame to camera_link so velocities are relative to the camera frame
        # twist.header.frame_id = 'fr3_camera_link'
        twist.header.frame_id = 'fr3_hand'

        # --- A. PATH PLANNING COMPONENT (Linear Velocities) ---
        # Example Path: Sine wave translation along camera local Y-axis
        t = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        # twist.twist.linear.x = 0.05 * np.sin(0.5 * t) # Move side-to-side
        # twist.twist.linear.y = 0.0                     # Keep fixed height
        # twist.twist.linear.z = 0.0                     # Keep fixed distance

        twist.twist.linear.x = 0.1 * np.sin(0.4 * t)        # Side-to-side sweeping (8 cm range)
        twist.twist.linear.y = 0.1 * np.cos(0.8 * t)        # Up-and-down oscillation (5 cm range)
        twist.twist.linear.z = 0.1 * np.sin(0.3 * t)        # In-and-out depth zooming (4 cm range)

        # --- B. TRACKING COMPONENT (Angular Velocities) ---
        if self.target_visible:
            # -kp * error rotates camera towards the target to reduce pixel error
            twist.twist.angular.z = -self.kp_yaw * self.latest_error_x    # Yaw
            twist.twist.angular.x = -self.kp_pitch * self.latest_error_y   # Pitch
        else:
            # Stop rotation if target is lost
            twist.twist.angular.z = 0.0
            twist.twist.angular.x = 0.0

        # Publish merged command to MoveIt Servo
        self.twist_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PathAndTrackController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()