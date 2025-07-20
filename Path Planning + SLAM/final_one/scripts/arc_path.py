#!/usr/bin/env python3
import math
import socket
import struct
import rospy
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
import tf.transformations as tf_trans
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class SmoothPathFollower:
    def __init__(self):
        rospy.init_node('smooth_path_follower')
        self.UDP_IP = "192.168.1.120"
        self.UDP_PORT = 43893
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Control parameters
        self.LOOKAHEAD_DIST = 0.4  # meters
        self.MAX_LINEAR_VEL = 0.5  # m/s
        self.MAX_ANGULAR_VEL = 0.5  # rad/s
        self.CONTROL_RATE = 20.0  # Hz
        self.GOAL_TOLERANCE = 0.1  # meters
        
        # Robot state
        self.current_pose = (0.0, 0.0, 0.0)  # x, y, theta
        self.current_path = []
        self.current_target_idx = 0
        self.active = False

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribers
        rospy.Subscriber("/rtabmap/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", Path, self.path_callback)

        # Control timer
        self.control_timer = rospy.Timer(rospy.Duration(1.0/self.CONTROL_RATE), self.control_loop)
        
        rospy.loginfo("[INIT] Smooth path follower ready")

    def odom_callback(self, msg):
        """Update current robot pose"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, theta = tf_trans.euler_from_quaternion([
            orientation_q.x, orientation_q.y,
            orientation_q.z, orientation_q.w
        ])
        self.current_pose = (x, y, theta)

    def path_callback(self, msg):
        """Store new path and transform to odom frame"""
        try:
            transformed_path = []
            for pose_stamped in msg.poses:
                transformed = do_transform_pose(pose_stamped, 
                                             self.tf_buffer.lookup_transform(
                                                 "odom",
                                                 msg.header.frame_id,
                                                 rospy.Time(0),
                                                 rospy.Duration(0.1)))
                x = transformed.pose.position.x
                y = transformed.pose.position.y
                transformed_path.append((x, y))
            
            if len(transformed_path) > 1:
                self.current_path = transformed_path
                self.current_target_idx = 0
                self.active = True
                rospy.loginfo(f"[PATH] New path received with {len(transformed_path)} points")

        except (tf2_ros.LookupException, 
               tf2_ros.ConnectivityException,
               tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"[TF] Transform error: {str(e)}")

    def control_loop(self, event):
        """Main control loop for smooth path following"""
        if not self.active or len(self.current_path) == 0:
            return

        # Get current pose
        x, y, theta = self.current_pose
        
        # Find closest point on path
        closest_dist = float('inf')
        closest_idx = 0
        for i, (path_x, path_y) in enumerate(self.current_path):
            dist = math.hypot(path_x - x, path_y - y)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i
        
        # Find lookahead point
        lookahead_idx = closest_idx
        total_dist = 0
        while lookahead_idx < len(self.current_path) - 1 and total_dist < self.LOOKAHEAD_DIST:
            dx = self.current_path[lookahead_idx+1][0] - self.current_path[lookahead_idx][0]
            dy = self.current_path[lookahead_idx+1][1] - self.current_path[lookahead_idx][1]
            total_dist += math.hypot(dx, dy)
            lookahead_idx += 1
        
        target_x, target_y = self.current_path[lookahead_idx]
        
        # Check if goal reached
        if math.hypot(self.current_path[-1][0] - x, self.current_path[-1][1] - y) < self.GOAL_TOLERANCE:
            self.active = False
            self.send_udp_command(0, 0)
            rospy.loginfo("[GOAL] Reached final target")
            return

        # Pure pursuit control
        alpha = math.atan2(target_y - y, target_x - x) - theta
        curvature = 2.0 * math.sin(alpha) / max(0.1, closest_dist)
        
        # Calculate velocities
        linear_vel = min(self.MAX_LINEAR_VEL, 
                        self.MAX_LINEAR_VEL * (1.0 - abs(curvature)/2.0))
        angular_vel = curvature * linear_vel
        angular_vel = np.clip(angular_vel, -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)
        
        # Send command
        self.send_udp_command(linear_vel, angular_vel)
        
        rospy.loginfo(f"[CONTROL] Target: ({target_x:.2f}, {target_y:.2f}), "
                     f"Vel: {linear_vel:.2f}m/s, {math.degrees(angular_vel):.1f}°/s")

    def send_udp_command(self, linear, angular):
        """Send velocity command to robot"""
        try:
            ang_cmd = struct.pack('<IIId', 0x0141, 8, 1, angular)
            self.sock.sendto(ang_cmd, (self.UDP_IP, self.UDP_PORT))
            lin_cmd = struct.pack('<IIId', 0x0140, 8, 1, linear)
            self.sock.sendto(lin_cmd, (self.UDP_IP, self.UDP_PORT))
        except Exception as e:
            rospy.logerr(f"UDP send failed: {str(e)}")

if __name__ == "__main__":
    controller = SmoothPathFollower()
    rospy.spin()