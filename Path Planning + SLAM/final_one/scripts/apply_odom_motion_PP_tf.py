#!/usr/bin/env python3
import math
import socket
import struct
import rospy
import tf2_ros
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf.transformations as tf_trans
from tf2_geometry_msgs import do_transform_pose

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower')
        
        # Robot communication setup
        self.UDP_IP = "192.168.1.120"
        self.UDP_PORT = 43893
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Robot state
        self.current_pose = (0.0, 0.0, 0.0)  # x, y, theta in odom frame
        self.path = []
        self.current_waypoint_idx = 0
        self.active = False
        
        # TF2 listener for frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Control parameters
        self.waypoint_tolerance = 0.1  # meters
        self.angle_tolerance = 0.05  # radians
        
        # ROS subscribers
        rospy.Subscriber("/rtabmap/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.path_callback)
        
        rospy.loginfo("Path follower initialized")

    def odom_callback(self, msg):
        """Update current robot pose from odometry (in odom frame)"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Convert quaternion to euler angles
        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = tf_trans.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        
        self.current_pose = (x, y, yaw)

    def transform_pose_to_odom(self, pose_stamped):
        """Transform a pose from map frame to odom frame"""
        try:
            # Get the transform from map to odom
            transform = self.tf_buffer.lookup_transform("odom", 
                                                      pose_stamped.header.frame_id,
                                                      rospy.Time(0),
                                                      rospy.Duration(1.0))
            
            # Transform the pose
            transformed_pose = do_transform_pose(pose_stamped, transform)
            return transformed_pose
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF transform failed: {e}")
            return None

    def path_callback(self, msg):
        """Handle new path from move_base (in map frame)"""
        if len(msg.poses) < 2:
            rospy.logwarn("Received empty or single-point path")
            return
            
        self.path = []
        for pose_stamped in msg.poses:
            # Transform each pose from map frame to odom frame
            transformed_pose = self.transform_pose_to_odom(pose_stamped)
            if transformed_pose is None:
                rospy.logerr("Failed to transform path - aborting")
                return
                
            x = transformed_pose.pose.position.x
            y = transformed_pose.pose.position.y
            
            # Convert quaternion to euler angles
            orientation_q = transformed_pose.pose.orientation
            roll, pitch, yaw = tf_trans.euler_from_quaternion([
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w
            ])
            
            self.path.append((x, y, yaw))
        
        self.current_waypoint_idx = 0
        self.active = True
        rospy.loginfo(f"New path received with {len(self.path)} waypoints (transformed to odom frame)")
        self.execute_next_waypoint()

    def send_velocity_command(self, linear, angular):
        """Send velocity command to robot via UDP"""
        try:
            # Limit speeds
            linear = max(min(linear, 0.3), -0.3)
            angular = max(min(angular, 0.5), -0.5)
            
            # Send commands
            ang_cmd = struct.pack('<IIId', 0x0141, 8, 1, angular)
            self.sock.sendto(ang_cmd, (self.UDP_IP, self.UDP_PORT))
            
            lin_cmd = struct.pack('<IIId', 0x0140, 8, 1, linear)
            self.sock.sendto(lin_cmd, (self.UDP_IP, self.UDP_PORT))
        except Exception as e:
            rospy.logerr(f"Failed to send command: {e}")

    def execute_next_waypoint(self):
        """Execute movement to the next waypoint in the path"""
        if not self.active or self.current_waypoint_idx >= len(self.path):
            self.send_velocity_command(0, 0)
            return
            
        # Get current target waypoint (already in odom frame)
        target = self.path[self.current_waypoint_idx]
        
        # Calculate angle to waypoint
        dx = target[0] - self.current_pose[0]
        dy = target[1] - self.current_pose[1]
        desired_angle = math.atan2(dy, dx)
        
        # Calculate angle difference - properly normalized
        angle_diff = self.normalize_angle(desired_angle - self.current_pose[2])
        
        rospy.loginfo(f"Target: {target[0]:.2f}, {target[1]:.2f} | "
                     f"Current: {self.current_pose[0]:.2f}, {self.current_pose[1]:.2f} | "
                     f"Angle to target: {math.degrees(desired_angle):.1f}° | "
                     f"Current angle: {math.degrees(self.current_pose[2]):.1f}° | "
                     f"Angle diff: {math.degrees(angle_diff):.1f}°")
        
        # First rotate to face the waypoint
        if abs(angle_diff) > self.angle_tolerance:
            # Proportional control for rotation
            angular_speed = 0.5 * angle_diff  # Proportional gain
            self.send_velocity_command(0, angular_speed)
            return
        
        # Then move forward
        distance = math.sqrt(dx**2 + dy**2)
        if distance > self.waypoint_tolerance:
            # Simple proportional control for forward motion
            linear_speed = 0.2 * min(distance, 1.0)  # Cap at 0.2 m/s
            self.send_velocity_command(linear_speed, 0)
            return
        
        # If we get here, we've reached the waypoint
        self.current_waypoint_idx += 1
        if self.current_waypoint_idx >= len(self.path):
            rospy.loginfo("Final waypoint reached!")
            self.active = False
            self.send_velocity_command(0, 0)
        else:
            rospy.loginfo(f"Reached waypoint {self.current_waypoint_idx-1}, moving to next")
            self.execute_next_waypoint()

    def normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

if __name__ == "__main__":
    try:
        follower = PathFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass