#!/usr/bin/env python3
import rospy
import math
import socket
import struct
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower')

        # UDP Configuration
        self.UDP_IP = rospy.get_param("~ip", "192.168.1.120")
        self.UDP_PORT = rospy.get_param("~port", 43893)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Controller Parameters
        self.lookahead_dist = 0.5  # meters
        self.max_linear = 0.5      # m/s
        self.max_angular = 1.0     # rad/s
        self.goal_tolerance = 0.15 # meters

        # Using /rtabmap/odom for pose
        self.current_pose = None
        self.odom_sub = rospy.Subscriber(
            '/rtabmap/odom',
            Odometry,
            self.odom_callback,
            queue_size=1
        )

        # Path tracking
        self.current_path = None
        self.active_goal = None
        self.path_sub = rospy.Subscriber(
            '/move_base/NavfnROS/plan',
            Path,
            self.path_callback,
            queue_size=1,
            buff_size=2**24
        )

        # Control timer (20Hz)
        self.control_timer = rospy.Timer(
            rospy.Duration(0.05),
            self.control_loop
        )

        rospy.loginfo("🚀 Path follower ready (using /rtabmap/odom)")

    def odom_callback(self, msg):
        """Store current pose from /rtabmap/odom"""
        self.current_pose = PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
        rospy.loginfo_throttle(1.0, 
            f"🤖 Pose: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}")

    def path_callback(self, msg):
        """Store new path and reset tracking"""
        if len(msg.poses) < 2:
            rospy.logwarn("⚠ Empty path received")
            return

        self.current_path = msg
        self.active_goal = 0
        rospy.loginfo(f"📭 New path received with {len(msg.poses)} points")
        rospy.loginfo(f"First point: x={msg.poses[0].pose.position.x:.2f}, y={msg.poses[0].pose.position.y:.2f}")
        rospy.loginfo(f"Last point: x={msg.poses[-1].pose.position.x:.2f}, y={msg.poses[-1].pose.position.y:.2f}")

    def control_loop(self, event):
        """Main control loop"""
        if self.current_path is None or self.current_pose is None:
            return

        robot_pose = self.current_pose

        # Find closest point on path
        closest_dist = float('inf')
        for i, pose in enumerate(self.current_path.poses):
            dx = pose.pose.position.x - robot_pose.pose.position.x
            dy = pose.pose.position.y - robot_pose.pose.position.y
            dist = math.sqrt(dx**2 + dy**2)
            if dist < closest_dist:
                closest_dist = dist
                self.active_goal = i

        # Find lookahead point
        lookahead_idx = self.active_goal
        total_dist = 0
        while (lookahead_idx < len(self.current_path.poses)-1 and 
               total_dist < self.lookahead_dist):
            p1 = self.current_path.poses[lookahead_idx]
            p2 = self.current_path.poses[lookahead_idx+1]
            segment_dist = math.sqrt(
                (p2.pose.position.x - p1.pose.position.x)**2 +
                (p2.pose.position.y - p1.pose.position.y)**2)
            total_dist += segment_dist
            lookahead_idx += 1

        target_pose = self.current_path.poses[min(
            lookahead_idx, 
            len(self.current_path.poses)-1)]

        # Calculate controls
        linear, angular = self.pure_pursuit(robot_pose, target_pose)
        self.send_udp_command(linear, angular)

        # Check goal reached
        last_point = self.current_path.poses[-1]
        goal_dist = math.sqrt(
            (last_point.pose.position.x - robot_pose.pose.position.x)**2 +
            (last_point.pose.position.y - robot_pose.pose.position.y)**2)
        if goal_dist < self.goal_tolerance:
            rospy.loginfo("🎯 Goal reached!")
            self.send_udp_command(0, 0)
            self.current_path = None

    def pure_pursuit(self, robot_pose, target_pose):
        """Pure pursuit control law using /rtabmap/odom data"""
        # Get current yaw from quaternion
        orientation_q = robot_pose.pose.orientation
        _, _, current_yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

        # Calculate relative position
        dx = target_pose.pose.position.x - robot_pose.pose.position.x
        dy = target_pose.pose.position.y - robot_pose.pose.position.y

        # Transform to robot frame
        x = dx * math.cos(-current_yaw) - dy * math.sin(-current_yaw)
        y = dx * math.sin(-current_yaw) + dy * math.cos(-current_yaw)

        # Calculate curvature
        curvature = 2.0 * y / (x**2 + y**2)

        # Calculate velocities
        linear = min(self.max_linear, 0.3)  # Reduced base speed
        angular = curvature * linear

        # Apply limits
        angular = max(min(angular, self.max_angular), -self.max_angular)
        return linear, angular

    def send_udp_command(self, linear, angular):
        """Send command to robot"""
        try:
            # Angular (0x0141)
            ang_cmd = struct.pack('<IIId', 0x0141, 8, 1, angular)
            self.sock.sendto(ang_cmd, (self.UDP_IP, self.UDP_PORT))
            
            # Linear (0x0140)
            lin_cmd = struct.pack('<IIId', 0x0140, 8, 1, linear)
            self.sock.sendto(lin_cmd, (self.UDP_IP, self.UDP_PORT))
            
            rospy.logdebug(f"📤 Sent command: lin={linear:.2f}, ang={angular:.2f}")
        except Exception as e:
            rospy.logerr(f"❌ UDP send failed: {str(e)}")

if __name__ == '__main__':
    try:
        PathFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown")