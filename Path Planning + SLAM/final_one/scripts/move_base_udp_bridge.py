#!/usr/bin/env python3
import rospy
import math
import time
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import socket
from nav_msgs.msg import Path, Odometry
import struct
from tf.transformations import euler_from_quaternion

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower')

        # UDP Configuration
        self.UDP_IP = rospy.get_param("~ip", "192.168.1.120")
        self.UDP_PORT = rospy.get_param("~port", 43893)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Path tracking
        self.current_path = None
        self.path_sub = rospy.Subscriber(
            '/move_base/NavfnROS/plan',
            Path,
            self.path_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        rospy.loginfo("🚀 Path follower ready (using move_base for execution)")

        # Odometry Subscriber
        self.current_pose = None
        self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        """Odometry callback to update robot's position and orientation."""
        self.current_pose = msg.pose.pose

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

    def calculate_angle_to_goal(self, goal_pose):
        """Calculate the angle to the goal from the robot's current position."""
        dx = goal_pose.position.x - self.current_pose.position.x
        dy = goal_pose.position.y - self.current_pose.position.y
        return math.atan2(dy, dx)

    def calculate_distance_to_goal(self, goal_pose):
        """Calculate the distance to the goal."""
        dx = goal_pose.position.x - self.current_pose.position.x
        dy = goal_pose.position.y - self.current_pose.position.y
        return math.sqrt(dx**2 + dy**2)

    def move_to_goal(self, goal_pose):
        """Move towards the goal by first rotating, then translating, and then aligning heading."""
        rospy.loginfo(f"🚀 Moving towards waypoint")

        # Step 1: Rotate to face the goal's direction
        target_angle = self.calculate_angle_to_goal(goal_pose)
        rospy.loginfo(f"🎯 Target angle: {target_angle:.2f} radians")

        while not rospy.is_shutdown():
            if not self.current_pose:
                rospy.logwarn("⚠ Waiting for odometry data...")
                time.sleep(0.1)
                continue

            # Calculate the current angle
            current_angle = self.get_yaw(self.current_pose.orientation)
            angle_diff = self.normalize_angle(target_angle - current_angle)

            # Rotate to target angle
            if abs(angle_diff) > 0.1:
                angular_velocity = 0.3 * angle_diff  # Rotational speed (adjust as needed)
                linear_velocity = 0.0
                self.send_udp_command(linear_velocity, angular_velocity)
            else:
                rospy.loginfo(f"✅ Aligned to target angle.")
                break

            time.sleep(0.1)

        # Step 2: Move towards the goal
        distance_to_goal = self.calculate_distance_to_goal(goal_pose)
        while distance_to_goal > 0.1:  # Adjust threshold for goal reach
            if not self.current_pose:
                rospy.logwarn("⚠ Waiting for odometry data...")
                time.sleep(0.1)
                continue

            distance_to_goal = self.calculate_distance_to_goal(goal_pose)

            linear_velocity = min(0.2, distance_to_goal)  # Forward speed (adjust as needed)
            angular_velocity = 0.0  # No rotation, just move forward

            self.send_udp_command(linear_velocity, angular_velocity)
            time.sleep(0.1)

        # Step 3: Align the robot's heading to the goal's orientation
        target_orientation = self.get_yaw(goal_pose.orientation)
        while not rospy.is_shutdown():
            if not self.current_pose:
                rospy.logwarn("⚠ Waiting for odometry data...")
                time.sleep(0.1)
                continue

            current_angle = self.get_yaw(self.current_pose.orientation)
            angle_diff = self.normalize_angle(target_orientation - current_angle)

            # Rotate to final target orientation
            if abs(angle_diff) > 0.1:
                angular_velocity = 0.3 * angle_diff  # Rotational speed (adjust as needed)
                linear_velocity = 0.0
                self.send_udp_command(linear_velocity, angular_velocity)
            else:
                rospy.loginfo(f"✅ Final heading aligned.")
                break

            time.sleep(0.1)

        rospy.loginfo("✅ Goal reached!")

    def get_yaw(self, orientation):
        """Convert quaternion orientation to yaw (rotation around Z-axis)."""
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return yaw

    def normalize_angle(self, angle):
        """Normalize the angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def path_callback(self, msg):
        """Send the final waypoint of the received path to move_base."""
        if len(msg.poses) < 2:
            rospy.logwarn("⚠ Empty path received")
            return

        self.current_path = msg
        goal_pose = msg.poses[-1].pose  # Get the last point in the path
        rospy.loginfo(f"🎯 Sending goal to move_base: x={goal_pose.position.x:.2f}, y={goal_pose.position.y:.2f}")

        self.move_to_goal(goal_pose)
        rospy.loginfo("✅ Goal execution completed")

if __name__ == '__main__':
    try:
        PathFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown")
