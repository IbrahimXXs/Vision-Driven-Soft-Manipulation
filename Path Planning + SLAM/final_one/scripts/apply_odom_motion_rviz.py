#!/usr/bin/env python3
import time
import math
import socket
import struct
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf_trans

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.UDP_IP = "192.168.1.120"  # Replace with your robot's IP
        self.UDP_PORT = 43893
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.current_pose = (0.0, 0.0, 0.0)  # (x, y, theta)

        # Subscribe to /rtabmap/odom topic
        rospy.Subscriber("/rtabmap/odom", Odometry, self.odom_callback)
        # Subscribe to Rviz 2D Nav Goal topic
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        rospy.loginfo("[INIT] Robot Controller Initialized")
        rospy.loginfo("[INIT] Ready to receive goals from Rviz '2D Nav Goal' tool...")

    def odom_callback(self, msg):
        """Callback function to update the current pose from odometry data"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract yaw (theta) from quaternion
        orientation_q = msg.pose.pose.orientation
        _, _, theta = tf_trans.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

        self.current_pose = (x, y, theta)

    def goal_callback(self, msg):
        """Callback for Rviz 2D Nav Goal"""
        x_goal = msg.pose.position.x
        y_goal = msg.pose.position.y
        
        # Extract yaw from quaternion
        orientation_q = msg.pose.orientation
        _, _, theta_goal = tf_trans.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        
        goal_pose = (x_goal, y_goal, theta_goal)
        start_pose = self.get_current_pose()
        
        rospy.loginfo(f"[GOAL] Received new goal: x={x_goal:.3f}, y={y_goal:.3f}, theta={theta_goal:.3f}")
        rospy.loginfo(f"[GOAL] Starting from: x={start_pose[0]:.3f}, y={start_pose[1]:.3f}, theta={start_pose[2]:.3f}")
        
        self.execute_motion(start_pose, goal_pose)

    def get_current_pose(self):
        """Fetch the latest odometry values"""
        return self.current_pose  

    def send_udp_command(self, linear, angular):
        """Send velocity commands via UDP"""
        rospy.loginfo(f"[UDP] Sending Linear={linear:.3f}, Angular={angular:.3f}")
        try:
            ang_cmd = struct.pack('<IIId', 0x0141, 8, 1, angular)
            self.sock.sendto(ang_cmd, (self.UDP_IP, self.UDP_PORT))
            
            lin_cmd = struct.pack('<IIId', 0x0140, 8, 1, linear)
            self.sock.sendto(lin_cmd, (self.UDP_IP, self.UDP_PORT))
        except Exception as e:
            rospy.logerr(f"❌ UDP send failed: {str(e)}")

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def rotate_to_angle(self, target_theta, speed=0.25):
        """Rotate the robot until it reaches the target angle with self-correction"""
        prev_angle_diff = float('inf')  # Track previous angle difference
        wrong_direction_count = 0      # Count consecutive wrong direction movements
        MAX_WRONG_DIRECTION = 3        # Threshold before correcting
        
        while not rospy.is_shutdown():
            x, y, theta = self.get_current_pose()
            theta = self.normalize_angle(theta)
            target_theta = self.normalize_angle(target_theta)
            
            angle_diff = self.normalize_angle(target_theta - theta)
            rospy.loginfo(f"[ROTATE] Current: {theta:.3f}, Target: {target_theta:.3f}, Diff: {angle_diff:.3f}")
            
            # Check if we're moving in the right direction
            if abs(angle_diff) >= abs(prev_angle_diff) - 0.01:  # Allowing small margin
                wrong_direction_count += 1
                rospy.logwarn(f"Possible wrong direction! Count: {wrong_direction_count}")
            else:
                wrong_direction_count = 0
            
            # If we've been going the wrong direction too many times, reverse
            if wrong_direction_count >= MAX_WRONG_DIRECTION:
                speed *= -1  # Reverse rotation direction
                wrong_direction_count = 0
                rospy.logwarn("Direction reversed due to wrong movement!")
            
            # Check for completion
            if abs(angle_diff) < 0.05:
                rospy.loginfo("[ROTATE] Rotation complete")
                self.send_udp_command(0, 0)
                break
            
            # Store current difference for next iteration
            prev_angle_diff = abs(angle_diff)
            
            # Determine rotation direction
            angular_speed = speed if angle_diff > 0 else -speed
            self.send_udp_command(0, angular_speed)
            rospy.sleep(0.15)

    def move_forward(self, distance, speed=0.2):
        """Move the robot forward by a given distance"""
        x_start, y_start, _ = self.get_current_pose()
        rospy.loginfo(f"[MOVE] Moving forward {distance:.3f} meters")

        while not rospy.is_shutdown():
            x, y, _ = self.get_current_pose()
            traveled_distance = math.sqrt((x - x_start)**2 + (y - y_start)**2)
            rospy.loginfo(f"[MOVE] Traveled: {traveled_distance:.3f} meters")

            if traveled_distance >= distance:
                self.send_udp_command(0, 0)
                break

            self.send_udp_command(speed, 0)
            rospy.sleep(0.1)

    def execute_motion(self, start_pose, goal_pose):
        """Apply the three motion steps"""
        x, y, theta = start_pose
        x_goal, y_goal, theta_goal = goal_pose
        
        rospy.loginfo("[EXECUTE] Motion Execution Started")

        # Step 1: Compute first rotation (face goal direction)
        delta_rot1 = math.atan2(y_goal - y, x_goal - x) - theta
        self.rotate_to_angle(theta + delta_rot1)  # Rotate to face goal

        # Step 2: Move forward
        self.move_forward(math.sqrt((x_goal - x)**2 + (y_goal - y)**2))

        # **Update Pose After Moving**
        x, y, theta = self.get_current_pose()

        # Step 3: Compute final rotation (align to goal orientation)
        delta_rot2 = theta_goal - theta
        self.rotate_to_angle(theta + delta_rot2)

        rospy.loginfo("[EXECUTE] Motion Execution Completed")

if __name__ == "__main__":
    controller = RobotController()
    rospy.spin()  # Just keep the node running and wait for callbacks