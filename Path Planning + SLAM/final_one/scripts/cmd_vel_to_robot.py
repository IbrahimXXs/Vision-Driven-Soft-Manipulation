#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import socket
import struct
import time

class Lite3DirectControl:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('lite3_direct_control')
        
        # Configuration (same as before)
        self.MOTION_HOST_IP = "192.168.1.120"
        self.MOTION_HOST_PORT = 43893
        self.angular_velocity_range = [-1.5, 1.5]
        self.linear_velocity_x_range = [-1.0, 1.0]
        self.linear_velocity_y_range = [-0.5, 0.5]
        
        # NEW: Add cmd_vel subscriber (just prints messages for now)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.loginfo("Subscribed to /cmd_vel topic for monitoring")
        
        rospy.loginfo("Lite3 Direct Control initialized")
        self.set_combined_velocity()

    def cmd_vel_callback(self, msg):
        """NEW: Simply print incoming cmd_vel messages"""
        print("\nReceived /cmd_vel message:")
        print(f"Linear: x={msg.linear.x:.2f}, y={msg.linear.y:.2f}, z={msg.linear.z:.2f}")
        print(f"Angular: x={msg.angular.x:.2f}, y={msg.angular.y:.2f}, z={msg.angular.z:.2f}")

    def set_combined_velocity(self):
        """Original working function unchanged"""
        try:
            # Get user inputs (same as original)
            angular_velocity = float(input(f"\nEnter Angular Velocity (rad/s) [{self.angular_velocity_range[0]} to {self.angular_velocity_range[1]}]: "))
            if angular_velocity < self.angular_velocity_range[0] or angular_velocity > self.angular_velocity_range[1]:
                rospy.logerr(f"Error: Angular Velocity must be within range {self.angular_velocity_range}.")
                return

            linear_velocity_x = float(input(f"Enter Linear Velocity in X-axis (m/s) [{self.linear_velocity_x_range[0]} to {self.linear_velocity_x_range[1]}]: "))
            if linear_velocity_x < self.linear_velocity_x_range[0] or linear_velocity_x > self.linear_velocity_x_range[1]:
                rospy.logerr(f"Error: Linear Velocity in X must be within range {self.linear_velocity_x_range}.")
                return

            linear_velocity_y = float(input(f"Enter Linear Velocity in Y-axis (m/s) [{self.linear_velocity_y_range[0]} to {self.linear_velocity_y_range[1]}]: "))
            if linear_velocity_y < self.linear_velocity_y_range[0] or linear_velocity_y > self.linear_velocity_y_range[1]:
                rospy.logerr(f"Error: Linear Velocity in Y must be within range {self.linear_velocity_y_range}.")
                return

            duration = float(input("Enter the duration to run the command (in seconds): "))
            if duration <= 0:
                rospy.logerr("Error: Duration must be greater than 0.")
                return

            # Calculate iterations (same as original)
            interval = 1 / 20  # 20Hz = 50ms
            iterations = int(duration / interval)

            # Create socket (same as original)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            for i in range(iterations):
                # Pack and send commands (same as original)
                angular_command_head = struct.pack('<III', 0x0141, 8, 1)
                angular_command_value = struct.pack('<d', angular_velocity)
                sock.sendto(angular_command_head + angular_command_value, (self.MOTION_HOST_IP, self.MOTION_HOST_PORT))

                linear_x_command_head = struct.pack('<III', 0x0140, 8, 1)
                linear_x_command_value = struct.pack('<d', linear_velocity_x)
                sock.sendto(linear_x_command_head + linear_x_command_value, (self.MOTION_HOST_IP, self.MOTION_HOST_PORT))

                linear_y_command_head = struct.pack('<III', 0x0145, 8, 1)
                linear_y_command_value = struct.pack('<d', linear_velocity_y)
                sock.sendto(linear_y_command_head + linear_y_command_value, (self.MOTION_HOST_IP, self.MOTION_HOST_PORT))

                rospy.loginfo(f"Command sent: Angular={angular_velocity}, X={linear_velocity_x}, Y={linear_velocity_y} (Iteration {i+1}/{iterations})")
                time.sleep(interval)

            rospy.loginfo("Velocity commands completed.")
            
        except ValueError:
            rospy.logerr("Error: Invalid input. Please enter numeric values.")
        except Exception as e:
            rospy.logerr(f"Error: {e}")
        finally:
            sock.close()

if __name__ == '__main__':
    try:
        controller = Lite3DirectControl()
        rospy.spin()  # NEW: Keep node alive to listen to topics
    except rospy.ROSInterruptException:
        pass