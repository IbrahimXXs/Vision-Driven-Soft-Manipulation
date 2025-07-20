#!/usr/bin/env python3
import rospy
import math
import socket
import struct
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion

class PathExecutor:
    def __init__(self):
        rospy.init_node('path_executor')

        # UDP Configuration (from your working setup)
        self.UDP_IP = rospy.get_param("~ip", "192.168.1.120")
        self.UDP_PORT = rospy.get_param("~port", 43893)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Controller Parameters
        self.lookahead_distance = 0.3  # meters
        self.max_linear_speed = 0.5    # m/s
        self.max_angular_speed = 1.3   # rad/s

        # Verified path topic from your test
        self.path_sub = rospy.Subscriber(
            '/move_base/NavfnROS/plan',
            Path,
            self.path_callback,
            queue_size=1
        )

        # Debug publisher
        self.debug_pub = rospy.Publisher('/path_executor/debug_cmd', Twist, queue_size=1)

        rospy.loginfo("✅ Path executor ready")
        rospy.loginfo(f"📡 Listening to: /move_base/NavfnROS/plan")
        rospy.loginfo(f"📤 UDP target: {self.UDP_IP}:{self.UDP_PORT}")

    def path_callback(self, msg):
        """Handle incoming path"""
        if len(msg.poses) < 2:
            rospy.logwarn("⚠ Empty path received")
            return

        rospy.loginfo("="*50)
        rospy.loginfo(f"📭 New path received! Points: {len(msg.poses)}")
        rospy.loginfo(f"📍 First: ({msg.poses[0].pose.position.x:.2f}, {msg.poses[0].pose.position.y:.2f})")
        rospy.loginfo(f"🎯 Last: ({msg.poses[-1].pose.position.x:.2f}, {msg.poses[-1].pose.position.y:.2f})")

        # Simplified control - replace with your actual robot control logic
        cmd = Twist()
        cmd.linear.x = 0.3  # Constant forward speed
        cmd.angular.z = 0.1  # Small constant turn
        
        self.send_udp_command(cmd.linear.x, cmd.angular.z)
        self.debug_pub.publish(cmd)
        rospy.loginfo(f"📤 Sent command: lin={cmd.linear.x:.2f}, ang={cmd.angular.z:.2f}")

    def send_udp_command(self, linear, angular):
        """Your working UDP command sender"""
        try:
            # Angular (0x0141)
            ang_cmd = struct.pack('<IIId', 0x0141, 8, 1, angular)
            self.sock.sendto(ang_cmd, (self.UDP_IP, self.UDP_PORT))
            
            # Linear (0x0140)
            lin_cmd = struct.pack('<IIId', 0x0140, 8, 1, linear)
            self.sock.sendto(lin_cmd, (self.UDP_IP, self.UDP_PORT))
        except Exception as e:
            rospy.logerr(f"❌ UDP send failed: {str(e)}")

if __name__ == '__main__':
    try:
        PathExecutor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown")