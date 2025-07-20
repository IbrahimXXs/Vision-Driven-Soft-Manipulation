#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from final_one.srv import CommandVel, CommandVelResponse  # Your custom service
import socket
import struct

class Lite3Controller:
    def __init__(self):
        rospy.init_node('lite3_controller')

        # UDP Configuration (from your working code)
        self.UDP_IP = rospy.get_param("~ip", "192.168.1.120")
        self.UDP_PORT = rospy.get_param("~port", 43893)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Custom Service (for direct velocity control)
        self.vel_service = rospy.Service(
            '/lite3/command_vel', 
            CommandVel, 
            self.handle_command_vel
        )

        # Topic Bridge for move_base (with debugging)
        self.path_sub = rospy.Subscriber(
            '/move_base/NavfnROS/plan', 
            Path, 
            self.path_callback,
            queue_size=1
        )

        # Debug publishers
        self.debug_path_pub = rospy.Publisher('/lite3/debug_path', Path, queue_size=1)
        self.debug_cmd_pub = rospy.Publisher('/lite3/debug_cmd', Twist, queue_size=1)

        rospy.loginfo(f"Controller ready | UDP: {self.UDP_IP}:{self.UDP_PORT}")
        rospy.spin()

    def handle_command_vel(self, req):
        """Handle custom service requests"""
        try:
            self._send_udp(req.linear_x, req.angular_z)
            rospy.loginfo(f"Service command: lin={req.linear_x:.2f}, ang={req.angular_z:.2f}")
            return CommandVelResponse(True, "Command executed")
        except Exception as e:
            rospy.logerr(f"Service failed: {e}")
            return CommandVelResponse(False, str(e))

    def path_callback(self, msg):
        """Process move_base path"""
        self.debug_path_pub.publish(msg)  # Debug output
        
        if len(msg.poses) > 0:
            # Simplified: Use first pose to determine steering
            cmd = Twist()
            cmd.linear.x = 0.5  # Constant forward speed
            cmd.angular.z = 0.2  # Simple steering
            
            self._send_udp(cmd.linear.x, cmd.angular.z)
            self.debug_cmd_pub.publish(cmd)  # Debug output

    def _send_udp(self, linear, angular):
        """Your working UDP command sender"""
        try:
            # Angular (0x0141)
            ang_cmd = struct.pack('<IIId', 0x0141, 8, 1, angular)
            self.sock.sendto(ang_cmd, (self.UDP_IP, self.UDP_PORT))
            
            # Linear (0x0140)
            lin_cmd = struct.pack('<IIId', 0x0140, 8, 1, linear)
            self.sock.sendto(lin_cmd, (self.UDP_IP, self.UDP_PORT))
        except Exception as e:
            rospy.logerr(f"UDP send failed: {e}")

if __name__ == '__main__':
    Lite3Controller()