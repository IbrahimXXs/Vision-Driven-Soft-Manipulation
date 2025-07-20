#!/usr/bin/env python3
import rospy
import socket
import struct
from geometry_msgs.msg import Twist

class UDPCommandSender:
    def __init__(self):
        rospy.init_node('udp_command_sender')
        
        # UDP setup
        self.UDP_IP = "192.168.1.120"
        self.UDP_PORT = 43893
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Subscriber to cmd_vel
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        
        rospy.loginfo("[INIT] UDP Command Sender Node Initialized")
    
    def send_udp_command(self, linear, angular):
        rospy.loginfo(f"[UDP] Sending Linear={linear:.3f}, Angular={angular:.3f}")
        try:
            ang_cmd = struct.pack('<IIId', 0x0141, 8, 1, angular)
            self.sock.sendto(ang_cmd, (self.UDP_IP, self.UDP_PORT))
            lin_cmd = struct.pack('<IIId', 0x0140, 8, 1, linear)
            self.sock.sendto(lin_cmd, (self.UDP_IP, self.UDP_PORT))
        except Exception as e:
            rospy.logerr(f"❌ UDP send failed: {str(e)}")
    
    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        self.send_udp_command(linear_velocity, angular_velocity)

if __name__ == "__main__":
    sender = UDPCommandSender()
    rospy.spin()
