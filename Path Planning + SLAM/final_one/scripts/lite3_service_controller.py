#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse
import socket
import struct

class Lite3ServiceController:
    def __init__(self):
        rospy.init_node('lite3_service_controller')
        
        # UDP Setup (your working config)
        self.UDP_IP = "192.168.1.120"
        self.UDP_PORT = 43893
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Service instead of subscriber
        self.srv = rospy.Service('/lite3/send_command', SetBool, self.handle_command)
        
        # Publisher for debugging
        self.cmd_pub = rospy.Publisher('/lite3/current_cmd', Twist, queue_size=1)
        
        rospy.loginfo("Service READY at /lite3/send_command")
        rospy.spin()

    def handle_command(self, req):
        """Service handler that always executes"""
        try:
            # Create a sample command (modify with your actual control logic)
            cmd = Twist()
            cmd.linear.x = 0.5 if req.data else 0.0
            cmd.angular.z = 0.3 if req.data else 0.0
            
            # Send UDP commands (your working code)
            ang_cmd = struct.pack('<IIId', 0x0141, 8, 1, cmd.angular.z)
            self.sock.sendto(ang_cmd, (self.UDP_IP, self.UDP_PORT))
            
            lin_cmd = struct.pack('<IIId', 0x0140, 8, 1, cmd.linear.x)
            self.sock.sendto(lin_cmd, (self.UDP_IP, self.UDP_PORT))
            
            # Debug publish
            self.cmd_pub.publish(cmd)
            rospy.loginfo(f"Executed command: {cmd}")
            
            return SetBoolResponse(success=True, message="Command executed")
            
        except Exception as e:
            rospy.logerr(f"Service failed: {str(e)}")
            return SetBoolResponse(success=False, message=str(e))

if __name__ == '__main__':
    Lite3ServiceController()