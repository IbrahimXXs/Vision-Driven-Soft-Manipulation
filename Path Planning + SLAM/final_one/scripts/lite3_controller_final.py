#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from final_one.srv import CommandVel, CommandVelResponse
import socket
import struct

class Lite3Controller:
    def __init__(self):
        rospy.init_node('lite3_nav_controller')

        # UDP Configuration
        self.UDP_IP = rospy.get_param("~ip", "192.168.1.120")
        self.UDP_PORT = rospy.get_param("~port", 43893)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Service (for manual control)
        self.vel_service = rospy.Service(
            '/lite3/command_vel', 
            CommandVel, 
            self.handle_command_vel
        )

        # Path following
        self.path_sub = rospy.Subscriber(
            '/move_base/NavfnROS/plan', 
            Path, 
            self.path_callback,
            queue_size=1
        )

        # Controller parameters
        self.lookahead_dist = 0.5  # meters
        self.max_linear = 0.5       # m/s
        self.max_angular = 1.3      # rad/s

        rospy.loginfo("Navigation controller ready")
        rospy.spin()

    def handle_command_vel(self, req):
        """Service handler for direct commands"""
        try:
            # Clip velocities to safe ranges
            linear = max(min(req.linear_x, self.max_linear), -self.max_linear)
            angular = max(min(req.angular_z, self.max_angular), -self.max_angular)
            
            self._send_udp(linear, angular)
            rospy.loginfo(f"Executed command: lin={linear:.2f}, ang={angular:.2f}")
            return CommandVelResponse(True, "Command executed")  # Properly indented
        except Exception as e:
            rospy.logerr(f"Service failed: {str(e)}")
            return CommandVelResponse(False, str(e))

    def path_callback(self, msg):
        """Convert path to velocity commands"""
        if len(msg.poses) < 2:
            return

        # Get current robot pose (simplified - use real odom in practice)
        current_pose = PoseStamped()
        current_pose.pose.position.x = 0
        current_pose.pose.position.y = 0

        # Find lookahead point
        lookahead_point = self._get_lookahead_point(msg.poses, current_pose)

        # Pure Pursuit control
        linear, angular = self._pure_pursuit(current_pose, lookahead_point)

        # Send command
        self._send_udp(linear, angular)

    def _get_lookahead_point(self, poses, current_pose):
        """Find first pose beyond lookahead distance"""
        for pose in poses:
            dx = pose.pose.position.x - current_pose.pose.position.x
            dy = pose.pose.position.y - current_pose.pose.position.y
            if math.sqrt(dx**2 + dy**2) >= self.lookahead_dist:
                return pose
        return poses[-1]  # Return last point if none found

    def _pure_pursuit(self, current_pose, target_pose):
        """Basic Pure Pursuit controller"""
        dx = target_pose.pose.position.x - current_pose.pose.position.x
        dy = target_pose.pose.position.y - current_pose.pose.position.y
        
        # Calculate curvature
        curvature = 2.0 * dy / (dx**2 + dy**2)
        
        # Convert to velocities
        linear = min(self.max_linear, 0.5)  # Constant forward speed
        angular = min(self.max_angular, curvature * linear)
        
        return linear, angular

    def _send_udp(self, linear, angular):
        """Your working UDP sender"""
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