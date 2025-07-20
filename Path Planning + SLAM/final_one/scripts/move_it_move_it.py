#!/usr/bin/env python3
import math
import socket
import struct
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf_trans
import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class LocalPlanController:
    def __init__(self):
        rospy.init_node('local_plan_controller')
        self.UDP_IP = "192.168.1.120"
        self.UDP_PORT = 43893
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Robot state
        self.current_pose = (0.0, 0.0, 0.0)
        self.current_plan = []
        self.executing = False
        self.current_goal_index = 0
        self.latest_plan = None
        self.last_plan_time = rospy.Time(0)

        # Parameters
        self.POSITION_TOLERANCE = 0.15  # meters
        self.ANGLE_TOLERANCE = 0.2      # radians
        self.PLAN_UPDATE_INTERVAL = 0.5  # seconds
        self.MIN_PLAN_DURATION = 1.0     # seconds

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribers
        rospy.Subscriber("/rtabmap/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self.plan_callback)

        # Timer for processing plans
        self.plan_timer = rospy.Timer(rospy.Duration(self.PLAN_UPDATE_INTERVAL), self.process_plan)
        
        rospy.loginfo("[INIT] Controller initialized, waiting for plans...")

    def odom_callback(self, msg):
        """Update current robot pose"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, theta = tf_trans.euler_from_quaternion([
            orientation_q.x, orientation_q.y,
            orientation_q.z, orientation_q.w
        ])
        self.current_pose = (x, y, theta)

    def plan_callback(self, msg):
        """Store incoming plans without immediate processing"""
        self.latest_plan = msg

    def process_plan(self, event):
        """Process buffered plan at controlled interval"""
        if self.latest_plan is None or self.executing:
            return

        if (rospy.Time.now() - self.last_plan_time).to_sec() < self.MIN_PLAN_DURATION:
            return

        try:
            transformed_plan = []
            for pose_stamped in self.latest_plan.poses:
                transformed = do_transform_pose(pose_stamped, 
                                              self.tf_buffer.lookup_transform(
                                                  "odom",
                                                  self.latest_plan.header.frame_id,
                                                  rospy.Time(0),
                                                  rospy.Duration(0.1)))
                x = transformed.pose.position.x
                y = transformed.pose.position.y
                transformed_plan.append((x, y))  # Only store position
            
            if len(transformed_plan) > 1:
                self.current_plan = transformed_plan
                self.current_goal_index = 1
                self.executing = True
                self.last_plan_time = rospy.Time.now()
                rospy.loginfo(f"[PLAN] New plan with {len(transformed_plan)} waypoints")
                self.execute_plan()

        except (tf2_ros.LookupException, 
               tf2_ros.ConnectivityException,
               tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"[TF] Transform error: {str(e)}")

    def execute_plan(self):
        """Execute the current plan focusing only on positions"""
        rospy.loginfo("[EXEC] Starting position-based plan execution")
        
        while self.executing and not rospy.is_shutdown():
            if self.current_goal_index >= len(self.current_plan):
                rospy.loginfo("[EXEC] Plan completed successfully!")
                self.executing = False
                break

            # Get current target position (ignore orientation)
            target_x, target_y = self.current_plan[self.current_goal_index]
            current_x, current_y, current_theta = self.current_pose
            
            # Calculate required motion
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            
            rospy.loginfo(f"[EXEC] Moving to waypoint {self.current_goal_index} at ({target_x:.2f}, {target_y:.2f})")
            
            try:
                # 1. Rotate to face the target
                angle_diff = self.normalize_angle(target_angle - current_theta)
                if abs(angle_diff) > self.ANGLE_TOLERANCE:
                    rospy.loginfo(f"[ROTATE] Adjusting angle by {math.degrees(angle_diff):.1f}°")
                    self.rotate_to_angle(target_angle)
                
                # 2. Move straight to target
                if distance > self.POSITION_TOLERANCE:
                    rospy.loginfo(f"[MOVE] Moving {distance:.2f}m to target")
                    self.move_forward(distance)
                
                # Check if we reached the waypoint
                current_x, current_y, _ = self.current_pose
                new_distance = math.hypot(target_x - current_x, target_y - current_y)
                if new_distance < self.POSITION_TOLERANCE:
                    self.current_goal_index += 1
                    rospy.loginfo(f"[EXEC] Reached waypoint {self.current_goal_index-1}")
                else:
                    rospy.logwarn(f"[EXEC] Failed to reach waypoint {self.current_goal_index}")
                    self.executing = False

            except Exception as e:
                rospy.logerr(f"[EXEC] Motion execution failed: {str(e)}")
                self.executing = False

    # Keep all other existing methods unchanged below this line
    # (get_current_pose, send_udp_command, normalize_angle,
    # rotate_to_angle, move_forward, execute_motion)

    def get_current_pose(self):
        return self.current_pose

    def send_udp_command(self, linear, angular):
        rospy.loginfo(f"[UDP] Sending Linear={linear:.3f}, Angular={angular:.3f}")
        try:
            ang_cmd = struct.pack('<IIId', 0x0141, 8, 1, angular)
            self.sock.sendto(ang_cmd, (self.UDP_IP, self.UDP_PORT))
            lin_cmd = struct.pack('<IIId', 0x0140, 8, 1, linear)
            self.sock.sendto(lin_cmd, (self.UDP_IP, self.UDP_PORT))
        except Exception as e:
            rospy.logerr(f"❌ UDP send failed: {str(e)}")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def rotate_to_angle(self, target_theta, speed=0.8):
        """
        Optimized rotation that always chooses the shortest angular path
        with continuous direction checking and adaptive control
        """
        # Initial setup
        MAX_STUCK_TIME = 1.0  # seconds
        last_improvement_time = rospy.Time.now().to_sec()
        min_angle_diff = float('inf')
        
        while not rospy.is_shutdown():
            # Get current state
            x, y, theta = self.get_current_pose()
            current_time = rospy.Time.now().to_sec()
            
            # Calculate normalized angle difference (-π to π)
            angle_diff = self.normalize_angle(target_theta - theta)
            abs_diff = abs(angle_diff)
            
            rospy.loginfo(f"[ROTATE] Current: {theta:.3f}, Target: {target_theta:.3f}, Diff: {math.degrees(angle_diff):.1f}°")
            
            # Check completion
            if abs_diff < self.ANGLE_TOLERANCE:
                self.send_udp_command(0, 0)
                rospy.loginfo("[ROTATE] Rotation complete")
                return True
            
            # Track minimum difference for progress monitoring
            if abs_diff < min_angle_diff:
                min_angle_diff = abs_diff
                last_improvement_time = current_time
            
            # Detect stuck condition
            if (current_time - last_improvement_time) > MAX_STUCK_TIME:
                rospy.logwarn("[ROTATE] No progress detected, aborting rotation")
                self.send_udp_command(0, 0)
                return False
            
            # Always choose optimal rotation direction
            angular_speed = speed if angle_diff > 0 else -speed
            
            # Apply command
            self.send_udp_command(0, angular_speed)
            rospy.sleep(0.05)
        
        # Shutdown case
        self.send_udp_command(0, 0)
        return False

    def move_forward(self, distance, speed=0.2):
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
        rospy.loginfo("[EXECUTE] Starting motion segment")
        x, y, theta = start_pose
        x_goal, y_goal, theta_goal = goal_pose

        # Step 1: Initial rotation to face target
        delta_rot1 = math.atan2(y_goal - y, x_goal - x) - theta
        rospy.loginfo(f"[ROT1] Required rotation: {math.degrees(delta_rot1):.1f}°")
        self.rotate_to_angle(theta + delta_rot1)

        # Step 2: Direct linear motion to target
        distance = math.hypot(x_goal - x, y_goal - y)
        rospy.loginfo(f"[MOVE] Moving {distance:.2f}m to target")
        self.move_forward(distance)

        # Optional: Uncomment to enable final orientation alignment
        current_pose = self.get_current_pose()
        delta_rot2 = theta_goal - current_pose[2]
        rospy.loginfo(f"[ROT2] Final rotation: {math.degrees(delta_rot2):.1f}°")
        self.rotate_to_angle(current_pose[2] + delta_rot2)

        rospy.loginfo("[EXECUTE] Motion segment completed")

if __name__ == "__main__":
    controller = LocalPlanController()
    rospy.spin()