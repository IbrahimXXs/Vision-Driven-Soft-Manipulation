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

        # New parameters for plan throttling
        self.PLAN_UPDATE_INTERVAL = 0.5  # Seconds between plan updates
        self.MIN_PLAN_DURATION = 1.0     # Minimum time to follow a plan

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribers
        rospy.Subscriber("/rtabmap/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/move_base/TrajectoryPlannerROS/local_plan", Path, self.plan_callback)

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
        rospy.logdebug(f"[ODOM] Updated pose: {self.current_pose}")

    def plan_callback(self, msg):
        """Store incoming plans without immediate processing"""
        self.latest_plan = msg
        rospy.logdebug("[PLAN] Received new plan (buffered)")

    def process_plan(self, event):
        """Process buffered plan at controlled interval"""
        if self.latest_plan is None or self.executing:
            return

        # Check minimum duration between plan updates
        if (rospy.Time.now() - self.last_plan_time).to_sec() < self.MIN_PLAN_DURATION:
            rospy.logdebug("[PLAN] Skipping update - minimum duration not reached")
            return

        try:
            transformed_plan = []
            for pose_stamped in self.latest_plan.poses:
                transformed = do_transform_pose(pose_stamped, 
                                              self.tf_buffer.lookup_transform(
                                                  "odom",
                                                  self.latest_plan.header.frame_id,
                                                  rospy.Time(0),
                                                  rospy.Duration(0.1)
                                              ))
                x = transformed.pose.position.x
                y = transformed.pose.position.y
                orientation_q = transformed.pose.orientation
                _, _, theta = tf_trans.euler_from_quaternion([
                    orientation_q.x, orientation_q.y,
                    orientation_q.z, orientation_q.w
                ])
                transformed_plan.append((x, y, theta))
            
            if len(transformed_plan) > 1:
                self.current_plan = transformed_plan
                self.current_goal_index = 1
                self.executing = True
                self.last_plan_time = rospy.Time.now()
                rospy.loginfo(f"[PLAN] New plan received with {len(transformed_plan)} waypoints")
                self.execute_plan()

        except (tf2_ros.LookupException, 
               tf2_ros.ConnectivityException,
               tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"[TF] Transform error: {str(e)}")

    # Keep all existing methods EXACTLY AS THEY WERE below this line
    # (execute_plan, get_current_pose, send_udp_command, normalize_angle,
    # rotate_to_angle, move_forward, execute_motion)

    def execute_plan(self):
        """Execute the current plan using your motion primitives"""
        rospy.loginfo("[EXEC] Starting plan execution")
        
        while self.executing and not rospy.is_shutdown():
            if self.current_goal_index >= len(self.current_plan):
                rospy.loginfo("[EXEC] Plan completed successfully!")
                self.executing = False
                break

            # Get current segment
            start_pose = self.current_pose
            goal_pose = self.current_plan[self.current_goal_index]
            
            rospy.loginfo(f"[EXEC] Moving to waypoint {self.current_goal_index} at ({goal_pose[0]:.2f}, {goal_pose[1]:.2f})")
            
            try:
                # Use your existing motion execution
                self.execute_motion(start_pose, goal_pose)
                
                # Verify arrival
                current = self.current_pose
                distance = math.hypot(goal_pose[0]-current[0], goal_pose[1]-current[1])
                if distance < 0.1:
                    self.current_goal_index += 1
                    rospy.loginfo(f"[EXEC] Reached waypoint {self.current_goal_index-1}")
                else:
                    rospy.logwarn(f"[EXEC] Failed to reach waypoint {self.current_goal_index}, distance remaining: {distance:.2f}m")
                    self.executing = False

            except Exception as e:
                rospy.logerr(f"[EXEC] Motion execution failed: {str(e)}")
                self.executing = False

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

    def rotate_to_angle(self, target_theta, speed=0.3):
        prev_angle_diff = float('inf')
        wrong_direction_count = 0
        MAX_WRONG_DIRECTION = 3
        
        while not rospy.is_shutdown():
            x, y, theta = self.get_current_pose()
            theta = self.normalize_angle(theta)
            target_theta = self.normalize_angle(target_theta)
            
            angle_diff = self.normalize_angle(target_theta - theta)
            rospy.loginfo(f"[ROTATE] Current: {theta:.3f}, Target: {target_theta:.3f}, Diff: {angle_diff:.3f}")
            
            if abs(angle_diff) >= abs(prev_angle_diff) - 0.01:
                wrong_direction_count += 1
                rospy.logwarn(f"Possible wrong direction! Count: {wrong_direction_count}")
            else:
                wrong_direction_count = 0
            
            if wrong_direction_count >= MAX_WRONG_DIRECTION:
                speed *= -1
                wrong_direction_count = 0
                rospy.logwarn("Direction reversed due to wrong movement!")
            
            if abs(angle_diff) < 0.05:
                rospy.loginfo("[ROTATE] Rotation complete")
                self.send_udp_command(0, 0)
                break
            
            prev_angle_diff = abs(angle_diff)
            angular_speed = speed if angle_diff > 0 else -speed
            self.send_udp_command(0, angular_speed)
            rospy.sleep(0.15)

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

        # Step 1: Initial rotation
        delta_rot1 = math.atan2(y_goal - y, x_goal - x) - theta
        rospy.loginfo(f"[ROT1] Required rotation: {math.degrees(delta_rot1):.1f}°")
        self.rotate_to_angle(theta + delta_rot1)

        # Step 2: Forward motion
        distance = math.hypot(x_goal - x, y_goal - y)
        rospy.loginfo(f"[MOVE] Moving {distance:.2f}m to target")
        self.move_forward(distance)

        # Step 3: Final rotation
        current_pose = self.get_current_pose()
        delta_rot2 = theta_goal - current_pose[2]
        rospy.loginfo(f"[ROT2] Final rotation: {math.degrees(delta_rot2):.1f}°")
        self.rotate_to_angle(current_pose[2] + delta_rot2)

        rospy.loginfo("[EXECUTE] Motion segment completed")

if __name__ == "__main__":
    controller = LocalPlanController()
    rospy.spin()