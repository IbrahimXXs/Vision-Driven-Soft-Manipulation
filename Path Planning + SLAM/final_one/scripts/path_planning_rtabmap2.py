#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_srvs.srv import Empty
from nav_msgs.srv import GetPlan

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner', anonymous=True)

        # Subscribers
        self.map_sub = rospy.Subscriber('/rtabmap/grid_map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        # Publishers
        self.path_pub = rospy.Publisher('/move_base/NavfnROS/plan', Path, queue_size=10)
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.make_plan_service = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)

        # Move Base Action Client
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Variables
        self.map_data = None
        self.start_pose = None
        self.goal_pose = None

    def map_callback(self, msg):
        """ Store the received map """
        self.map_data = msg
        rospy.loginfo("✅ Map received.")

    def odom_callback(self, msg):
        """ Extract the robot's position from Odometry message and convert to PoseStamped """
        self.start_pose = PoseStamped()
        
        # Manually set the header if not provided by Odometry message
        self.start_pose.header.stamp = rospy.Time.now()  # Set the current time as the stamp
        self.start_pose.header.frame_id = "map"  # Set the frame_id to 'map' (or another relevant frame)
        
        # Set the pose from the Odometry message
        self.start_pose.pose = msg.pose.pose  # Use the pose from Odometry message
        
        # rospy.loginfo("✅ Start position updated from odometry.")

    def goal_callback(self, msg):
        """ Store the goal position and trigger path planning """
        self.goal_pose = PoseStamped()  # Ensure goal_pose is of type PoseStamped
        
        # Set the header for the goal pose
        self.goal_pose.header.stamp = rospy.Time.now()  # Set the current time as the stamp
        self.goal_pose.header.frame_id = "map"  # Set the frame_id to 'map'
        
        # Set the pose from the received message
        self.goal_pose.pose = msg.pose
        
        rospy.loginfo("✅ Goal received.")
        rospy.loginfo("start is.")
        rospy.loginfo(self.start_pose)
        rospy.loginfo("goal is.")
        rospy.loginfo(self.goal_pose)
        
        self.check_and_generate_path()

    def plan_path(self):
        """ Request path from move_base and send goal for execution """
        if self.map_data is None:
            rospy.logwarn("⚠ No map received yet.")
            return
        if self.start_pose is None:
            rospy.logwarn("⚠ No start position received yet.")
            return
        if self.goal_pose is None:
            rospy.logwarn("⚠ No goal position received yet.")
            return

        rospy.loginfo("🚀 Requesting path from move_base...")

        # Construct goal for move_base
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = self.goal_pose.pose  # Use the pose from goal_pose

        # Send goal to move_base for execution
        self.client.send_goal(goal)
        rospy.loginfo("🟢 Goal sent to move_base. Waiting for result...")

        # Wait for result
        self.client.wait_for_result()
        rospy.loginfo("✅ Path execution completed.")

    def check_and_generate_path(self):
        """ Checks if both start and goal are set, then calls the path planner. """
        if self.start_pose and self.goal_pose:
            rospy.loginfo("Both start and goal received! Cancelling move_base and generating path...")
            self.cancel_move_base_goal()  # Ensure move_base is inactive
            rospy.sleep(1)  # Allow time for cancellation
            rospy.sleep(1)  # Allow time for costmaps to clear
            self.call_make_plan()  # Generate path

    def cancel_move_base_goal(self):
        """ Cancels any active navigation goal in move_base. """
        rospy.loginfo("Cancelling move_base goal...")
        rospy.loginfo("move_base goal cancelled.")

    def clear_costmaps(self):
        """ Clears the costmaps of move_base to ensure fresh planning. """
        try:
            self.clear_costmaps_service()
            rospy.loginfo("Cleared move_base costmaps.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to clear costmaps: {e}")

    def call_make_plan(self):
        """ Calls the move_base service to get a planned path and publishes it. """
        try:
            tolerance = 0.5
            # Ensure start_pose and goal_pose are of type PoseStamped (with header) when calling the service
            response = self.make_plan_service(self.start_pose, self.goal_pose, tolerance)

            if response.plan.poses:
                rospy.loginfo("Path generated successfully!")

                # Publish the path
                path = response.plan
                path.header.frame_id = "map"
                self.path_pub.publish(path)
                rospy.loginfo("Path published!")

            else:
                rospy.logwarn("No valid path found!")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        planner = PathPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
