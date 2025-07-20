#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from nav_msgs.srv import GetPlan

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner', anonymous=True)

        # Subscribers
        self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)

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
        rospy.loginfo("got msg...")
        self.start_pose = PoseStamped()
        
        # Manually set the header if not provided by Odometry message
        self.start_pose.header.stamp = rospy.Time.now()  # Set the current time as the stamp
        self.start_pose.header.frame_id = "map"  # Set the frame_id to 'map' (or another relevant frame)
        
        # Set the pose from the Odometry message
        self.start_pose.pose = msg.pose.pose  # Use the pose from Odometry message
        
        # rospy.loginfo("✅ Start position updated from odometry.")

if __name__ == '__main__':
    try:
        planner = PathPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
