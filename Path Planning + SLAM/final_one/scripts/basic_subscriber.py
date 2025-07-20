#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def callback(msg):
    rospy.loginfo(f"Received: Linear X={msg.linear.x:.2f}, Angular Z={msg.angular.z:.2f}")

rospy.init_node('test_subscriber')
rospy.Subscriber('/test_cmd_vel', Twist, callback, queue_size=1)
rospy.loginfo("Listening to /test_cmd_vel...")
rospy.spin()