#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

class OdometryTimeFixer:
    def __init__(self):
        rospy.init_node('odometry_time_fixer', anonymous=True)

        # Subscribers
        self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        self.clock_sub = rospy.Subscriber('/clock', Clock, self.clock_callback)

        # Publisher
        self.odom_pub = rospy.Publisher('/fixed_odom', Odometry, queue_size=10)

        # Store latest clock time
        self.latest_clock_time = None

    def clock_callback(self, msg):
        """ Update latest clock time """
        self.latest_clock_time = msg.clock

    def odom_callback(self, msg):
        """ Modify odometry timestamp and republish """
        if self.latest_clock_time is None:
            rospy.logwarn("Waiting for /clock message...")
            return
        
        # Modify the timestamp
        fixed_odom = msg
        fixed_odom.header.stamp = self.latest_clock_time

        # Publish fixed odometry
        self.odom_pub.publish(fixed_odom)

if __name__ == '__main__':
    try:
        OdometryTimeFixer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
