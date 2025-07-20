#!/usr/bin/env python3

import rospy
from rosgraph_msgs.msg import Clock

def clock_publisher():
    rospy.init_node("fake_clock_publisher", anonymous=True)
    clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)
    
    rate = rospy.Rate(100)  # Publish at 100 Hz

    while not rospy.is_shutdown():
        sim_time = Clock()
        sim_time.clock = rospy.Time.now()  # Use exact ROS time
        clock_pub.publish(sim_time)
        rate.sleep()

if __name__ == "__main__":
    clock_publisher()
