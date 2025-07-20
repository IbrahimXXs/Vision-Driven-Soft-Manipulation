#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo, Imu

def callback(msg, pub):
    msg.header.stamp = rospy.Time.now()  # Rewrite to current time
    pub.publish(msg)

rospy.init_node('time_warper')
pub_rgb = rospy.Publisher('/camera/color/image_raw_fixed', Image, queue_size=10)
pub_depth = rospy.Publisher('/camera/depth/image_rect_raw_fixed', Image, queue_size=10)
pub_info = rospy.Publisher('/camera/color/camera_info_fixed', CameraInfo, queue_size=10)
pub_imu = rospy.Publisher('/imu/data_fixed', Imu, queue_size=10)

rospy.Subscriber('/camera/color/image_raw', Image, lambda m: callback(m, pub_rgb))
rospy.Subscriber('/camera/depth/image_rect_raw', Image, lambda m: callback(m, pub_depth))
rospy.Subscriber('/camera/color/camera_info', CameraInfo, lambda m: callback(m, pub_info))
rospy.Subscriber('/imu/data', Imu, lambda m: callback(m, pub_imu))
rospy.spin()