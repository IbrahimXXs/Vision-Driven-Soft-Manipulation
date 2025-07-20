#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

def pose_callback(msg):
    br = tf.TransformBroadcaster()
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation

    # Broadcast transform from 'map' to 'base_link' using RTAB-Map localization
    br.sendTransform((position.x, position.y, position.z),
                     (orientation.x, orientation.y, orientation.z, orientation.w),
                     rospy.Time.now(),
                     "base_link",
                     "map")

def tf_broadcaster():
    rospy.init_node('rtabmap_tf_broadcaster', anonymous=True)
    
    # Subscribe to RTAB-Map's localization pose
    rospy.Subscriber('/rtabmap/localization_pose', PoseWithCovarianceStamped, pose_callback)

    rospy.spin()

if __name__ == '__main__':
    tf_broadcaster()
