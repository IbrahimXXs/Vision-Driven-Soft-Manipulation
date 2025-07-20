#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

class EssentialTFPublisher:
    def __init__(self):
        rospy.init_node('essential_tf_publisher')
        
        # TF Broadcaster (using classic tf)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # Time offset constant (in seconds)
        self.time_offset = rospy.Duration(29535553.078489542)
        
        # Subscribe to RTAB-Map odometry
        rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        
        # Publish rate (40Hz)
        self.rate = rospy.Rate(40)
        rospy.loginfo("TF publisher using classic tf ready")

    def odom_callback(self, msg):
        # Store the current pose
        self.current_pose = msg.pose.pose

    def run(self):
        while not rospy.is_shutdown():
            if hasattr(self, 'current_pose'):
                current_time = rospy.Time.now() + self.time_offset
                
                # Publish map->odom transform
                self.tf_broadcaster.sendTransform(
                    (self.current_pose.position.x, 
                     self.current_pose.position.y, 
                     self.current_pose.position.z),
                    (self.current_pose.orientation.x,
                     self.current_pose.orientation.y,
                     self.current_pose.orientation.z,
                     self.current_pose.orientation.w),
                    current_time,
                    "odom",
                    "map"
                )
                
                # Publish odom->base_link (identity transform)
                self.tf_broadcaster.sendTransform(
                    (0, 0, 0),
                    (0, 0, 0, 1),
                    current_time,
                    "base_link",
                    "odom"
                )
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = EssentialTFPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass