#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathSubscriberTest:
    def __init__(self):
        rospy.init_node('path_subscriber_test', anonymous=True)

        # Subscriber using your exact working pattern
        self.path_sub = rospy.Subscriber(
            '/move_base/NavfnROS/plan',
            Path,
            self.path_callback,
            queue_size=1
        )

        rospy.loginfo("✅ Path subscriber initialized")
        rospy.spin()

    def path_callback(self, msg):
        """Basic callback that just verifies reception"""
        rospy.loginfo("="*50)
        rospy.loginfo("✅ PATH RECEIVED!")
        rospy.loginfo(f"Frame ID: {msg.header.frame_id}")
        rospy.loginfo(f"Number of points: {len(msg.poses)}")
        if len(msg.poses) > 0:
            rospy.loginfo(f"First point: x={msg.poses[0].pose.position.x:.2f}, y={msg.poses[0].pose.position.y:.2f}")
            rospy.loginfo(f"Last point: x={msg.poses[-1].pose.position.x:.2f}, y={msg.poses[-1].pose.position.y:.2f}")
        rospy.loginfo("="*50)

if __name__ == '__main__':
    try:
        PathSubscriberTest()
    except rospy.ROSInterruptException:
        pass