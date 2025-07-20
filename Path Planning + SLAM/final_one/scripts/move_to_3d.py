#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PointStamped
import tf2_ros
import tf2_geometry_msgs

class CoordinateProjector:
    def __init__(self):
        rospy.init_node('coordinate_projector', anonymous=True)

        self.map_info = None
        self.map_frame = None

        # Publisher for goal
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        # Subscriber for occupancy grid
        self.map_sub = rospy.Subscriber('/rtabmap/grid_map', OccupancyGrid, self.map_callback)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_frame = msg.header.frame_id
        rospy.loginfo("Map received: resolution = %.3f, origin = (%.2f, %.2f)" % (
            self.map_info.resolution,
            self.map_info.origin.position.x,
            self.map_info.origin.position.y
        ))

    def transform_point(self, point, from_frame, to_frame):
        point_stamped = PointStamped()
        point_stamped.header.frame_id = from_frame
        point_stamped.header.stamp = rospy.Time.now()

        point_stamped.point.x = point[0]
        point_stamped.point.y = point[1]
        point_stamped.point.z = point[2]

        try:
            transform = self.tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
            transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            return (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z)
        except Exception as e:
            rospy.logerr(f"Transform error: {e}")
            return None

    def publish_goal(self, x, y, z):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.map_frame

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0  # Keep it flat

        goal.pose.orientation.w = 1.0  # No rotation

        self.goal_pub.publish(goal)
        rospy.loginfo(f"Published goal at ({x:.2f}, {y:.2f}, {z:.2f}) to {x} {y} in frame {self.map_frame}")

    def run(self):
        rospy.loginfo("Waiting for map...")
        while not rospy.is_shutdown() and self.map_info is None:
            rospy.sleep(0.1)
        rospy.loginfo("Map is ready.")

        while not rospy.is_shutdown():
            try:
                raw_input = input("Enter 3D coordinates (x y z) in camera frame (or 'q' to quit): ")
                if raw_input.lower() == 'q':
                    break
                x, y, z = map(float, raw_input.split())

                transformed = self.transform_point((x, y, z), from_frame="camera_color_optical_frame", to_frame=self.map_frame)
                if transformed is None:
                    continue

                x_map, y_map, z_map = transformed

                self.publish_goal(x_map, y_map, z_map)

            except Exception as e:
                rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    node = CoordinateProjector()
    node.run()
