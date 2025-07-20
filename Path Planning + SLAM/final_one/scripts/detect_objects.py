#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import message_filters
import torch
import numpy as np
import cv2
from cv_bridge import CvBridge

# Detectron2 imports
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2 import model_zoo

class Object3DDetector:
    def __init__(self):
        rospy.init_node('object_3d_detector', anonymous=True)

        # Setup publishers
        self.objects_pub = rospy.Publisher('/objects', PointStamped, queue_size=10)

        # Setup CV Bridge
        self.bridge = CvBridge()

        # Setup Detectron2 predictor
        self.cfg = get_cfg()
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-Detection/faster_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5
        self.cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-Detection/faster_rcnn_R_50_FPN_3x.yaml")
        self.cfg.MODEL.DEVICE = "cpu"
        self.predictor = DefaultPredictor(self.cfg)

        # Subscribe to both RGB and Depth topics
        self.rgb_sub = message_filters.Subscriber('/camera/color/image_raw_fixed', Image)
        self.depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw_fixed', Image)

        # Synchronize the two topics
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

        # Camera intrinsics (from /camera/color/camera_info)
        self.fx = 608.3186645507812
        self.fy = 608.3549194335938
        self.cx = 425.7488098144531
        self.cy = 248.66311645507812

        rospy.loginfo("Object 3D Detector Node Started")

    def callback(self, rgb_msg, depth_msg):
        try:
            # Convert ROS images to OpenCV format
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return

        # Check image shapes
        rgb_h, rgb_w = rgb_image.shape[:2]
        depth_h, depth_w = depth_image.shape[:2]

        # Run Detectron2 object detection
        outputs = self.predictor(rgb_image)
        instances = outputs["instances"].to("cpu")
        boxes = instances.pred_boxes.tensor.numpy()
        classes = instances.pred_classes.numpy()

        # Class names for COCO (you can replace this with a custom list if needed)
        from detectron2.data import MetadataCatalog
        class_names = MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0]).thing_classes

        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box.astype(int)
            cx_bbox = int((x1 + x2) / 2)
            cy_bbox = int((y1 + y2) / 2)

            # If RGB and depth sizes differ, scale the coordinates
            cx_depth = int(cx_bbox * (depth_w / rgb_w))
            cy_depth = int(cy_bbox * (depth_h / rgb_h))

            if cx_depth < 0 or cy_depth < 0 or cx_depth >= depth_w or cy_depth >= depth_h:
                rospy.logwarn(f"Bounding box center ({cx_depth},{cy_depth}) out of depth image bounds, skipping...")
                continue

            depth_mm = depth_image[cy_depth, cx_depth]

            if depth_mm == 0 or np.isnan(depth_mm):
                rospy.logwarn(f"Invalid depth at ({cx_depth},{cy_depth}), skipping...")
                continue

            depth_m = depth_mm / 1000.0

            X = (cx_bbox - self.cx) * depth_m / self.fx
            Y = (cy_bbox - self.cy) * depth_m / self.fy
            Z = depth_m

            # Publish the 3D point
            point = PointStamped()
            point.header = Header()
            point.header.stamp = rospy.Time.now()
            point.header.frame_id = "camera_color_optical_frame"

            point.point.x = X
            point.point.y = Y
            point.point.z = Z

            self.objects_pub.publish(point)

            rospy.loginfo(f"Published object at: x={X:.2f} m, y={Y:.2f} m, z={Z:.2f} m")

            # --- Visualization ---
            label = class_names[classes[i]] if classes[i] < len(class_names) else "Unknown"
            text = f"{label} ({X:.2f},{Y:.2f},{Z:.2f}) m"

            # Draw bounding box
            cv2.rectangle(rgb_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw text above the bounding box
            (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(rgb_image, (x1, y1 - 20), (x1 + text_width, y1), (0, 255, 0), -1)
            cv2.putText(rgb_image, text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # Show the image
        cv2.imshow("Detected Objects", rgb_image)
        cv2.waitKey(1)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = Object3DDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass
