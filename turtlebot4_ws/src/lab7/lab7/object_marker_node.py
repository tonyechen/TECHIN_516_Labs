#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from yolo_msgs.msg import DetectionArray

import tf2_ros
import tf2_geometry_msgs

import numpy as np
from cv_bridge import CvBridge


class ObjectMarkerNode(Node):

    def __init__(self):
        super().__init__('object_marker_node')

        self.bridge = CvBridge()
        self.depth_image = None
        self.camera_info = None

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Subscribers
        self.create_subscription(
            DetectionArray,
            '/yolo/detections',
            self.detection_callback,
            10)

        self.create_subscription(
            Image,
            '/oakd/rgb/preview/depth',
            self.depth_callback,
            10)

        self.create_subscription(
            CameraInfo,
            '/oakd/rgb/preview/camera_info',
            self.camera_info_callback,
            10)

        # Marker publisher
        self.marker_pub = self.create_publisher(
            Marker,
            '/lab7/object_marker',
            10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        

        self.get_logger().info("Lab7 Object Marker Node Started")

    # ------------------------------------------------------------

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        self.depth_header = msg.header

    # ------------------------------------------------------------

    def detection_callback(self, msg):
        if self.depth_image is None:
            self.get_logger().warn("No depth image yet")
            return
        if self.camera_info is None:
            self.get_logger().warn("No camera info yet")
            return

        if len(msg.detections) == 0:
            return

        self.get_logger().info(f"Processing {len(msg.detections)} detection(s)")

        for i, detection in enumerate(msg.detections):
            bbox = detection.bbox
            cx_pixel = int(bbox.center.position.x)
            cy_pixel = int(bbox.center.position.y)

            h, w = self.depth_image.shape[:2]

            if cx_pixel < 0 or cy_pixel < 0 or cx_pixel >= w or cy_pixel >= h:
                self.get_logger().warn(f"[{i}] Pixel ({cx_pixel},{cy_pixel}) out of bounds for {w}x{h} image")
                continue

            # Sample a small patch around the center to get a more stable depth
            patch_r = 3
            y0 = max(0, cy_pixel - patch_r)
            y1 = min(h, cy_pixel + patch_r + 1)
            x0 = max(0, cx_pixel - patch_r)
            x1 = min(w, cx_pixel + patch_r + 1)
            patch = self.depth_image[y0:y1, x0:x1].astype(float)
            valid = patch[patch > 0]

            if valid.size == 0:
                self.get_logger().warn(f"[{i}] No valid depth around pixel ({cx_pixel},{cy_pixel})")
                continue

            depth = float(np.median(valid))
            self.get_logger().info(f"[{i}] Class: {detection.class_name}  depth raw: {depth:.1f}")

            # Camera intrinsics
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]

            # OAK-D depth is in millimeters → convert to meters
            # If your depth image is already in meters, remove the / 1000.0
            z = depth
            x = (cx_pixel - cx) * z / fx
            y = (cy_pixel - cy) * z / fy

            self.get_logger().info(f"[{i}] 3D camera frame: ({x:.3f}, {y:.3f}, {z:.3f}) m")

            # Build PointStamped in camera frame
            # Use Time(0) so TF gives us the latest available transform
            # rather than trying to extrapolate to now (avoids clock mismatch)
            point_camera = PointStamped()
            point_camera.header.frame_id = self.camera_info.header.frame_id
            point_camera.header.stamp = Time().to_msg()  # latest available TF
            point_camera.point.x = x
            point_camera.point.y = y
            point_camera.point.z = z

            # Transform to map frame
            try:
                point_map = self.tf_buffer.transform(
                    point_camera,
                    'map',
                    timeout=Duration(seconds=0.5)
                )
            except Exception as e:
                self.get_logger().error(f"[{i}] TF transform failed: {e}")
                continue

            self.get_logger().info(
                f"[{i}] Map frame: ({point_map.point.x:.3f}, "
                f"{point_map.point.y:.3f}, {point_map.point.z:.3f}) m"
            )
            self.publish_marker(point_map, marker_id=i, label=detection.class_name)

    # ------------------------------------------------------------

    def publish_marker(self, point, marker_id=0, label='object'):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lab7'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position = point.point
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # Bright green
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        # Marker persists for 2 seconds then disappears
        # so stale markers don't pile up when objects move
        marker.lifetime.sec = 2
        marker.lifetime.nanosec = 0

        self.marker_pub.publish(marker)
        self.get_logger().info(f"Marker [{marker_id}] '{label}' published at "
                               f"({marker.pose.position.x:.3f}, "
                               f"{marker.pose.position.y:.3f}, "
                               f"{marker.pose.position.z:.3f})")


# ------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ObjectMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()