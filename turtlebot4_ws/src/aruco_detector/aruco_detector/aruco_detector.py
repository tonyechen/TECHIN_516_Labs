#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
import cv2
import numpy as np
from cv_bridge import CvBridge


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('marker_size', 0.03)          # meters
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('camera_info_topic', '/oakd/right/camera_info')
        self.declare_parameter('image_topic', '/oakd/right/image_raw/compressed')
        self.declare_parameter('publish_debug_image', True)

        self.marker_size = self.get_parameter('marker_size').value
        dict_name = self.get_parameter('dictionary').value
        self.publish_debug = self.get_parameter('publish_debug_image').value

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dict_name)
        )
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Camera intrinsics (populated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.bridge = CvBridge()

        # Subscribers
        cam_info_topic = self.get_parameter('camera_info_topic').value
        image_topic = self.get_parameter('image_topic').value

        self.camera_info_sub = self.create_subscription(
            CameraInfo, cam_info_topic, self.camera_info_callback, 10
        )
        self.image_sub = self.create_subscription(
            CompressedImage, image_topic, self.image_callback, 10
        )

        # Publishers
        self.pose_pub = self.create_publisher(PoseArray, '/aruco_detections/poses', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '/aruco_detector/debug/compressed', 10)

        self.get_logger().info(f'ArUco detector started')
        self.get_logger().info(f'  Dictionary : {dict_name}')
        self.get_logger().info(f'  Marker size: {self.marker_size * 1000:.0f} mm')
        self.get_logger().info(f'  Image topic: {image_topic}')
        self.get_logger().info(f'  CameraInfo : {cam_info_topic}')
        self.get_logger().info(f'  Waiting for camera info...')

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera info received.')
            self.get_logger().info(f'  Resolution: {msg.width}x{msg.height}')

    def image_callback(self, msg: CompressedImage):
        if self.camera_matrix is None:
            self.get_logger().warn('No camera info yet, skipping frame.', throttle_duration_sec=5.0)
            return

        # Decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().error('Failed to decode compressed image.')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = msg.header.frame_id or 'oakd_right_camera_optical_frame'

        if ids is not None:
            self.get_logger().info(f'Detected {len(ids)} marker(s): {ids.flatten().tolist()}')

            for i, corner in enumerate(corners):
                obj_points = np.array([
                    [-self.marker_size / 2,  self.marker_size / 2, 0],
                    [ self.marker_size / 2,  self.marker_size / 2, 0],
                    [ self.marker_size / 2, -self.marker_size / 2, 0],
                    [-self.marker_size / 2, -self.marker_size / 2, 0],
                ], dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    obj_points, corner[0], self.camera_matrix, self.dist_coeffs
                )
                if not success:
                    continue
                rvec = rvec.reshape(1, 1, 3)
                tvec = tvec.reshape(1, 1, 3)

                # Convert to Pose message
                pose = Pose()
                pose.position.x = float(tvec[0][0][0])
                pose.position.y = float(tvec[0][0][1])
                pose.position.z = float(tvec[0][0][2])

                # Convert rotation vector to quaternion
                rot_matrix, _ = cv2.Rodrigues(rvec[0][0])
                quat = self.rotation_matrix_to_quaternion(rot_matrix)
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                pose_array.poses.append(pose)

                marker_id = ids[i][0]
                dist = np.linalg.norm(tvec[0][0])
                self.get_logger().info(
                    f'  Marker {marker_id}: distance={dist:.3f}m  '
                    f'x={tvec[0][0][0]:.3f} y={tvec[0][0][1]:.3f} z={tvec[0][0][2]:.3f}'
                )

                if self.publish_debug:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    cv2.drawFrameAxes(
                        frame, self.camera_matrix, self.dist_coeffs,
                        rvec, tvec, self.marker_size * 0.5
                    )

        self.pose_pub.publish(pose_array)

        if self.publish_debug:
            _, enc = cv2.imencode('.jpg', frame)
            debug_msg = CompressedImage()
            debug_msg.header = msg.header
            debug_msg.format = 'jpeg'
            debug_msg.data = enc.tobytes()
            self.debug_pub.publish(debug_msg)

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()