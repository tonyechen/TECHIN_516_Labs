from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_detector',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen',
            parameters=[{
                'marker_size': 0.03,               # 30 mm
                'dictionary': 'DICT_4X4_50',
                'image_topic': '/oakd/right/image_raw/compressed',
                'camera_info_topic': '/oakd/right/camera_info',
                'publish_debug_image': True,
            }]
        )
    ])