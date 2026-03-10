from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ── SLAM Toolbox ──────────────────────────────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_navigation'),
                'launch',
                'slam.launch.py'
            )
        ),
        launch_arguments={
            'params_file': os.path.join(
                get_package_share_directory('orchestrator'),
                'config', 'slam_toolbox_params.yaml'
            )
        }.items()
    )

    # ── Nav2 ──────────────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_navigation'),
                'launch',
                'nav2.launch.py'
            )
        ),
        launch_arguments={
            'params_file': os.path.join(
                get_package_share_directory('orchestrator'),
                'config', 'nav2.yaml'
            )
        }.items()
    )

    # ── RViz ──────────────────────────────────────────────────────────────────
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_viz'),
                'launch',
                'view_robot.launch.py'
            )
        )
    )

    # ── ArUco detector ────────────────────────────────────────────────────────
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('aruco_detector'),
                'launch',
                'aruco_detector.launch.py'
            )
        )
    )

    # ── Nodes ─────────────────────────────────────────────────────────────────
    map_annotator = Node(
    package='lab6',   # or whatever package it's in
    executable='map_annotator',
    name='map_annotator',
    output='screen',
    )
    
    wall_follower = Node(
        package='lab4',
        executable='wall_follower',
        name='wall_follower',
        output='screen',
    )

    orchestrator = Node(
        package='orchestrator',
        executable='orchestrator',
        name='orchestrator',
        output='screen',
    )

    return LaunchDescription([
        slam_launch,
        nav2_launch,
        rviz_launch,
        aruco_launch,
        wall_follower,
        map_annotator,
        orchestrator,
    ])