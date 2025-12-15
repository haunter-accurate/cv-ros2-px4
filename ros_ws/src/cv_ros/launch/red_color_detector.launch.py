#!/usr/bin/env python3

"""
Launch file for the red color detector ROS2 node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description with red color detector node.
    """
    
    red_color_detector_node = Node(
        package='cv_ros',
        executable='red_color_detector_ros2',
        name='red_color_detector_ros2',
        output='screen',
        parameters=[
            {'camera_id': 0},
            {'publish_rate': 10.0},
            {'min_area': 500},
            {'headless': False}
        ]
    )
    
    return LaunchDescription([
        red_color_detector_node
    ])
