#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dh_transform',
            executable='listener',
            name='listener_node',
            parameters=[os.path.join(
                get_package_share_directory('dh_transform'),
                'config', 'manipulator_3l.yaml')],
            output='screen'),
    ])