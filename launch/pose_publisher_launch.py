#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    manipulator_config_filename='manipulator_3l.yaml'
    return LaunchDescription([
        Node(
            package='dh_transform',
            executable='pose_publisher',
            parameters=[os.path.join(
                get_package_share_directory('dh_transform'),
                'config', manipulator_config_filename)],
            output='screen'),

        Node(
            package='dh_transform',
            executable='encoder',
            parameters=[os.path.join(
                get_package_share_directory('dh_transform'),
                'config', manipulator_config_filename)],
            output='screen'),

        Node(
            package='dh_transform',
            executable='service_client_lifecycle',
            output='screen'),
    ])