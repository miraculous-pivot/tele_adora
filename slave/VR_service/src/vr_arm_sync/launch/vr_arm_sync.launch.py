#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vr_arm_sync',
            executable='vr_arm_sync_node',
            name='vr_arm_sync_node',
            output='screen',
            parameters=[
                # 可以在这里添加参数配置
            ]
        )
    ])
