#!/usr/bin/env python3
"""
单个摄像头RGB图像HTTP流启动文件
发布RGB彩色图像到指定端口
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='webrtc_pub',
            executable='http_camera_node',
            name='rgb_camera_stream_node',
            parameters=[
                {'camera_topic': '/camera/color/image_raw'},
                {'http_port': 8080},
                {'jpeg_quality': 80},
                {'target_width': 640},
                {'target_height': 480},
                {'host': '0.0.0.0'}
            ],
            output='screen',
            emulate_tty=True
        )
    ])
