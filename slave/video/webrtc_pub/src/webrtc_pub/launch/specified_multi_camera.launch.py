#!/usr/bin/env python3
"""
指定的多路摄像头RGB图像HTTP流启动文件
根据用户指定的4个视频话题和配置进行流传输
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 第一个摄像头 - ZED相机RGB图像流 (端口8081, 960x540)
        Node(
            package='webrtc_pub',
            executable='http_camera_node',
            name='zed_camera_rgb_stream',
            parameters=[
                {'camera_topic': '/zed/zed_node/rgb/image_rect_color'},
                {'http_port': 8081},
                {'target_width': 960},
                {'target_height': 540},
                {'jpeg_quality': 85},
                {'host': '0.0.0.0'}
            ],
            output='screen',
            emulate_tty=True
        ),
        
        # 第二个摄像头 - 相机1 RGB图像流 (端口8082, 848x480)
        Node(
            package='webrtc_pub',
            executable='http_camera_node',
            name='camera1_rgb_stream',
            parameters=[
                {'camera_topic': '/camera1/camera1/color/image_rect_raw'},
                {'http_port': 8082},
                {'target_width': 848},
                {'target_height': 480},
                {'jpeg_quality': 80},
                {'host': '0.0.0.0'}
            ],
            output='screen',
            emulate_tty=True
        ),
        
        # 第三个摄像头 - 相机2 RGB图像流 (端口8083, 848x480)
        Node(
            package='webrtc_pub',
            executable='http_camera_node',
            name='camera2_rgb_stream',
            parameters=[
                {'camera_topic': '/camera2/camera2/color/image_rect_raw'},
                {'http_port': 8083},
                {'target_width': 848},
                {'target_height': 480},
                {'jpeg_quality': 80},
                {'host': '0.0.0.0'}
            ],
            output='screen',
            emulate_tty=True
        ),
        
        # 第四个摄像头 - 主相机RGB图像流 (端口8084, 640x480)
        Node(
            package='webrtc_pub',
            executable='http_camera_node',
            name='main_camera_rgb_stream',
            parameters=[
                {'camera_topic': '/camera/color/image_raw'},
                {'http_port': 8084},
                {'target_width': 640},
                {'target_height': 480},
                {'jpeg_quality': 80},
                {'host': '0.0.0.0'}
            ],
            output='screen',
            emulate_tty=True
        )
    ])
