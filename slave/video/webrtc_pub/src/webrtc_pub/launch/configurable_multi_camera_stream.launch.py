#!/usr/bin/env python3
"""
可配置的多路摄像头RGB图像HTTP流启动文件
每个相机只发布RGB彩色图像，通过参数传递配置信息
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    return LaunchDescription([
        # 声明摄像头1参数 - RGB图像
        DeclareLaunchArgument('camera1_topic', default_value='/camera1/color/image_raw'),
        DeclareLaunchArgument('camera1_port', default_value='8081'),
        DeclareLaunchArgument('camera1_width', default_value='640'),
        DeclareLaunchArgument('camera1_height', default_value='480'),
        DeclareLaunchArgument('camera1_quality', default_value='80'),
        
        # 声明摄像头2参数 - RGB图像
        DeclareLaunchArgument('camera2_topic', default_value='/camera2/color/image_raw'),
        DeclareLaunchArgument('camera2_port', default_value='8082'),
        DeclareLaunchArgument('camera2_width', default_value='640'),
        DeclareLaunchArgument('camera2_height', default_value='480'),
        DeclareLaunchArgument('camera2_quality', default_value='80'),
        
        # 声明摄像头3参数 - RGB图像
        DeclareLaunchArgument('camera3_topic', default_value='/camera3/color/image_raw'),
        DeclareLaunchArgument('camera3_port', default_value='8083'),
        DeclareLaunchArgument('camera3_width', default_value='640'),
        DeclareLaunchArgument('camera3_height', default_value='480'),
        DeclareLaunchArgument('camera3_quality', default_value='80'),
        
        # 声明主摄像头参数 - RGB图像
        DeclareLaunchArgument('main_camera_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('main_camera_port', default_value='8084'),
        DeclareLaunchArgument('main_camera_width', default_value='1280'),
        DeclareLaunchArgument('main_camera_height', default_value='720'),
        DeclareLaunchArgument('main_camera_quality', default_value='85'),
        
        # 摄像头1 RGB图像HTTP流节点
        Node(
            package='webrtc_pub',
            executable='http_camera_node',
            name='camera1_rgb_http_stream',
            parameters=[
                {'camera_topic': LaunchConfiguration('camera1_topic')},
                {'http_port': LaunchConfiguration('camera1_port')},
                {'target_width': LaunchConfiguration('camera1_width')},
                {'target_height': LaunchConfiguration('camera1_height')},
                {'jpeg_quality': LaunchConfiguration('camera1_quality')},
                {'host': '0.0.0.0'}
            ],
            output='screen',
            emulate_tty=True,
            prefix='camera1_rgb: '
        ),
        
        # 摄像头2 RGB图像HTTP流节点
        Node(
            package='webrtc_pub',
            executable='http_camera_node',
            name='camera2_rgb_http_stream',
            parameters=[
                {'camera_topic': LaunchConfiguration('camera2_topic')},
                {'http_port': LaunchConfiguration('camera2_port')},
                {'target_width': LaunchConfiguration('camera2_width')},
                {'target_height': LaunchConfiguration('camera2_height')},
                {'jpeg_quality': LaunchConfiguration('camera2_quality')},
                {'host': '0.0.0.0'}
            ],
            output='screen',
            emulate_tty=True,
            prefix='camera2_rgb: '
        ),
        
        # 摄像头3 RGB图像HTTP流节点
        Node(
            package='webrtc_pub',
            executable='http_camera_node',
            name='camera3_rgb_http_stream',
            parameters=[
                {'camera_topic': LaunchConfiguration('camera3_topic')},
                {'http_port': LaunchConfiguration('camera3_port')},
                {'target_width': LaunchConfiguration('camera3_width')},
                {'target_height': LaunchConfiguration('camera3_height')},
                {'jpeg_quality': LaunchConfiguration('camera3_quality')},
                {'host': '0.0.0.0'}
            ],
            output='screen',
            emulate_tty=True,
            prefix='camera3_rgb: '
        ),
        
        # 主摄像头 RGB图像HTTP流节点
        Node(
            package='webrtc_pub',
            executable='http_camera_node',
            name='main_camera_rgb_http_stream',
            parameters=[
                {'camera_topic': LaunchConfiguration('main_camera_topic')},
                {'http_port': LaunchConfiguration('main_camera_port')},
                {'target_width': LaunchConfiguration('main_camera_width')},
                {'target_height': LaunchConfiguration('main_camera_height')},
                {'jpeg_quality': LaunchConfiguration('main_camera_quality')},
                {'host': '0.0.0.0'}
            ],
            output='screen',
            emulate_tty=True,
            prefix='main_camera_rgb: '
        )
    ])
