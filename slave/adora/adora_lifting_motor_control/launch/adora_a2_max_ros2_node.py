from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adora_lifting_motor_control',
            executable='adora_lifting_motor_control_node',
            name='adora_lifting_motor_control_node',
            output='screen',
            parameters=[
                {'dev': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_56D0001775-if00'},
                {'baud': 19200},
                {'max_lifting_distance': 850},
                {'robot_name': 'ADORA_A2_MAX'},
                {'sub_cmdvel_topic': '/adora/lifting_motor/cmd'},
                {'pub_position_topic': '/adora/lifting_motor/states'}
            ]
        )
    ])

 