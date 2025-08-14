from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'dev',
            default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_56D0001775-if00',
            description='升降电机控制器串口设备路径'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='19200',
            description='串口波特率'
        ),
        DeclareLaunchArgument(
            'max_lifting_distance',
            default_value='850',
            description='最大升降距离'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='ADORA_A2_MAX',
            description='机器人名称'
        ),
        
        Node(
            package='adora_lifting_motor_control',
            executable='adora_lifting_motor_control_node',
            name='adora_lifting_motor_control_node',
            output='screen',
            parameters=[
                {'dev': LaunchConfiguration('dev')},
                {'baud': LaunchConfiguration('baud')},
                {'max_lifting_distance': LaunchConfiguration('max_lifting_distance')},
                {'robot_name': LaunchConfiguration('robot_name')},
                {'sub_cmdvel_topic': '/adora/lifting_motor/cmd'},
                {'pub_position_topic': '/adora/lifting_motor/states'}
            ]
        )
    ])

 