from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C021229-if00',
            description='头部控制器串口设备路径'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='串口波特率'
        ),
        
        # 头部控制节点
        Node(
            package='ros2_head_control',
            executable='head_control',
            name='head_control_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate')
            }]
        )
    ])