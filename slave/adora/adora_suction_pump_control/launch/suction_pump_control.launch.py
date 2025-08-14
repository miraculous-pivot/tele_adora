from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='吸盘控制器串口设备路径'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='9600',
            description='串口波特率'
        ),
        DeclareLaunchArgument(
            'channel',
            default_value='0',
            description='吸盘通道号'
        ),
        
        # 吸盘控制节点
        Node(
            package='adora_suction_pump_control',
            executable='suction_pump_node',
            name='suction_pump_control_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'channel': LaunchConfiguration('channel'),
            }],
            remappings=[
                # 可以根据需要添加话题重映射
            ]
        )
    ])
