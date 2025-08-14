from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'dt_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C027770-if00',
            description='底盘控制器串口设备路径'
        ),
        DeclareLaunchArgument(
            'dt_baudrate',
            default_value='115200',
            description='串口波特率'
        ),
        DeclareLaunchArgument(
            'dt_odom_enable',
            default_value='True',
            description='是否启用里程计'
        ),
        DeclareLaunchArgument(
            'dt_drive_type',
            default_value='SDFZ',
            description='驱动类型: SDFZ 或 HLS'
        ),
        
        Node(
            package='adora_chassis_bringup',
            executable='adora_a2_max_node',
            name='adora_a2_max_node',
            output='screen',
            parameters=[{
                'dt_port': LaunchConfiguration('dt_port'),
                'dt_baudrate': LaunchConfiguration('dt_baudrate'),
                'dt_odom_enable': LaunchConfiguration('dt_odom_enable'),
                'dt_drive_type': LaunchConfiguration('dt_drive_type'),
                'dt_log_display': True,
                'dt_original_display': False,
            }],
            remappings=[
                # pub: old, new
                ('/adora_robot/chassis/frame_info',             '/adora_robot/chassis/frame_info'),
                ('/adora_robot/chassis/drive_left_error_info',  '/adora_robot/chassis/drive_left_error_info'),
                ('/adora_robot/chassis/drive_right_error_info', '/adora_robot/chassis/drive_right_error_info'),
                ('/adora_robot/chassis/auto_charge_info',       '/adora_robot/chassis/auto_charge_info'),
                ('/adora_robot/chassis/battery_info',           '/adora_robot/chassis/battery_info'),
                ('/adora_robot/chassis/date_info',              '/adora_robot/chassis/date_info'),
                ('/adora_robot/chassis/parameter_info',         '/adora_robot/chassis/parameter_info'),
                ('/adora_robot/chassis/state_info',             '/adora_robot/chassis/state_info'),
                ('/adora_robot/chassis/velocity_info',          '/adora_robot/chassis/velocity_info'),
                ('/adora_robot/chassis/hardware_version_info',  '/adora_robot/chassis/hardware_version_info'),
                ('/adora_robot/chassis/led_strip_mode_info',    '/adora_robot/chassis/led_strip_mode_info'),
                ('/adora_robot/chassis/led_strips_info',        '/adora_robot/chassis/led_strips_info'),
                ('/adora_robot/chassis/remote_control_info',    '/adora_robot/chassis/remote_control_info'),
                ('/adora_robot/chassis/software_version_info',  '/adora_robot/chassis/software_version_info'),
                ('/adora_robot/chassis/current_info',           '/adora_robot/chassis/current_info'),
                ('/adora_robot/chassis/speed_info',             '/adora_robot/chassis/speed_info'),
                ('/adora_robot/chassis/odom_info',              '/adora_robot/chassis/odom_info'),

                # sub: old, new
                ('/dt/velocity_ctrl',                    '/adora_robot/chassis/velocity_ctrl'),
                ('/adora_robot/chassis/speed_ctrl',          '/adora_robot/chassis/speed_ctrl'),
                ('/adora_robot/chassis/stop_ctrl',           '/adora_robot/chassis/stop_ctrl'),
                ('/adora_robot/chassis/collision_clean',     '/adora_robot/chassis/collision_clean'),
                ('/adora_robot/chassis/fault_clean',         '/adora_robot/chassis/fault_clean'),
                ('/adora_robot/chassis/auto_charge_ctrl',    '/adora_robot/chassis/auto_charge_ctrl'),
                ('/adora_robot/chassis/led_strip_mode_ctrl', '/adora_robot/chassis/led_strip_mode_ctrl'),
                ('/adora_robot/chassis/led_strip_ctrl',      '/adora_robot/chassis/led_strip_ctrl'),
                ('/adora_robot/chassis/led_strips_ctrl',     '/adora_robot/chassis/led_strips_ctrl'),
                ('/adora_robot/chassis/odom_clean',          '/adora_robot/chassis/odom_clean'),
            ]
        )
    ])
