#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from .MOTOR_MG4010E import MOTOR_MG4010E
import math
import os

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        # 声明参数
        self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C021229-if00')
        self.declare_parameter('baudrate', 115200)
        
        # 获取参数
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        # 初始化电机控制器
        self.get_logger().info(f"Initializing MOTOR_MG4010E controller - Port: {port}, Baudrate: {baudrate}")
        self.app = MOTOR_MG4010E(port=port, baudrate=baudrate)
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Point,
            'adora_robot/head/servo_cmd',
            self.angle_callback,
            10
            
        )
        self.get_logger().info("Motor control node ready")
        
        # 参数
        self.conversion_factor = 57.3 * 10 * 100  # 弧度转控制单位
        
        # 从配置文件加载零点偏移
        self.pitch_zero_offset, self.yaw_zero_offset = self.load_zero_config()
        self.get_logger().info(f"Loaded zero offsets - Pitch: {self.pitch_zero_offset:.2f}°, Yaw: {self.yaw_zero_offset:.2f}°")
    
    def load_zero_config(self):
        """从配置文件加载零点偏移"""
        # 使用相对于当前文件的路径
        current_dir = os.path.dirname(os.path.dirname(__file__))  # 返回到ros2_head_control目录
        config_file = os.path.join(current_dir, "..", "..", "config", "head_zero_config.txt")
        config_file = os.path.abspath(config_file)  # 转换为绝对路径以避免相对路径问题
        
        pitch_offset = 0.0  # 默认pitch零点偏移
        yaw_offset = 180.0  # 默认yaw零点偏移
        
        if os.path.exists(config_file):
            try:
                with open(config_file, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if line.startswith('PITCH_ZERO_OFFSET='):
                            pitch_offset = float(line.split('=', 1)[1])
                        elif line.startswith('YAW_ZERO_OFFSET='):
                            yaw_offset = float(line.split('=', 1)[1])
                            
                self.get_logger().info(f"Zero config loaded from: {config_file}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load zero config: {e}, using defaults")
        else:
            self.get_logger().warn(f"Zero config file not found: {config_file}, using defaults")
        
        return pitch_offset, yaw_offset
    
    def angle_callback(self, msg):
        # 从消息中提取pitch和yaw值，并应用零点偏移
        pitch_value = (msg.x + self.pitch_zero_offset * math.pi / 180) * self.conversion_factor
        yaw_value = (msg.y + self.yaw_zero_offset * math.pi / 180) * self.conversion_factor
        
        self.get_logger().info(
            f"Setting pitch: {int(pitch_value)}, yaw: {int(yaw_value)} (with zero offsets)",
            throttle_duration_sec=1.0  # 限流，每秒最多打印一次
        )
        
        # 控制电机
        try:
            self.app.motor_mg4010e_set_multi_loop_angle_control2(1, int(pitch_value), 30000)  # ID1: 俯仰
            
            self.app.motor_mg4010e_set_multi_loop_angle_control2(2, int(yaw_value), 30000)    # ID2: 偏航
        except Exception as e:
            self.get_logger().error(f"Motor control failed: {str(e)}")
    
    def __del__(self):
        if hasattr(self, 'app'):
            self.app.close()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()