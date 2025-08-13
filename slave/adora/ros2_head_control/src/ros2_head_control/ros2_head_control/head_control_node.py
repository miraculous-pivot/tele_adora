#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from .MOTOR_MG4010E import MOTOR_MG4010E
import math

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        # 初始化电机控制器
        self.get_logger().info("Initializing MOTOR_MG4010E controller")
        self.app = MOTOR_MG4010E(port='/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C021229-if00', baudrate=115200)
        
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
    
    def angle_callback(self, msg):
        # 从消息中提取pitch和yaw值
        pitch_value = msg.x * self.conversion_factor
        yaw_value = msg.y * self.conversion_factor
        self.get_logger().info(
            f"Setting pitch: {int(pitch_value)}, yaw: {int(yaw_value)}",
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