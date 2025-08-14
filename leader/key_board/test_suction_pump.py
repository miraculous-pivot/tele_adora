#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class SuctionPumpTestNode(Node):
    def __init__(self):
        super().__init__('suction_pump_test_node')
        
        # 创建订阅者监听吸盘控制话题
        self.cmd_subscription = self.create_subscription(
            Bool,
            '/adora_robot/suction_pump/cmd',
            self.suction_cmd_callback,
            10
        )
        
        # 创建订阅者监听吸盘状态话题
        self.status_subscription = self.create_subscription(
            Bool,
            '/adora_robot/suction_pump/status',
            self.suction_status_callback,
            10
        )
        
        self.get_logger().info("Suction Pump Test Node started")
        self.get_logger().info("Listening to:")
        self.get_logger().info("  - /adora_robot/suction_pump/cmd (command)")
        self.get_logger().info("  - /adora_robot/suction_pump/status (feedback)")
    
    def suction_cmd_callback(self, msg):
        """吸盘命令回调函数"""
        state = "ON" if msg.data else "OFF"
        self.get_logger().info(f"Received suction pump command: {state}")
    
    def suction_status_callback(self, msg):
        """吸盘状态回调函数"""
        state = "ON" if msg.data else "OFF"
        self.get_logger().info(f"Suction pump status: {state}")

def main(args=None):
    rclpy.init(args=args)
    node = SuctionPumpTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
