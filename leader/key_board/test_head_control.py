#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class HeadControlTestNode(Node):
    def __init__(self):
        super().__init__('head_control_test_node')
        
        # 创建订阅者监听云台控制话题
        self.subscription = self.create_subscription(
            Point,
            'adora_robot/head/servo_cmd',
            self.head_control_callback,
            10
        )
        
        self.get_logger().info("Head Control Test Node started - listening to adora_robot/head/servo_cmd")
    
    def head_control_callback(self, msg):
        """云台控制回调函数"""
        self.get_logger().info(f"Received head command - Pitch: {msg.x:.3f}, Yaw: {msg.y:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = HeadControlTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
