#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ChassisControlTestNode(Node):
    def __init__(self):
        super().__init__('chassis_control_test_node')
        
        # 创建订阅者监听底盘控制话题
        self.subscription = self.create_subscription(
            Twist,
            '/dt/velocity_ctrl',
            self.chassis_control_callback,
            10
        )
        
        self.get_logger().info("Chassis Control Test Node started - listening to /dt/velocity_ctrl")
    
    def chassis_control_callback(self, msg):
        """底盘控制回调函数"""
        self.get_logger().info(f"Received chassis command - Linear: {msg.linear.x:.2f} m/s, Angular: {msg.angular.z:.2f} rad/s")

def main(args=None):
    rclpy.init(args=args)
    node = ChassisControlTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
