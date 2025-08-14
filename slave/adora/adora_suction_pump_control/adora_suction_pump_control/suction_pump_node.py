#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from .SuctionPumpController import SuctionPumpController

class SuctionPumpControlNode(Node):
    def __init__(self):
        super().__init__('suction_pump_control_node')
        
        # 声明参数
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('channel', 0)
        
        # 获取参数
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.channel = self.get_parameter('channel').value
        
        # 初始化吸盘控制器
        self.controller = None
        try:
            self.controller = SuctionPumpController(port=port, baudrate=baudrate)
            self.get_logger().info(f"Suction pump controller initialized - Port: {port}, Baudrate: {baudrate}, Channel: {self.channel}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize suction pump controller: {e}")
            self.controller = None
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Bool,
            '/adora_robot/suction_pump/cmd',
            self.suction_pump_callback,
            10
        )
        
        # 创建发布者用于状态反馈
        self.status_pub = self.create_publisher(Bool, '/adora_robot/suction_pump/status', 10)
        
        # 当前状态
        self.current_state = False
        
        self.get_logger().info("Suction Pump Control Node started - listening to /adora_robot/suction_pump/cmd")
    
    def suction_pump_callback(self, msg):
        """吸盘控制回调函数"""
        if not self.controller:
            self.get_logger().warn("Suction pump controller not available")
            return
        
        try:
            if msg.data and not self.current_state:
                # 开启吸盘
                self.controller.open_channel(self.channel)
                self.current_state = True
                self.get_logger().info(f"Suction pump channel {self.channel} turned ON")
                
            elif not msg.data and self.current_state:
                # 关闭吸盘
                self.controller.close_channel(self.channel)
                self.current_state = False
                self.get_logger().info(f"Suction pump channel {self.channel} turned OFF")
            
            # 发布状态反馈
            status_msg = Bool()
            status_msg.data = self.current_state
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to control suction pump: {e}")
    
    def __del__(self):
        """析构函数，确保安全关闭"""
        if self.controller:
            try:
                # 关闭吸盘
                self.controller.close_channel(self.channel)
                # 关闭串口
                self.controller.close()
                self.get_logger().info("Suction pump controller safely closed")
            except Exception as e:
                self.get_logger().error(f"Error closing suction pump controller: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SuctionPumpControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
