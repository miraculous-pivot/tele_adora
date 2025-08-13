#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros2_head_control.msg import MotorStatus
from .MOTOR_MG4010E import MOTOR_MG4010E
import time
import struct

class MotorStatusNode(Node):
    def __init__(self):
        super().__init__('motor_status_node')
        
        # 初始化电机控制器
        self.get_logger().info("Initializing MOTOR_MG4010E status reader")
        self.app = MOTOR_MG4010E(port='/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C021229-if00', baudrate=115200)
        
        # 创建状态发布者
        self.status_publisher = self.create_publisher(
            MotorStatus,
            'motor_status',
            10
        )
        
        # 创建定时器读取状态
        self.timer = self.create_timer(0.5, self.read_and_publish_status)  # 2Hz
        self.get_logger().info("Motor status node ready")
        
        # 转换系数 (控制单位转弧度)
        self.angle_conversion = 1.0 / (57.3 * 10 * 100)  # 控制单位转弧度
        
        # 状态缓存
        self.last_pitch_angle = 0.0
        self.last_yaw_angle = 0.0
        self.last_update_time = time.time()
    
    def parse_motor_response(self, data):
        """解析电机响应数据"""
        if len(data) < 7:
            self.get_logger().warn("Response too short")
            return None
            
        # 打印原始数据用于调试
        hex_str = ' '.join(f"{b:02X}" for b in data)
        self.get_logger().debug(f"Raw response: [{hex_str}]")
        
        # 检查帧头
        if data[0] != 0x01 or data[1] != 0x03 or data[2] != 0x04:
            self.get_logger().warn("Invalid response header")
            return None
            
        # 解析位置数据 (大端序，有符号)
        position_bytes = bytearray([data[5], data[6], data[3], data[4]])
        position = int.from_bytes(position_bytes, 'big', signed=True)
        
        return position
    
    def calculate_velocity(self, current_angle, last_angle, time_delta):
        """计算角速度 (弧度/秒)"""
        if time_delta <= 0:
            return 0.0
        return (current_angle - last_angle) / time_delta
    
    def read_and_publish_status(self):
        status_msg = MotorStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        
        current_time = time.time()
        time_delta = current_time - self.last_update_time
        
        try:
            # 读取俯仰电机状态 (ID=1)
            self.app.motor_mg4010e_read_multi_loop_angle(1)
            time.sleep(0.05)  # 等待响应
            pitch_data = self.app.ser.read_all()
            pitch_position = self.parse_motor_response(pitch_data)
            
            if pitch_position is not None:
                # 转换为弧度
                pitch_angle = pitch_position * self.angle_conversion
                status_msg.pitch_angle = pitch_angle
                
                # 计算速度
                status_msg.pitch_velocity = self.calculate_velocity(
                    pitch_angle, self.last_pitch_angle, time_delta
                )
                self.last_pitch_angle = pitch_angle
            else:
                status_msg.pitch_angle = self.last_pitch_angle
                status_msg.pitch_velocity = 0.0
            
            # 读取偏航电机状态 (ID=2)
            self.app.motor_mg4010e_read_multi_loop_angle(2)
            time.sleep(0.05)  # 等待响应
            yaw_data = self.app.ser.read_all()
            yaw_position = self.parse_motor_response(yaw_data)
            
            if yaw_position is not None:
                # 转换为弧度
                yaw_angle = yaw_position * self.angle_conversion
                status_msg.yaw_angle = yaw_angle
                
                # 计算速度
                status_msg.yaw_velocity = self.calculate_velocity(
                    yaw_angle, self.last_yaw_angle, time_delta
                )
                self.last_yaw_angle = yaw_angle
            else:
                status_msg.yaw_angle = self.last_yaw_angle
                status_msg.yaw_velocity = 0.0
            
            # 发布状态
            self.status_publisher.publish(status_msg)
            self.get_logger().info(
                f"Pitch: {status_msg.pitch_angle:.3f} rad, "
                f"Yaw: {status_msg.yaw_angle:.3f} rad",
                throttle_duration_sec=1.0
            )
            
        except Exception as e:
            self.get_logger().error(f"Failed to read motor status: {str(e)}")
        
        # 更新时间戳
        self.last_update_time = current_time
    
    def __del__(self):
        if hasattr(self, 'app'):
            self.app.motor_exit()

def main(args=None):
    rclpy.init(args=args)
    node = MotorStatusNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()