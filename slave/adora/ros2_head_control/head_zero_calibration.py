#!/usr/bin/env python3
"""
head_zero_calibration.py - 云台调零校准工具
提供完整的云台硬件交互和校准功能
"""

import time
import os
import sys
import argparse
from typing import Tuple, Optional

# 使用相对路径添加模块路径
script_dir = os.path.dirname(os.path.abspath(__file__))
module_path = os.path.join(script_dir, 'src', 'ros2_head_control', 'ros2_head_control')
sys.path.append(module_path)

try:
    from MOTOR_MG4010E import MOTOR_MG4010E
except ImportError:
    print("警告: 无法导入MOTOR_MG4010E，可能需要检查模块路径")
    MOTOR_MG4010E = None

class HeadZeroReader:
    def __init__(self, port=None):
        # 使用相对路径
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(script_dir, "config", "head_zero_config.txt")
        
        # 如果没有指定端口，尝试从设备配置中读取
        if port is None:
            port = self.get_head_control_port()
        
        self.motor = None
        self.port = port
        
    def get_head_control_port(self):
        """从设备配置文件中获取云台控制端口"""
        # 使用相对路径找到项目根目录的配置文件
        script_dir = os.path.dirname(os.path.abspath(__file__))
        device_config = os.path.join(script_dir, "..", "..", "..", "config", "device_mapping.txt")
        default_port = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C021229-if00"
        
        if os.path.exists(device_config):
            try:
                with open(device_config, 'r') as f:
                    for line in f:
                        if line.startswith('HEAD_CONTROL_PORT='):
                            return line.split('=', 1)[1].strip()
            except Exception as e:
                print(f"读取设备配置失败: {e}")
        
        print(f"使用默认端口: {default_port}")
        return default_port
    
    def connect(self):
        """连接到云台控制器"""
        try:
            print(f"正在连接云台控制器: {self.port}")
            self.motor = MOTOR_MG4010E(port=self.port, baudrate=115200)
            print("✅ 云台控制器连接成功")
            return True
        except Exception as e:
            print(f"❌ 云台控制器连接失败: {e}")
            return False
    
    def read_current_position(self):
        """读取当前位置"""
        if not self.motor:
            print("❌ 云台控制器未连接")
            return None, None
            
        try:
            print("正在读取当前位置...")
            
            # 清空缓冲区
            if hasattr(self.motor, '_ser') and self.motor._ser:
                self.motor._ser.reset_input_buffer()
            
            # 多次读取取平均值
            pitch_readings = []
            yaw_readings = []
            
            for i in range(5):  # 读取5次
                self.motor.run()
                time.sleep(0.2)
                
                angles = self.motor.get_current_angles()
                if angles['pitch'] != 0 or angles['yaw'] != 0:  # 有效读取
                    pitch_readings.append(angles['pitch'])
                    yaw_readings.append(angles['yaw'])
                    print(f"读取 {i+1}/5: Pitch={angles['pitch']:.2f}°, Yaw={angles['yaw']:.2f}°")
            
            if pitch_readings and yaw_readings:
                avg_pitch = sum(pitch_readings) / len(pitch_readings)
                avg_yaw = sum(yaw_readings) / len(yaw_readings)
                print(f"\n平均值: Pitch={avg_pitch:.2f}°, Yaw={avg_yaw:.2f}°")
                return avg_pitch, avg_yaw
            else:
                print("❌ 未能读取到有效数据")
                return None, None
                
        except Exception as e:
            print(f"❌ 读取位置失败: {e}")
            return None, None
    
    def load_current_config(self):
        """加载当前零点配置"""
        pitch_offset = 0.0
        yaw_offset = 180.0
        
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if line.startswith('PITCH_ZERO_OFFSET='):
                            pitch_offset = float(line.split('=', 1)[1])
                        elif line.startswith('YAW_ZERO_OFFSET='):
                            yaw_offset = float(line.split('=', 1)[1])
            except Exception as e:
                print(f"读取配置文件失败: {e}")
        
        return pitch_offset, yaw_offset
    
    def save_zero_config(self, pitch_offset, yaw_offset):
        """保存零点配置"""
        try:
            os.makedirs(os.path.dirname(self.config_file), exist_ok=True)
            
            config_content = f"""# 云台零点配置文件
# 自动生成于 {time.strftime('%Y年%m月%d日 %H:%M:%S')}
# 
# 格式说明:
# PITCH_ZERO_OFFSET=<pitch轴零点偏移值，单位：度>
# YAW_ZERO_OFFSET=<yaw轴零点偏移值，单位：度>
#
PITCH_ZERO_OFFSET={pitch_offset:.2f}
YAW_ZERO_OFFSET={yaw_offset:.2f}
"""
            
            with open(self.config_file, 'w') as f:
                f.write(config_content)
            
            print(f"✅ 零点配置已保存到: {self.config_file}")
            return True
            
        except Exception as e:
            print(f"❌ 保存配置失败: {e}")
            return False
    
    def interactive_calibration(self):
        """交互式零点校准"""
        print("\n" + "="*50)
        print("         云台零点校准向导")
        print("="*50)
        
        # 显示当前配置
        current_pitch, current_yaw = self.load_current_config()
        print(f"\n当前零点配置:")
        print(f"  Pitch零点偏移: {current_pitch:.2f}°")
        print(f"  Yaw零点偏移: {current_yaw:.2f}°")
        
        # 连接设备
        if not self.connect():
            return False
        
        print(f"\n请将云台调整到你希望的零点位置")
        print(f"- Pitch轴（俯仰）: 通常为水平位置")
        print(f"- Yaw轴（偏航）: 通常为正前方")
        print(f"\n调整完成后按回车继续...")
        input()
        
        # 读取当前位置
        pitch_pos, yaw_pos = self.read_current_position()
        if pitch_pos is None or yaw_pos is None:
            print("❌ 无法读取当前位置，校准失败")
            return False
        
        print(f"\n当前位置读取完成:")
        print(f"  Pitch: {pitch_pos:.2f}°")
        print(f"  Yaw: {yaw_pos:.2f}°")
        
        # 确认保存
        while True:
            choice = input(f"\n是否将当前位置设置为零点? (y/N): ").strip().lower()
            if choice in ['y', 'yes']:
                # 将当前位置设为零点，意味着偏移量就是当前读取值
                if self.save_zero_config(pitch_pos, yaw_pos):
                    print(f"\n✅ 零点校准完成!")
                    print(f"新的零点配置:")
                    print(f"  Pitch零点偏移: {pitch_pos:.2f}°")
                    print(f"  Yaw零点偏移: {yaw_pos:.2f}°")
                    return True
                else:
                    return False
            elif choice in ['n', 'no', '']:
                print("取消校准")
                return False
            else:
                print("请输入 y 或 n")
    
    def show_current_status(self):
        """显示当前状态"""
        print("\n" + "="*50)
        print("         云台当前状态")
        print("="*50)
        
        # 显示配置
        pitch_offset, yaw_offset = self.load_current_config()
        print(f"\n当前零点配置:")
        print(f"  Pitch零点偏移: {pitch_offset:.2f}°")
        print(f"  Yaw零点偏移: {yaw_offset:.2f}°")
        
        # 连接并读取位置
        if self.connect():
            pitch_pos, yaw_pos = self.read_current_position()
            if pitch_pos is not None and yaw_pos is not None:
                print(f"\n当前实际位置:")
                print(f"  Pitch: {pitch_pos:.2f}°")
                print(f"  Yaw: {yaw_pos:.2f}°")
                
                print(f"\n相对零点的位置:")
                print(f"  Pitch相对位置: {pitch_pos - pitch_offset:.2f}°")
                print(f"  Yaw相对位置: {yaw_pos - yaw_offset:.2f}°")
            else:
                print(f"\n❌ 无法读取当前位置")
        else:
            print(f"\n❌ 无法连接到云台控制器")
    
    def close(self):
        """关闭连接"""
        if self.motor:
            try:
                self.motor.close()
            except:
                pass

def main():
    parser = argparse.ArgumentParser(description='云台零点读取和校准工具')
    parser.add_argument('--port', help='指定云台控制端口')
    parser.add_argument('--calibrate', action='store_true', help='启动交互式校准')
    parser.add_argument('--status', action='store_true', help='显示当前状态')
    
    args = parser.parse_args()
    
    reader = HeadZeroReader(port=args.port)
    
    try:
        if args.calibrate:
            reader.interactive_calibration()
        elif args.status:
            reader.show_current_status()
        else:
            # 默认行为：显示状态
            reader.show_current_status()
    
    finally:
        reader.close()

if __name__ == '__main__':
    main()
