#!/usr/bin/env python3
"""
云台零点校准测试脚本
简化版本，用于测试配置文件读写功能
"""

import os
import time

def load_zero_config():
    """加载零点配置"""
    # 使用相对路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_file = os.path.join(script_dir, "config", "head_zero_config.txt")
    pitch_offset = 0.0
    yaw_offset = 180.0
    
    if os.path.exists(config_file):
        try:
            with open(config_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith('PITCH_ZERO_OFFSET='):
                        pitch_offset = float(line.split('=', 1)[1])
                    elif line.startswith('YAW_ZERO_OFFSET='):
                        yaw_offset = float(line.split('=', 1)[1])
            print(f"✅ 配置文件加载成功: {config_file}")
        except Exception as e:
            print(f"❌ 读取配置文件失败: {e}")
    else:
        print(f"⚠️  配置文件不存在: {config_file}")
    
    return pitch_offset, yaw_offset

def save_zero_config(pitch_offset, yaw_offset):
    """保存零点配置"""
    # 使用相对路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_file = os.path.join(script_dir, "config", "head_zero_config.txt")
    
    try:
        os.makedirs(os.path.dirname(config_file), exist_ok=True)
        
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
        
        with open(config_file, 'w') as f:
            f.write(config_content)
        
        print(f"✅ 零点配置已保存到: {config_file}")
        return True
        
    except Exception as e:
        print(f"❌ 保存配置失败: {e}")
        return False

def main():
    print("="*50)
    print("         云台零点配置测试")
    print("="*50)
    
    # 显示当前配置
    print("\n1. 读取当前配置:")
    pitch_offset, yaw_offset = load_zero_config()
    print(f"   Pitch零点偏移: {pitch_offset:.2f}°")
    print(f"   Yaw零点偏移: {yaw_offset:.2f}°")
    
    # 模拟校准过程
    print("\n2. 模拟校准过程:")
    print("   假设当前云台位置为:")
    simulated_pitch = 5.5
    simulated_yaw = 175.0
    print(f"   Pitch: {simulated_pitch:.2f}°")
    print(f"   Yaw: {simulated_yaw:.2f}°")
    
    choice = input(f"\n是否将此位置设为新的零点? (y/N): ").strip().lower()
    if choice in ['y', 'yes']:
        if save_zero_config(simulated_pitch, simulated_yaw):
            print(f"\n✅ 零点校准完成!")
            
            # 验证保存
            print("\n3. 验证保存结果:")
            new_pitch, new_yaw = load_zero_config()
            print(f"   新的Pitch零点偏移: {new_pitch:.2f}°")
            print(f"   新的Yaw零点偏移: {new_yaw:.2f}°")
        else:
            print(f"\n❌ 零点校准失败!")
    else:
        print(f"\n取消校准")

if __name__ == '__main__':
    main()
