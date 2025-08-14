#!/bin/bash

# 设备配置系统演示脚本
# 演示完整的设备识别和配置流程

echo "===== 设备配置系统演示 ====="
echo ""

echo "本演示将展示如何使用新的设备配置系统："
echo "1. 设备识别和映射"
echo "2. 配置加载和验证"
echo "3. 参数化启动ROS2节点"
echo "4. 吸盘功能控制"
echo ""

# 显示当前连接的设备
echo "=== 当前连接的串口设备 ==="
if [ -d "/dev/serial/by-id" ]; then
    ls -la /dev/serial/by-id/ 2>/dev/null | grep -E "(usb-1a86|usb-Arduino)" || echo "未发现常见设备"
else
    echo "未找到 /dev/serial/by-id 目录"
fi
echo ""

# 演示配置文件功能
echo "=== 演示配置文件功能 ==="
echo "配置文件路径: /home/feng/tele_adora/config/device_mapping.txt"
echo ""

if [ -f "/home/feng/tele_adora/config/device_mapping.txt" ]; then
    echo "✓ 现有配置文件内容："
    echo "========================"
    cat /home/feng/tele_adora/config/device_mapping.txt
    echo "========================"
else
    echo "❌ 配置文件不存在"
    echo "要创建配置文件，请运行："
    echo "bash script/device_identification.sh"
fi
echo ""

# 演示配置加载
echo "=== 演示配置加载 ==="
source /home/feng/tele_adora/script/load_device_config.sh
if load_device_config 2>/dev/null; then
    echo "✓ 配置加载成功！"
    echo ""
    echo "已加载的环境变量："
    echo "HEAD_CONTROL_PORT: $HEAD_CONTROL_PORT"
    echo "CHASSIS_CONTROL_PORT: $CHASSIS_CONTROL_PORT"  
    echo "LIFTING_MOTOR_PORT: $LIFTING_MOTOR_PORT"
    echo "SUCTION_PUMP_PORT: $SUCTION_PUMP_PORT"
    echo "SUCTION_PUMP_ENABLED: $SUCTION_PUMP_ENABLED"
else
    echo "❌ 配置加载失败"
fi
echo ""

# 演示ROS2启动命令
echo "=== 演示ROS2启动命令 ==="
echo "基于加载的配置，以下是ROS2启动命令示例："
echo ""

if [ -n "$HEAD_CONTROL_PORT" ]; then
    echo "头部控制："
    echo "ros2 launch ros2_head_control head_control.launch.py port:=$HEAD_CONTROL_PORT"
    echo ""
fi

if [ -n "$CHASSIS_CONTROL_PORT" ]; then
    echo "底盘控制："
    echo "ros2 launch adora_chassis_bringup adora_a2_max_ros2.launch.py dt_port:=$CHASSIS_CONTROL_PORT"
    echo ""
fi

if [ -n "$LIFTING_MOTOR_PORT" ]; then
    echo "升降电机："
    echo "ros2 launch adora_lifting_motor_control adora_a2_max_ros2_node.py dev:=$LIFTING_MOTOR_PORT"
    echo ""
fi

if [ "$SUCTION_PUMP_ENABLED" = "true" ] && [ -n "$SUCTION_PUMP_PORT" ]; then
    echo "吸盘控制（已启用）："
    echo "ros2 launch adora_suction_pump_control suction_pump_control.launch.py port:=$SUCTION_PUMP_PORT"
    echo ""
elif [ "$SUCTION_PUMP_ENABLED" = "false" ]; then
    echo "吸盘控制: 已禁用，跳过启动"
    echo ""
fi

# 演示启动脚本使用
echo "=== 演示启动脚本使用 ==="
echo "配置完成后，您可以直接使用以下命令启动系统："
echo ""
echo "键盘控制模式:"
echo "bash script/setup_keyservice.sh"
echo ""
echo "VR控制模式:"
echo "bash script/setup_vrservice.sh" 
echo ""
echo "显示终端窗口模式:"
echo "bash script/setup_keyservice.sh --show-terminals"
echo "bash script/setup_vrservice.sh --show-terminals"
echo ""

# 提供使用建议
echo "=== 使用建议 ==="
echo "1. 首次使用前运行设备识别: bash script/device_identification.sh"
echo "2. 验证配置正确性: bash script/test_device_config.sh"
echo "3. 设备更换后重新运行设备识别脚本"
echo "4. 定期备份配置文件: cp config/device_mapping.txt config/device_mapping.txt.backup"
echo ""

echo "演示完成！"
