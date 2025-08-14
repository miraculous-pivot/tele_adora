#!/bin/bash

echo "=== 测试Launch文件参数传递修复 ==="

cd /home/feng/tele_adora

# 加载设备配置
source script/load_device_config.sh
load_device_config

echo ""
echo "当前设备配置:"
echo "LIFTING_MOTOR_PORT: $LIFTING_MOTOR_PORT"
echo "CHASSIS_CONTROL_PORT: $CHASSIS_CONTROL_PORT"

echo ""
echo "=== 测试升降电机Launch文件 ==="
cd slave/adora

# 不要实际启动，只测试参数解析
timeout 5s ros2 launch adora_lifting_motor_control adora_a2_max_ros2_node.py dev:=$LIFTING_MOTOR_PORT --dry-run 2>&1 || echo "测试完成"

echo ""
echo "=== 检查设备是否存在 ==="
if [ -e "$LIFTING_MOTOR_PORT" ]; then
    echo "✅ 升降电机设备存在: $LIFTING_MOTOR_PORT"
else
    echo "❌ 升降电机设备不存在: $LIFTING_MOTOR_PORT"
fi

if [ -e "$CHASSIS_CONTROL_PORT" ]; then
    echo "✅ 底盘控制设备存在: $CHASSIS_CONTROL_PORT"
else
    echo "❌ 底盘控制设备不存在: $CHASSIS_CONTROL_PORT"
fi
