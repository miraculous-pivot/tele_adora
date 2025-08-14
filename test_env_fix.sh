#!/bin/bash

# 测试环境变量传递修复

echo "=== 测试环境变量传递修复 ==="

# 加载设备配置
source script/load_device_config.sh
load_device_config

echo ""
echo "主脚本中的环境变量:"
echo "LIFTING_MOTOR_PORT=$LIFTING_MOTOR_PORT"
echo "CHASSIS_CONTROL_PORT=$CHASSIS_CONTROL_PORT"
echo "HEAD_CONTROL_PORT=$HEAD_CONTROL_PORT"

echo ""
echo "=== 测试终端中的环境变量传递 ==="

# 模拟修复后的命令
test_cmd="echo '测试升降电机端口传递:'; export LIFTING_MOTOR_PORT='$LIFTING_MOTOR_PORT'; echo \$LIFTING_MOTOR_PORT"

echo "执行命令: $test_cmd"
echo "结果:"
bash -c "$test_cmd"

echo ""
echo "=== 验证设备是否存在 ==="
echo "升降电机设备: $LIFTING_MOTOR_PORT"
if [ -e "$LIFTING_MOTOR_PORT" ]; then
    echo "✅ 设备存在"
    ls -la "$LIFTING_MOTOR_PORT"
else
    echo "❌ 设备不存在"
fi
