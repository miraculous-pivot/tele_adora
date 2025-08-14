#!/bin/bash

# 全功能键盘控制启动脚本
# 包含双臂控制、云台控制、底盘控制、升降电机控制

echo "=========================================="
echo "启动全功能键盘控制节点"
echo "包含: 双臂控制 + 云台控制 + 底盘控制 + 吸盘控制 + 升降电机控制"
echo "=========================================="
echo ""

# 检查是否在正确的目录
if [ ! -f "install/setup.bash" ]; then
    echo "错误: 请在键盘控制包的根目录下运行此脚本"
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    echo "正确路径: $script_dir"
    exit 1
fi

# 设置环境
echo "设置ROS2环境..."
source install/setup.bash

echo ""
echo "控制说明:"
echo "=========================================="
echo "双臂控制:"
echo "  左臂: w/s/a/d/q/e/r/f/t/g/y/h + v(夹爪)"
echo "  右臂: i/k/j/l/u/o/p/;/[/]/'/\\ + m(夹爪)"
echo ""
echo "云台控制:"
echo "  1: 抬头    3: 低头"
echo "  5: 左转    7: 右转"
echo ""
echo "底盘控制:"
echo "  8: 前进    2: 后退"
echo "  4: 左转    6: 右转"
echo ""
echo "吸盘控制:"
echo "  c: 切换吸盘开关"
echo ""
echo "升降电机:"
echo "  z: 切换上升   x: 切换下降"
echo ""
echo "退出: 空格键 或 ESC键"
echo "=========================================="
echo ""

# 启动节点
echo "启动键盘控制节点..."
ros2 run my_teleop_pkg bimanual_teleop

echo ""
echo "键盘控制节点已退出"
