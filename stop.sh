#!/bin/bash

# Tele-Adora 快速关闭脚本
# 用于快速停止所有后台运行的服务
# 用法: bash stop.sh

echo "🛑 正在停止 Tele-Adora 系统..."
echo ""

# 停止ROS2节点
echo "停止 ROS2 节点..."
pkill -f "ros2 launch" 2>/dev/null || true
pkill -f "ros2 run" 2>/dev/null || true

# 停止后台进程
echo "停止后台进程..."
pkill -f "nohup.*ros2" 2>/dev/null || true
pkill -f "nohup.*tele_adora" 2>/dev/null || true

# 关闭终端窗口
echo "关闭终端窗口..."
pkill -f "gnome-terminal.*LIFTING" 2>/dev/null || true
pkill -f "gnome-terminal.*CHASSIS" 2>/dev/null || true
pkill -f "gnome-terminal.*GIMBAL" 2>/dev/null || true
pkill -f "gnome-terminal.*ARM" 2>/dev/null || true
pkill -f "gnome-terminal.*CAMERA" 2>/dev/null || true
pkill -f "gnome-terminal.*VR" 2>/dev/null || true
pkill -f "gnome-terminal.*HEAD" 2>/dev/null || true
pkill -f "gnome-terminal.*PUMP" 2>/dev/null || true
pkill -f "gnome-terminal.*WEB" 2>/dev/null || true

# 停止colcon构建进程
echo "停止构建进程..."
pkill -f "colcon" 2>/dev/null || true

# 等待进程完全停止
echo "等待进程停止..."
sleep 3

# 检查剩余进程
remaining=$(ps aux | grep -E "(ros2|colcon|adora)" | grep -v grep | grep -v stop.sh || true)
if [ -n "$remaining" ]; then
    echo ""
    echo "⚠️  仍有以下进程运行中："
    echo "$remaining"
    echo ""
    echo "如需强制终止，请运行: bash service_manager.sh kill-all"
else
    echo ""
    echo "✅ 所有 Tele-Adora 服务已停止"
fi

echo ""
echo "关闭完成！"
