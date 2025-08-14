#!/bin/bash

# D-Bus 修复脚本
echo "=== 修复 D-Bus 环境问题 ==="

# 设置基础环境变量
export DISPLAY=${DISPLAY:-:0}

# 检查并设置 D-Bus 会话地址
if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    USER_ID=$(id -u)
    DBUS_PATH="/run/user/$USER_ID/bus"
    
    if [ -S "$DBUS_PATH" ]; then
        export DBUS_SESSION_BUS_ADDRESS="unix:path=$DBUS_PATH"
        echo "已设置 DBUS_SESSION_BUS_ADDRESS=$DBUS_SESSION_BUS_ADDRESS"
    else
        echo "警告: D-Bus 套接字文件不存在: $DBUS_PATH"
        echo "尝试启动 D-Bus 会话..."
        
        # 尝试启动用户 D-Bus 会话
        if command -v dbus-launch >/dev/null 2>&1; then
            eval $(dbus-launch --sh-syntax)
            echo "已启动新的 D-Bus 会话"
        else
            echo "错误: 无法找到 dbus-launch 命令"
        fi
    fi
else
    echo "D-Bus 环境变量已设置: $DBUS_SESSION_BUS_ADDRESS"
fi

# 检查 gnome-terminal 是否可用
if command -v gnome-terminal >/dev/null 2>&1; then
    echo "gnome-terminal 可用"
    
    # 测试启动一个简单的终端
    echo "测试启动 gnome-terminal..."
    gnome-terminal --title="Test Terminal" -- bash -c "echo 'D-Bus 测试成功'; sleep 2" &
    
    # 等待一下看是否有错误
    sleep 1
    
    if [ $? -eq 0 ]; then
        echo "gnome-terminal 启动成功"
    else
        echo "gnome-terminal 启动失败，建议使用替代方案"
    fi
else
    echo "错误: gnome-terminal 不可用"
    echo "建议安装: sudo apt install gnome-terminal"
fi

echo "=== D-Bus 环境检查完成 ==="
