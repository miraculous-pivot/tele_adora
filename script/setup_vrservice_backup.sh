#!/bin/bash
cd ..
echo "这是VR控制方式的服务端"

# 参数控制：是否启动终端窗口
# 用法: bash setup_vrservice.sh --with-terminals
# 默认不启动终端窗口
LAUNCH_TERMINALS=false
if [ "$1" = "--with-terminals" ] || [ "$1" = "-t" ]; then
    LAUNCH_TERMINALS=true
    echo "模式: 启动独立终端窗口"
else
    echo "模式: 后台运行 (使用 --with-terminals 或 -t 参数启动终端窗口)"
fi

# 修复 D-Bus 相关问题
echo "=== 初始化环境设置 ==="
export DISPLAY=${DISPLAY:-:0}

# 检查并设置 D-Bus 会话地址
if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    USER_ID=$(id -u)
    DBUS_PATH="/run/user/$USER_ID/bus"
    
    if [ -S "$DBUS_PATH" ]; then
        export DBUS_SESSION_BUS_ADDRESS="unix:path=$DBUS_PATH"
        echo "已设置 DBUS_SESSION_BUS_ADDRESS"
    else
        echo "警告: D-Bus 套接字文件不存在，尝试启动 D-Bus 会话..."
        if command -v dbus-launch >/dev/null 2>&1; then
            eval $(dbus-launch --sh-syntax)
            echo "已启动新的 D-Bus 会话"
        fi
    fi
fi

echo ""
echo "=== 一次性权限设置 (需要输入sudo密码) ==="
echo "正在设置所有设备权限，只需要输入一次密码..."

# 统一设置所有设备权限，只需要输入一次sudo密码
sudo bash -c "
    # 设置串口设备权限
    if [ -e /dev/serial/by-id/usb-1a86_USB_Single_Serial_56D0001775-if00 ]; then
        chmod 777 /dev/serial/by-id/usb-1a86_USB_Single_Serial_56D0001775-if00
        echo '✓ 提升电机串口权限已设置'
    else
        echo '⚠️  提升电机串口设备未找到'
    fi
    
    if [ -e /dev/serial/by-id/usb-1a86_USB_Single_Serial_594C027770-if00 ]; then
        chmod 777 /dev/serial/by-id/usb-1a86_USB_Single_Serial_594C027770-if00
        echo '✓ 底盘控制串口权限已设置'
    else
        echo '⚠️  底盘控制串口设备未找到'
    fi
    
    if [ -e /dev/serial/by-id/usb-1a86_USB_Single_Serial_594C021229-if00 ]; then
        chmod 777 /dev/serial/by-id/usb-1a86_USB_Single_Serial_594C021229-if00
        echo '✓ 云台控制串口权限已设置'
    else
        echo '⚠️  云台控制串口设备未找到'
    fi
"

# 设置脚本权限（使用当前用户权限）
echo ""
echo "=== 设置脚本权限 ==="
chmod +x ./slave/VR_service/script/init.sh 2>/dev/null && echo "✓ VR_service init.sh 权限已设置" || echo "⚠️  VR_service init.sh 权限设置失败"
chmod +x ./slave/video/webrtc_pub/start_multi_camera.sh 2>/dev/null && echo "✓ webrtc 启动脚本权限已设置" || echo "⚠️  webrtc 启动脚本权限设置失败"

echo "✅ 所有权限设置完成！"
echo ""

#todo:
#检查环境
#检查依赖
#安装依赖
#构建一些源码安装的包
#id号绑定

# 启动Orbbec摄像头（独立终端）
cd ./slave/video/camera/orbbecsdk
echo "=== 启动Orbbec摄像头 ==="
if [ "$LAUNCH_TERMINALS" = true ]; then
    gnome-terminal --title="ORBBEC CAMERA" --working-directory="$PWD" -- \
      bash -c ". ./install/setup.bash;
               ros2 launch orbbec_camera astra.launch.py;
               exec bash"
else
    echo "启动 Orbbec摄像头 (后台模式)..."
    nohup bash -c "source ./install/setup.bash && ros2 launch orbbec_camera astra.launch.py" > /tmp/orbbec_camera_vr.log 2>&1 &
fi

cd ../../../../


# 在adora目录下统一构建底盘和升降模块
echo "=== 在 adora 目录下统一构建底盘和升降模块 ==="
cd ./slave/adora
colcon build
source install/setup.bash

# 启动提升电机控制模块（独立终端）
if [ "$LAUNCH_TERMINALS" = true ]; then
    gnome-terminal --title="LIFTING MOTOR CONTROL" --working-directory="$PWD" -- \
      bash -c "source install/setup.bash;
               echo '=== 启动 lifting_motor_control 节点 ==='; 
               ros2 launch adora_lifting_motor_control adora_a2_max_ros2_node.py;
               exec bash"
else
    echo "启动 lifting_motor_control 节点 (后台模式)..."
    nohup bash -c "source install/setup.bash && ros2 launch adora_lifting_motor_control adora_a2_max_ros2_node.py" > /tmp/lifting_motor_vr.log 2>&1 &
fi

# 等待2秒确保终端启动完成
sleep 2

# 启动底盘控制模块（独立终端）
gnome-terminal --title="CHASSIS CONTROL" --working-directory="$PWD" -- \
  bash -c "source install/setup.bash;
           echo '=== 启动 chassis_control 节点 ==='; 
           ros2 launch adora_chassis_bringup adora_a2_max_ros2.launch.py;
           exec bash"

# 等待2秒确保终端启动完成
sleep 2

# 启动云台控制模块（独立终端）
gnome-terminal --title="GIMBAL CONTROL" --working-directory="$PWD" -- \
  bash -c "source install/setup.bash;
           echo '=== 启动 gimbal_control 节点 ==='; 
           ros2 run ros2_head_control head_control;
           exec bash"

# 返回工作空间根目录
cd ../..

# 初始化vr_service（当前终端）
echo "=== 初始化arm_service ==="
cd ./slave/VR_service
#这里得交互一下，来确保can0绑定左臂，所以要改init.sh
bash ./script/init.sh

# 启动vr控制节点（独立终端）
gnome-terminal --title="VR CONTROL" --working-directory="$PWD" -- \
  bash -c "source install/setup.bash;
           echo '=== 启动vr控制节点 ===';
           ros2 run vr_arm_sync vr_arm_sync_node;
           exec bash"

# 等待2秒确保终端启动完成
sleep 2

# 启动head控制节点（独立终端）
gnome-terminal --title="HEAD CONTROL" --working-directory="$PWD" -- \
  bash -c "source install/setup.bash;
           echo '=== 启动head控制节点 ===';
           ros2 run head_servo_controller head_servo_controller;
           exec bash"

# 返回工作空间根目录
cd ../../


# 启动RealSense双摄像头（独立终端）
echo "=== 启动RealSense双摄像头 ==="
cd ./slave/video/camera/librealsense
gnome-terminal --title="REALSENSE CAMERAS" --working-directory="$PWD" -- \
  bash -c "source ./install/setup.bash;
           ros2 launch realsense2_camera rs_dual_camera_launch.py serial_no1:=_130322271114 serial_no2:=_130322271127;
           exec bash"

# 等待2秒确保终端启动完成
pwd
sleep 2
cd ../../../../

# 启动ZED摄像头（独立终端）
echo "=== 启动ZED摄像头 ==="
gnome-terminal --title="ZED CAMERA" --working-directory="$PWD" -- \
  bash -c "echo '=== 启动ZED摄像头 ===';
           ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm;
           exec bash"

sleep 2


#启动webrtc
echo "=== 启动webrtc ==="
pwd
cd ./slave/video/webrtc_pub
colcon build 
gnome-terminal --title="WEB RTC" --working-directory="$PWD" -- \
      bash -c "echo '=== 启动WEB RTC ===';
               sourve install/setup.bash;
               bash ./start_multi_camera.sh;"


echo "所有模块已启动完成！"
