#!/bin/bash
cd ..
echo "这是VR控制方式的服务端"

# 参数控制：是否显示终端窗口
# 用法: bash setup_vrservice.sh [--show-terminals]
SHOW_TERMINALS=false
if [ "$1" = "--show-terminals" ] || [ "$1" = "-t" ]; then
    SHOW_TERMINALS=true
    echo "模式: 显示终端窗口"
else
    echo "模式: 后台运行 (使用 --show-terminals 或 -t 参数显示终端窗口)"
fi
echo ""

# 加载设备配置
echo "=== 加载设备配置 ==="
source ./script/load_device_config.sh
if ! load_device_config; then
    echo "设备配置加载失败，请先运行设备识别脚本"
    echo "运行命令: bash script/device_identification.sh"
    exit 1
fi

# 验证设备连接
if ! validate_devices; then
    echo ""
    read -p "部分设备验证失败，是否继续? (y/N): " continue_choice
    case "$continue_choice" in
        [Yy]|[Yy][Ee][Ss])
            echo "继续执行..."
            ;;
        *)
            echo "已取消执行"
            exit 1
            ;;
    esac
fi

echo ""

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

# 函数：根据参数决定启动方式
run_service() {
    local title="$1"
    local working_dir="$2"
    local command="$3"
    
    if [ "$SHOW_TERMINALS" = true ]; then
        # 显示终端窗口
        gnome-terminal --title="$title" --working-directory="$working_dir" -- bash -c "$command"
    else
        # 后台运行，输出到日志文件
        local log_file="/tmp/tele_adora_${title// /_}.log"
        echo "后台启动: $title (日志: $log_file)"
        cd "$working_dir"
        nohup bash -c "$command" > "$log_file" 2>&1 &
        cd - > /dev/null
    fi
}

#todo:
#检查环境
#检查依赖
#安装依赖
#构建一些源码安装的包
#id号绑定

# 启动Orbbec摄像头（独立终端）
cd ./slave/video/camera/orbbecsdk
echo "=== 启动Orbbec摄像头 ==="
run_service "ORBBEC CAMERA" "$PWD" ". ./install/setup.bash; ros2 launch orbbec_camera astra.launch.py; exec bash"

cd ../../../../


# 在adora目录下统一构建底盘和升降模块
echo "=== 在 adora 目录下统一构建底盘和升降模块 ==="
cd ./slave/adora
colcon build
source install/setup.bash

# 启动提升电机控制模块（独立终端）
run_service "LIFTING MOTOR CONTROL" "$PWD" "source install/setup.bash; echo '=== 启动 lifting_motor_control 节点 ==='; ros2 launch adora_lifting_motor_control adora_a2_max_ros2_node.py dev:=$LIFTING_MOTOR_PORT; exec bash"

# 等待2秒确保终端启动完成
sleep 2

# 启动底盘控制模块（独立终端）
run_service "CHASSIS CONTROL" "$PWD" "source install/setup.bash; echo '=== 启动 chassis_control 节点 ==='; ros2 launch adora_chassis_bringup adora_a2_max_ros2.launch.py dt_port:=$CHASSIS_CONTROL_PORT; exec bash"

# 等待2秒确保终端启动完成
sleep 2

# 启动云台控制模块（独立终端）
run_service "GIMBAL CONTROL" "$PWD" "source install/setup.bash; echo '=== 启动 gimbal_control 节点 ==='; export HEAD_CONTROL_PORT='$HEAD_CONTROL_PORT'; ros2 launch ros2_head_control head_control.launch.py port:=\$HEAD_CONTROL_PORT; exec bash"

# 启动吸盘控制模块（如果启用）
if [ "$SUCTION_PUMP_ENABLED" = "true" ]; then
    echo "=== 启动吸盘控制模块 ==="
    sleep 2
    run_service "SUCTION PUMP CONTROL" "$PWD" "source install/setup.bash; echo '=== 启动 suction_pump_control 节点 ==='; export SUCTION_PUMP_PORT='$SUCTION_PUMP_PORT'; ros2 launch adora_suction_pump_control suction_pump_control.launch.py port:=\$SUCTION_PUMP_PORT; exec bash"
else
    echo "=== 跳过吸盘控制模块 (未启用) ==="
fi

# 返回工作空间根目录
cd ../..

# 初始化vr_service（当前终端）
echo "=== 初始化arm_service ==="
cd ./slave/VR_service
#这里得交互一下，来确保can0绑定左臂，所以要改init.sh
bash ./script/init.sh

# 启动vr控制节点（独立终端）
run_service "VR CONTROL" "$PWD" "source install/setup.bash; echo '=== 启动vr控制节点 ==='; ros2 run vr_arm_sync vr_arm_sync_node; exec bash"

# 等待2秒确保终端启动完成
sleep 2

# 启动head控制节点（独立终端）
run_service "HEAD CONTROL" "$PWD" "source install/setup.bash; echo '=== 启动head控制节点 ==='; ros2 run head_servo_controller head_servo_controller; exec bash"

# 返回工作空间根目录
cd ../../


# 启动RealSense双摄像头（独立终端）
echo "=== 启动RealSense双摄像头 ==="
cd ./slave/video/camera/librealsense
run_service "REALSENSE CAMERAS" "$PWD" "source install/setup.bash; ros2 launch realsense2_camera rs_dual_camera_launch.py serial_no1:=_130322271114 serial_no2:=_130322271127; exec bash"

# 等待2秒确保终端启动完成
pwd
sleep 2
cd ../../../../

# 启动ZED摄像头（独立终端）
echo "=== 启动ZED摄像头 ==="
run_service "ZED CAMERA" "$PWD" "echo '=== 启动ZED摄像头 ==='; ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm; exec bash"

sleep 2

#启动webrtc
echo "=== 启动webrtc ==="
pwd
cd ./slave/video/webrtc_pub
colcon build 
run_service "WEB RTC" "$PWD" "echo '=== 启动WEB RTC ==='; sourve install/setup.bash; bash ./start_multi_camera.sh;"


echo "所有模块已启动完成！"
