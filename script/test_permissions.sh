#!/bin/bash

# 测试权限设置功能的脚本

echo "=== 测试一次性权限设置功能 ==="
echo ""

# 模拟设备检测
echo "=== 模拟设备检测 ==="
devices=(
    "/dev/serial/by-id/usb-1a86_USB_Single_Serial_56D0001775-if00"
    "/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C027770-if00" 
    "/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C021229-if00"
)

device_names=(
    "提升电机串口"
    "底盘控制串口"
    "云台控制串口"
)

echo "检查串口设备状态："
for i in "${!devices[@]}"; do
    device="${devices[$i]}"
    name="${device_names[$i]}"
    
    if [ -e "$device" ]; then
        echo "✓ $name: 设备存在 ($device)"
    else
        echo "❌ $name: 设备不存在 ($device)"
    fi
done

echo ""
echo "=== 模拟权限设置 (不实际执行sudo) ==="
echo "如果实际运行，将执行以下命令："
echo ""

for i in "${!devices[@]}"; do
    device="${devices[$i]}"
    name="${device_names[$i]}"
    echo "sudo chmod 777 $device  # $name"
done

echo ""
echo "=== 脚本权限检查 ==="
scripts=(
    "./slave/key_service/arm_service/script/init.sh"
    "./slave/VR_service/script/init.sh"
    "./slave/video/webrtc_pub/start_multi_camera.sh"
)

script_names=(
    "键盘服务 init.sh"
    "VR服务 init.sh"
    "webrtc 启动脚本"
)

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

for i in "${!scripts[@]}"; do
    script="${scripts[$i]}"
    name="${script_names[$i]}"
    
    if [ -f "$script" ]; then
        if [ -x "$script" ]; then
            echo "✓ $name: 存在且可执行"
        else
            echo "⚠️  $name: 存在但不可执行"
        fi
    else
        echo "❌ $name: 文件不存在 ($script)"
    fi
done

echo ""
echo "=== 测试完成 ==="
echo "这个测试脚本展示了优化后的权限设置逻辑"
echo "实际运行时，用户只需要在开始时输入一次sudo密码"
