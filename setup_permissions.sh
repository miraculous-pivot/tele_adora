#!/bin/bash

# 为Tele-Adora项目中的所有shell脚本添加执行权限
# 这个脚本应该在项目根目录运行
# 用法: bash setup_permissions.sh [--quiet]

# 检查是否为静默模式
QUIET_MODE=false
if [ "$1" = "--quiet" ] || [ "$1" = "-q" ]; then
    QUIET_MODE=true
fi

# 输出函数 - 根据模式决定是否显示
output() {
    if [ "$QUIET_MODE" = false ]; then
        echo "$1"
    fi
}

output "=== Tele-Adora 脚本权限设置工具 ==="
output "正在为所有 .sh 文件添加执行权限..."
output ""

# 函数：设置文件权限并显示结果
set_executable() {
    local file="$1"
    if [ -f "$file" ]; then
        chmod +x "$file"
        if [ "$QUIET_MODE" = false ]; then
            echo "✓ $file"
        fi
        return 0
    else
        if [ "$QUIET_MODE" = false ]; then
            echo "❌ 文件不存在: $file"
        fi
        return 1
    fi
}

# 计数器
success_count=0
error_count=0

output "=== 主要脚本文件 ==="
# 根目录脚本
set_executable "./start.sh" && ((success_count++)) || ((error_count++))

# script目录下的脚本
script_files=(
    "./script/main_setup.sh"
    "./script/setup_keyboard.sh"
    "./script/setup_keyservice.sh"
    "./script/setup_vrservice.sh"
    "./script/setup_vrservice_backup.sh"
    "./script/fix_dbus.sh"
    "./script/test_confirm.sh"
    "./script/test_vr_init.sh"
    "./script/test_terminal_params.sh"
    "./script/device_identification.sh"
    "./script/device_config.sh"
    "./script/load_device_config.sh"
    "./script/demo_device_config.sh"
    "./script/test_start_menu.sh"
    "./script/test_auto_permissions.sh"
    "./script/test_chmod_sharing.sh"
    "./script/test_device_comprehensive.sh"
    "./script/test_device_config.sh"
    "./script/test_launch_files.sh"
    "./script/test_permissions.sh"
)

output ""
output "=== script 目录脚本 ==="
for file in "${script_files[@]}"; do
    set_executable "$file" && ((success_count++)) || ((error_count++))
done

output ""
output "=== leader 目录脚本 ==="
# leader目录脚本
leader_files=(
    "./leader/key_board/start_full_teleop.sh"
)

for file in "${leader_files[@]}"; do
    set_executable "$file" && ((success_count++)) || ((error_count++))
done

output ""
output "=== slave/video 目录脚本 ==="
# slave/video目录脚本
video_files=(
    "./slave/video/webrtc_pub/start_single_camera.sh"
    "./slave/video/webrtc_pub/start_multi_camera.sh"
)

for file in "${video_files[@]}"; do
    set_executable "$file" && ((success_count++)) || ((error_count++))
done

output ""
output "=== slave/key_service 目录脚本 ==="
# slave/key_service目录脚本
key_service_files=(
    "./slave/key_service/arm_service/script/init.sh"
    "./slave/key_service/arm_service/script/can_activate.sh"
    "./slave/key_service/arm_service/script/find_all_can_port.sh"
    "./slave/key_service/arm_service/script/version_check.sh"
)

for file in "${key_service_files[@]}"; do
    set_executable "$file" && ((success_count++)) || ((error_count++))
done

output ""
output "=== slave/VR_service 目录脚本 ==="
# slave/VR_service目录脚本
vr_service_files=(
    "./slave/VR_service/script/init.sh"
    "./slave/VR_service/script/can_activate.sh"
    "./slave/VR_service/script/find_all_can_port.sh"
    "./slave/VR_service/script/version_check.sh"
)

for file in "${vr_service_files[@]}"; do
    set_executable "$file" && ((success_count++)) || ((error_count++))
done

output ""
output "=== 开发工具脚本 ==="
# dev目录脚本
dev_files=(
    "./dev/piper_sdk/rm_tmp.sh"
    "./dev/piper_sdk/piper_sdk/can_activate.sh"
    "./dev/piper_sdk/piper_sdk/can_config.sh"
    "./dev/piper_sdk/piper_sdk/can_find_and_config.sh"
    "./dev/piper_sdk/piper_sdk/can_muti_activate.sh"
    "./dev/piper_sdk/piper_sdk/find_all_can_port.sh"
)

for file in "${dev_files[@]}"; do
    if [ -f "$file" ]; then
        set_executable "$file" && ((success_count++)) || ((error_count++))
    fi
done

output ""
output "=== 云台校准脚本 ==="
# 云台校准相关脚本
head_calibration_files=(
    "./slave/adora/ros2_head_control/head_zero_calibration.py"
    "./slave/adora/ros2_head_control/head_calibration_test.py"
)

for file in "${head_calibration_files[@]}"; do
    if [ -f "$file" ]; then
        set_executable "$file" && ((success_count++)) || ((error_count++))
    fi
done

output ""
output "=== 权限设置完成 ==="
output "成功: $success_count 个文件"
output "失败: $error_count 个文件"
output ""

if [ $error_count -eq 0 ]; then
    output "🎉 所有脚本权限设置成功！"
else
    output "⚠️  有 $error_count 个文件设置失败，请检查文件是否存在"
fi

output ""
output "现在您可以运行: bash ./start.sh 来启动系统"
