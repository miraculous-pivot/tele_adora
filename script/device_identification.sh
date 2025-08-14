#!/bin/bash

# 设备识别和配置脚本
# 用于识别串口设备并保存配置映射

DEVICE_CONFIG_FILE="/home/feng/tele_adora/config/device_mapping.txt"

# 创建配置目录
mkdir -p "$(dirname "$DEVICE_CONFIG_FILE")"

echo "===== 设备识别和配置脚本 ====="
echo ""

# 函数：显示当前连接的串口设备
show_current_devices() {
    echo "当前连接的串口设备："
    echo "===================="
    
    if [ -d "/dev/serial/by-id" ]; then
        ls -la /dev/serial/by-id/ 2>/dev/null | grep -v "^total" | while read line; do
            if [[ $line =~ "usb-" ]]; then
                device_id=$(echo "$line" | awk '{print $9}')
                device_link=$(echo "$line" | awk '{print $11}')
                echo "设备ID: $device_id"
                echo "  -> 映射到: $device_link"
                echo ""
            fi
        done
    else
        echo "未找到 /dev/serial/by-id 目录"
    fi
}

# 函数：交互式设备识别
identify_device() {
    local device_name="$1"
    local device_desc="$2"
    
    echo "==============================================="
    echo "正在识别设备: $device_desc"
    echo "==============================================="
    
    echo "1. 请先拔掉 $device_desc 的USB连接"
    read -p "拔掉后按回车继续..."
    
    echo ""
    echo "拔掉后的设备列表："
    devices_before=$(ls /dev/serial/by-id/ 2>/dev/null || echo "")
    show_current_devices
    
    echo ""
    echo "2. 现在请插入 $device_desc 的USB连接"
    read -p "插入后按回车继续..."
    
    echo ""
    echo "插入后的设备列表："
    devices_after=$(ls /dev/serial/by-id/ 2>/dev/null || echo "")
    show_current_devices
    
    # 找出新增的设备
    new_device=""
    if [ -n "$devices_after" ]; then
        for device in $devices_after; do
            if [[ ! "$devices_before" =~ "$device" ]]; then
                new_device="$device"
                break
            fi
        done
    fi
    
    if [ -n "$new_device" ]; then
        echo ""
        echo "✓ 检测到新设备: $new_device"
        echo "$device_name=/dev/serial/by-id/$new_device" >> "$DEVICE_CONFIG_FILE"
        echo "已保存设备映射: $device_name -> $new_device"
    else
        echo ""
        echo "❌ 未检测到新设备，请重试或手动输入设备ID"
        echo "可用设备列表："
        show_current_devices
        read -p "请输入设备ID (例如: usb-1a86_USB_Single_Serial_594C021229-if00): " manual_device
        if [ -n "$manual_device" ]; then
            echo "$device_name=/dev/serial/by-id/$manual_device" >> "$DEVICE_CONFIG_FILE"
            echo "已手动保存设备映射: $device_name -> $manual_device"
        fi
    fi
    
    echo ""
}

# 函数：检查现有配置
check_existing_config() {
    if [ -f "$DEVICE_CONFIG_FILE" ]; then
        echo "发现现有设备配置文件："
        echo "========================"
        cat "$DEVICE_CONFIG_FILE"
        echo "========================"
        echo ""
        
        while true; do
            read -p "是否要重新配置所有设备? (y/N): " choice
            case "$choice" in
                [Yy]|[Yy][Ee][Ss])
                    return 1  # 重新配置
                    ;;
                [Nn]|[Nn][Oo]|"")
                    return 0  # 使用现有配置
                    ;;
                *)
                    echo "请输入 y 或 n"
                    ;;
            esac
        done
    else
        return 1  # 需要配置
    fi
}

# 函数：询问是否配置吸盘
ask_suction_pump_config() {
    echo "==============================================="
    echo "吸盘配置选择"
    echo "==============================================="
    
    while true; do
        read -p "机器人是否配备吸盘? (y/N): " choice
        case "$choice" in
            [Yy]|[Yy][Ee][Ss])
                echo "SUCTION_PUMP_ENABLED=true" >> "$DEVICE_CONFIG_FILE"
                echo "已启用吸盘配置"
                return 0  # 配置吸盘
                ;;
            [Nn]|[Nn][Oo]|"")
                echo "SUCTION_PUMP_ENABLED=false" >> "$DEVICE_CONFIG_FILE"
                echo "已禁用吸盘配置"
                return 1  # 不配置吸盘
                ;;
            *)
                echo "请输入 y 或 n"
                ;;
        esac
    done
}

# 主程序开始
echo "设备识别向导将帮助您配置机器人的串口设备"
echo ""

# 检查现有配置
if check_existing_config; then
    echo "使用现有配置，设备识别完成。"
    exit 0
fi

# 清空或创建配置文件
> "$DEVICE_CONFIG_FILE"
echo "# 设备映射配置文件" >> "$DEVICE_CONFIG_FILE"
echo "# 自动生成于 $(date)" >> "$DEVICE_CONFIG_FILE"
echo "" >> "$DEVICE_CONFIG_FILE"

echo "开始设备识别流程..."
echo ""

# 显示当前设备状态
echo "当前连接的所有设备："
show_current_devices

echo ""
echo "请确保所有需要识别的设备都已连接，然后我们开始逐个识别。"
read -p "准备好后按回车开始..."

# 识别各个设备
identify_device "HEAD_CONTROL_PORT" "云台控制设备"
identify_device "CHASSIS_CONTROL_PORT" "底盘控制设备" 
identify_device "LIFTING_MOTOR_PORT" "升降电机控制设备"

# 询问是否配置吸盘
if ask_suction_pump_config; then
    identify_device "SUCTION_PUMP_PORT" "吸盘控制设备"
fi

echo ""
echo "==============================================="
echo "设备识别完成！"
echo "==============================================="
echo "配置文件已保存到: $DEVICE_CONFIG_FILE"
echo ""
echo "最终配置："
cat "$DEVICE_CONFIG_FILE"
echo ""
echo "您现在可以使用键盘或VR控制脚本启动机器人。"
