#!/bin/bash

# 设备配置加载器
# 用于从配置文件加载设备映射并设置环境变量

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DEVICE_CONFIG_FILE="$PROJECT_ROOT/config/device_mapping.txt"

# 函数：加载设备配置
load_device_config() {
    if [ ! -f "$DEVICE_CONFIG_FILE" ]; then
        echo "错误: 设备配置文件不存在: $DEVICE_CONFIG_FILE"
        echo "请先运行设备识别脚本: bash script/device_identification.sh"
        return 1
    fi
    
    echo "加载设备配置..."
    
    # 设置默认值（只在变量未设置时使用）
    export HEAD_CONTROL_PORT="${HEAD_CONTROL_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C021229-if00}"
    export CHASSIS_CONTROL_PORT="${CHASSIS_CONTROL_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C027770-if00}"
    export LIFTING_MOTOR_PORT="${LIFTING_MOTOR_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_56D0001775-if00}"
    export SUCTION_PUMP_PORT="${SUCTION_PUMP_PORT:-/dev/ttyACM0}"
    export SUCTION_PUMP_ENABLED="${SUCTION_PUMP_ENABLED:-false}"
    
    # 从配置文件加载
    while IFS='=' read -r key value; do
        # 跳过注释和空行
        [[ $key =~ ^#.*$ ]] && continue
        [[ -z $key ]] && continue
        
        # 去除空格
        key=$(echo "$key" | tr -d ' ')
        value=$(echo "$value" | tr -d ' ')
        
        case "$key" in
            HEAD_CONTROL_PORT)
                export HEAD_CONTROL_PORT="$value"
                echo "✓ 头部控制设备: $value"
                ;;
            CHASSIS_CONTROL_PORT)
                export CHASSIS_CONTROL_PORT="$value"
                echo "✓ 底盘控制设备: $value"
                ;;
            LIFTING_MOTOR_PORT)
                export LIFTING_MOTOR_PORT="$value"
                echo "✓ 升降电机设备: $value"
                ;;
            SUCTION_PUMP_PORT)
                export SUCTION_PUMP_PORT="$value"
                echo "✓ 吸盘控制设备: $value"
                ;;
            SUCTION_PUMP_ENABLED)
                export SUCTION_PUMP_ENABLED="$value"
                if [ "$value" = "true" ]; then
                    echo "✓ 吸盘功能: 已启用"
                else
                    echo "✓ 吸盘功能: 已禁用"
                fi
                ;;
        esac
    done < "$DEVICE_CONFIG_FILE"
    
    echo "设备配置加载完成"
    return 0
}

# 函数：验证设备存在性
validate_devices() {
    echo ""
    echo "验证设备连接状态..."
    
    local all_valid=true
    
    # 检查头部控制设备
    if [ -e "$HEAD_CONTROL_PORT" ]; then
        echo "✓ 头部控制设备连接正常: $HEAD_CONTROL_PORT"
    else
        echo "❌ 头部控制设备未找到: $HEAD_CONTROL_PORT"
        all_valid=false
    fi
    
    # 检查底盘控制设备
    if [ -e "$CHASSIS_CONTROL_PORT" ]; then
        echo "✓ 底盘控制设备连接正常: $CHASSIS_CONTROL_PORT"
    else
        echo "❌ 底盘控制设备未找到: $CHASSIS_CONTROL_PORT"
        all_valid=false
    fi
    
    # 检查升降电机设备
    if [ -e "$LIFTING_MOTOR_PORT" ]; then
        echo "✓ 升降电机设备连接正常: $LIFTING_MOTOR_PORT"
    else
        echo "❌ 升降电机设备未找到: $LIFTING_MOTOR_PORT"
        all_valid=false
    fi
    
    # 检查吸盘设备（如果启用）
    if [ "$SUCTION_PUMP_ENABLED" = "true" ]; then
        if [ -e "$SUCTION_PUMP_PORT" ]; then
            echo "✓ 吸盘控制设备连接正常: $SUCTION_PUMP_PORT"
        else
            echo "❌ 吸盘控制设备未找到: $SUCTION_PUMP_PORT"
            all_valid=false
        fi
    else
        echo "ℹ️  吸盘功能已禁用，跳过检查"
    fi
    
    if [ "$all_valid" = true ]; then
        echo ""
        echo "✅ 所有设备验证通过！"
        return 0
    else
        echo ""
        echo "⚠️  部分设备验证失败，请检查设备连接或重新运行设备识别脚本"
        return 1
    fi
}

# 函数：显示设备状态
show_device_status() {
    echo ""
    echo "当前设备配置:"
    echo "=============="
    
    if [ -f "$DEVICE_CONFIG_FILE" ]; then
        # 直接从配置文件读取并显示
        local head_port=""
        local chassis_port=""
        local lifting_port=""
        local suction_port=""
        local suction_enabled="false"
        
        while IFS='=' read -r key value; do
            # 跳过注释和空行
            [[ $key =~ ^#.*$ ]] && continue
            [[ -z $key ]] && continue
            
            # 去除空格
            key=$(echo "$key" | tr -d ' ')
            value=$(echo "$value" | tr -d ' ')
            
            case "$key" in
                HEAD_CONTROL_PORT) head_port="$value" ;;
                CHASSIS_CONTROL_PORT) chassis_port="$value" ;;
                LIFTING_MOTOR_PORT) lifting_port="$value" ;;
                SUCTION_PUMP_PORT) suction_port="$value" ;;
                SUCTION_PUMP_ENABLED) suction_enabled="$value" ;;
            esac
        done < "$DEVICE_CONFIG_FILE"
        
        echo "头部控制设备: $head_port"
        echo "底盘控制设备: $chassis_port"
        echo "升降电机设备: $lifting_port"
        echo "吸盘控制设备: $suction_port"
        echo "吸盘功能状态: $suction_enabled"
    else
        echo "设备配置文件不存在: $DEVICE_CONFIG_FILE"
    fi
    echo ""
}

# 当直接运行此脚本时
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    echo "===== 设备配置加载器 ====="
    
    if load_device_config; then
        show_device_status
        validate_devices
    else
        exit 1
    fi
fi
