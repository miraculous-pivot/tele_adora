#!/bin/bash

# 交互式CAN接口激活与版本检查脚本

# 设置错误处理
set -e

echo "===== 交互式CAN接口管理脚本 ====="
echo ""

# 函数：检测CAN接口
detect_can_ports() {
    echo "正在扫描CAN接口..."
    local output
    output=$(bash script/find_all_can_port.sh)
    
    # 解析输出并存储结果
    DETECTED_INTERFACES=()
    DETECTED_USB_ADDRESSES=()
    
    while IFS= read -r line; do
        if [[ $line =~ "Interface "(can[0-9]+)" is connected to USB port "([^ ]+) ]]; then
            DETECTED_INTERFACES+=("${BASH_REMATCH[1]}")
            DETECTED_USB_ADDRESSES+=("${BASH_REMATCH[2]}")
            echo "发现: ${BASH_REMATCH[1]} -> USB ${BASH_REMATCH[2]}"
        fi
    done <<< "$output"
    
    echo "扫描完成，检测到 ${#DETECTED_INTERFACES[@]} 个CAN接口"
}

# 函数：等待用户确认
wait_for_user_confirmation() {
    local message="$1"
    echo "$message"
    read -p "按回车键继续..." -r
    echo ""
}

# 函数：等待检测到指定数量的CAN接口
wait_for_can_interfaces() {
    local expected_count="$1"
    local device_name="$2"
    
    echo "正在等待检测到 $device_name..."
    
    while true; do
        detect_can_ports
        local current_count=${#DETECTED_INTERFACES[@]}
        
        if [ "$current_count" -eq "$expected_count" ]; then
            echo "✓ 检测到 $expected_count 个CAN接口"
            for i in "${!DETECTED_INTERFACES[@]}"; do
                echo "  - ${DETECTED_INTERFACES[$i]} (USB: ${DETECTED_USB_ADDRESSES[$i]})"
            done
            echo ""
            break
        elif [ "$current_count" -gt "$expected_count" ]; then
            echo "⚠️  检测到 $current_count 个CAN接口，预期 $expected_count 个"
            echo "请检查USB连接..."
            sleep 2
        else
            echo "等待中... (当前检测到 $current_count 个接口，需要 $expected_count 个)"
            sleep 2
        fi
    done
}

# 初始检查
echo "步骤1: 初始化检查..."
detect_can_ports
initial_count=${#DETECTED_INTERFACES[@]}

if [ "$initial_count" -gt 0 ]; then
    echo "⚠️  当前检测到 $initial_count 个CAN接口已连接："
    for i in "${!DETECTED_INTERFACES[@]}"; do
        echo "  - ${DETECTED_INTERFACES[$i]} (USB: ${DETECTED_USB_ADDRESSES[$i]})"
    done
    echo ""
    wait_for_user_confirmation "请先拔掉所有USB转CAN设备，然后按回车键继续..."
    
    # 等待所有设备拔掉
    while true; do
        detect_can_ports
        if [ ${#DETECTED_INTERFACES[@]} -eq 0 ]; then
            echo "✓ 所有CAN接口已断开"
            break
        else
            echo "仍检测到 ${#DETECTED_INTERFACES[@]} 个CAN接口，请确保所有设备已拔掉..."
            sleep 2
        fi
    done
else
    echo "✓ 当前未检测到CAN接口"
fi

echo ""
echo "步骤2: 配置左机械臂 (can0)..."
wait_for_user_confirmation "请插入左机械臂的USB转CAN设备，然后按回车键继续..."

# 等待检测到左机械臂设备
wait_for_can_interfaces 1 "左机械臂设备"

# 保存左机械臂信息
LEFT_ARM_INTERFACE="${DETECTED_INTERFACES[0]}"
LEFT_ARM_USB="${DETECTED_USB_ADDRESSES[0]}"

echo "步骤3: 激活左机械臂接口..."
echo "激活左机械臂: $LEFT_ARM_INTERFACE -> can0 (USB: $LEFT_ARM_USB)"
bash ./script/can_activate.sh can0 1000000 "$LEFT_ARM_USB"
echo "✓ 左机械臂激活完成"
echo ""

echo "步骤4: 配置右机械臂 (can1)..."
wait_for_user_confirmation "请插入右机械臂的USB转CAN设备 (保持左机械臂设备连接)，然后按回车键继续..."

# 等待检测到右机械臂设备
wait_for_can_interfaces 2 "右机械臂设备"

# 找到新增的接口（右机械臂）
RIGHT_ARM_INTERFACE=""
RIGHT_ARM_USB=""

for i in "${!DETECTED_INTERFACES[@]}"; do
    if [ "${DETECTED_INTERFACES[$i]}" != "$LEFT_ARM_INTERFACE" ]; then
        RIGHT_ARM_INTERFACE="${DETECTED_INTERFACES[$i]}"
        RIGHT_ARM_USB="${DETECTED_USB_ADDRESSES[$i]}"
        break
    fi
done

if [ -n "$RIGHT_ARM_INTERFACE" ]; then
    echo "步骤5: 激活右机械臂接口..."
    echo "激活右机械臂: $RIGHT_ARM_INTERFACE -> can1 (USB: $RIGHT_ARM_USB)"
    bash ./script/can_activate.sh can1 1000000 "$RIGHT_ARM_USB"
    echo "✓ 右机械臂激活完成"
else
    echo "❌ 未能检测到新的右机械臂接口"
    exit 1
fi

echo ""
echo "步骤6: 显示网络接口状态..."
ifconfig
echo ""

# 检查版本信息
echo "步骤7: 检查设备版本信息..."
bash ./script/version_check.sh

echo ""
echo "===== CAN接口配置总结 ====="
echo "✓ 左机械臂 (can0): $LEFT_ARM_INTERFACE (USB: $LEFT_ARM_USB)"
echo "✓ 右机械臂 (can1): $RIGHT_ARM_INTERFACE (USB: $RIGHT_ARM_USB)"
echo ""
echo "===== 所有操作已完成 ====="