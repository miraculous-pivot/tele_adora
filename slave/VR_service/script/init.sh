#!/bin/bash

# 自动CAN接口激活与版本检查脚本

# 设置错误处理
set -e

echo "===== 智能CAN接口管理脚本 ====="
echo ""

# 函数：自动获取所有CAN接口的USB地址
detect_can_ports() {
    echo "正在检测CAN接口..."
    # 运行检测脚本并捕获输出
    local output
    output=$(bash script/find_all_can_port.sh)
    
    # 解析输出并存储结果
    CAN_INTERFACES=()
    CAN_USB_ADDRESSES=()
    
    while IFS= read -r line; do
        if [[ $line =~ "Interface "(can[0-9]+)" is connected to USB port "([^ ]+) ]]; then
            CAN_INTERFACES+=("${BASH_REMATCH[1]}")
            CAN_USB_ADDRESSES+=("${BASH_REMATCH[2]}")
            echo "检测到: ${BASH_REMATCH[1]} -> USB ${BASH_REMATCH[2]}"
        fi
    done <<< "$output"
}

echo "步骤1: 自动检测CAN接口..."
detect_can_ports

# 检查是否检测到CAN接口
if [ ${#CAN_INTERFACES[@]} -eq 0 ]; then
    echo "错误: 未检测到任何CAN接口!"
    exit 1
fi

echo ""
echo "步骤2: 显示网络接口状态..."
ifconfig
echo ""

# 激活所有检测到的CAN接口
echo "步骤3: 激活CAN接口..."
for i in "${!CAN_INTERFACES[@]}"; do
    iface="${CAN_INTERFACES[$i]}"
    usb_addr="${CAN_USB_ADDRESSES[$i]}"
    echo "激活接口 $iface (USB: $usb_addr) ..."
    bash ./script/can_activate.sh "$iface" 1000000 "$usb_addr"
    echo ""
done

# 检查版本信息
echo "步骤4: 检查设备版本信息..."
bash ./script/version_check.sh

echo ""
echo "===== 所有操作已完成 ====="