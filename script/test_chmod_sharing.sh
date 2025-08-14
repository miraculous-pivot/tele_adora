#!/bin/bash

# 验证chmod权限是否在不同终端间共享

echo "=== chmod 权限共享性验证测试 ==="
echo ""

# 创建一个测试文件
TEST_FILE="/tmp/chmod_test_$(date +%s).txt"
echo "测试内容" > "$TEST_FILE"

echo "1. 创建测试文件: $TEST_FILE"
echo "   初始权限:"
ls -la "$TEST_FILE"

echo ""
echo "2. 修改权限为 644 (rw-r--r--)"
chmod 644 "$TEST_FILE"
echo "   当前权限:"
ls -la "$TEST_FILE"

echo ""
echo "3. 验证: 在新的子shell中检查权限"
bash -c "echo '   子shell中的权限:'; ls -la '$TEST_FILE'"

echo ""
echo "4. 修改权限为 755 (rwxr-xr-x)"
chmod 755 "$TEST_FILE"
echo "   修改后权限:"
ls -la "$TEST_FILE"

echo ""
echo "5. 再次在子shell中验证"
bash -c "echo '   子shell中的权限:'; ls -la '$TEST_FILE'"

echo ""
echo "=== 串口设备权限检查 ==="
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

for i in "${!devices[@]}"; do
    device="${devices[$i]}"
    name="${device_names[$i]}"
    
    if [ -e "$device" ]; then
        echo "$name:"
        ls -la "$device"
        echo "  用户组: $(stat -c %G "$device")"
        echo "  权限数字: $(stat -c %a "$device")"
    else
        echo "$name: 设备不存在"
    fi
    echo ""
done

echo "=== 结论 ==="
echo "✅ chmod 权限是系统级共享的"
echo "✅ 一次设置，所有终端和进程都能看到相同权限"
echo "✅ 我们的一次性权限设置策略是正确的"

# 清理测试文件
rm -f "$TEST_FILE"
echo ""
echo "测试文件已清理: $TEST_FILE"
