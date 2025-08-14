#!/bin/bash

# 设备配置测试脚本
# 用于测试设备识别和配置功能

echo "===== 设备配置系统测试 ====="
echo ""

# 测试1: 检查配置文件是否存在
test_config_file() {
    echo "测试1: 检查设备配置文件"
    echo "========================"
    
    local config_file="/home/feng/tele_adora/config/device_mapping.txt"
    
    if [ -f "$config_file" ]; then
        echo "✓ 配置文件存在: $config_file"
        echo "配置内容："
        cat "$config_file"
        echo ""
        return 0
    else
        echo "❌ 配置文件不存在: $config_file"
        echo "请先运行: bash script/device_identification.sh"
        echo ""
        return 1
    fi
}

# 测试2: 测试配置加载功能
test_config_loading() {
    echo "测试2: 测试配置加载功能"
    echo "========================="
    
    # 加载配置
    source ./script/load_device_config.sh
    
    if load_device_config; then
        echo "✓ 配置加载成功"
        echo ""
        echo "已加载的设备配置："
        echo "头部控制设备: ${HEAD_CONTROL_PORT:-未设置}"
        echo "底盘控制设备: ${CHASSIS_CONTROL_PORT:-未设置}"
        echo "升降电机设备: ${LIFTING_MOTOR_PORT:-未设置}"
        echo "吸盘控制设备: ${SUCTION_PUMP_PORT:-未设置}"
        echo "吸盘功能状态: ${SUCTION_PUMP_ENABLED:-未设置}"
        echo ""
        return 0
    else
        echo "❌ 配置加载失败"
        echo ""
        return 1
    fi
}

# 测试3: 测试设备验证功能
test_device_validation() {
    echo "测试3: 测试设备验证功能"
    echo "========================="
    
    # 确保配置已加载
    source ./script/load_device_config.sh
    load_device_config > /dev/null 2>&1
    
    if validate_devices; then
        echo "✓ 设备验证通过"
        return 0
    else
        echo "⚠️  部分设备验证失败（这在测试环境中是正常的）"
        return 1
    fi
}

# 测试4: 检查launch文件参数支持
test_launch_files() {
    echo "测试4: 检查launch文件参数支持"
    echo "============================="
    
    local files_to_check=(
        "./slave/adora/ros2_head_control/src/ros2_head_control/launch/head_control.launch.py"
        "./slave/adora/adora_suction_pump_control/launch/suction_pump_control.launch.py"
        "./slave/adora/adora_lifting_motor_control/launch/adora_a2_max_ros2_node.py"
        "./slave/adora/adora_chassis_control/adora_chassis_bringup/launch/adora_a2_max_ros2.launch.py"
    )
    
    local all_good=true
    
    for file in "${files_to_check[@]}"; do
        if [ -f "$file" ]; then
            if grep -q "LaunchConfiguration" "$file"; then
                echo "✓ $file - 支持参数配置"
            else
                echo "⚠️  $file - 可能不支持参数配置"
                all_good=false
            fi
        else
            echo "❌ 文件不存在: $file"
            all_good=false
        fi
    done
    
    echo ""
    if [ "$all_good" = true ]; then
        return 0
    else
        return 1
    fi
}

# 测试5: 模拟启动脚本配置检查
test_startup_scripts() {
    echo "测试5: 检查启动脚本配置支持"
    echo "=============================="
    
    local scripts=(
        "./script/setup_keyservice.sh"
        "./script/setup_vrservice.sh"
    )
    
    local all_good=true
    
    for script in "${scripts[@]}"; do
        if [ -f "$script" ]; then
            if grep -q "load_device_config" "$script"; then
                echo "✓ $script - 支持设备配置加载"
            else
                echo "❌ $script - 不支持设备配置加载"
                all_good=false
            fi
            
            if grep -q "SUCTION_PUMP_ENABLED" "$script"; then
                echo "✓ $script - 支持吸盘功能控制"
            else
                echo "❌ $script - 不支持吸盘功能控制"
                all_good=false
            fi
        else
            echo "❌ 脚本不存在: $script"
            all_good=false
        fi
    done
    
    echo ""
    if [ "$all_good" = true ]; then
        return 0
    else
        return 1
    fi
}

# 运行所有测试
main() {
    local passed=0
    local total=5
    
    test_config_file && ((passed++))
    test_config_loading && ((passed++))
    test_device_validation && ((passed++))
    test_launch_files && ((passed++))
    test_startup_scripts && ((passed++))
    
    echo "==============================================="
    echo "测试完成: $passed/$total 项测试通过"
    echo "==============================================="
    
    if [ $passed -eq $total ]; then
        echo "🎉 所有测试通过！设备配置系统工作正常。"
        return 0
    else
        echo "⚠️  部分测试失败，请检查上述错误信息。"
        return 1
    fi
}

# 切换到项目根目录
cd "$(dirname "$0")/.."

# 运行测试
main
