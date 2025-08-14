#!/bin/bash

# 测试终端模式控制功能

echo "=== 终端模式控制功能测试 ==="
echo ""

# 测试函数
test_terminal_mode() {
    echo "测试模拟函数:"
    
    # 模拟run_service函数
    run_service() {
        local title="$1"
        local working_dir="$2"
        local command="$3"
        
        if [ "$SHOW_TERMINALS" = true ]; then
            echo "  [终端模式] 启动终端: $title"
            echo "    工作目录: $working_dir"
            echo "    命令: $command"
        else
            local log_file="/tmp/tele_adora_${title// /_}.log"
            echo "  [后台模式] 后台启动: $title"
            echo "    日志文件: $log_file"
        fi
    }
    
    # 测试不同模式
    echo ""
    echo "1. 测试默认模式（后台运行）:"
    SHOW_TERMINALS=false
    run_service "TEST SERVICE" "/tmp" "echo test command"
    
    echo ""
    echo "2. 测试终端显示模式:"
    SHOW_TERMINALS=true
    run_service "TEST SERVICE" "/tmp" "echo test command"
}

test_terminal_mode

echo ""
echo "=== 参数解析测试 ==="

# 测试参数解析逻辑
test_args() {
    local arg="$1"
    echo "测试参数: '$arg'"
    
    SHOW_TERMINALS=false
    if [ "$arg" = "--show-terminals" ] || [ "$arg" = "-t" ]; then
        SHOW_TERMINALS=true
        echo "  结果: 显示终端模式"
    else
        echo "  结果: 后台运行模式"
    fi
}

test_args ""
test_args "--show-terminals"
test_args "-t"
test_args "invalid"

echo ""
echo "=== 使用示例 ==="
echo "默认模式（后台运行）:"
echo "  bash start.sh"
echo "  bash main_setup.sh"
echo "  bash setup_keyservice.sh"
echo ""
echo "显示终端模式:"
echo "  bash start.sh --show-terminals"
echo "  bash main_setup.sh -t"
echo "  bash setup_keyservice.sh --show-terminals"

echo ""
echo "=== 测试完成 ==="
