#!/bin/bash

# Tele-Adora 快速启动脚本
# 此脚本会调用 script 目录下的主脚本
# 用法: bash start.sh [--show-terminals] [--config-devices] [--demo-config]

echo "正在启动 Tele-Adora 遥操作系统..."
echo "请稍等..."

# 检查 script 目录是否存在
if [ ! -d "script" ]; then
    echo "错误: script 目录不存在！"
    exit 1
fi

# 检查主脚本是否存在
if [ ! -f "script/main_setup.sh" ]; then
    echo "错误: 主脚本 script/main_setup.sh 不存在！"
    exit 1
fi

# 处理特殊参数
case "$1" in
    "--config-devices")
        echo "启动设备配置向导..."
        bash script/device_identification.sh
        exit 0
        ;;
    "--demo-config")
        echo "运行设备配置演示..."
        bash script/demo_device_config.sh
        exit 0
        ;;
    "--test-config")
        echo "运行设备配置测试..."
        bash script/test_device_config.sh
        exit 0
        ;;
    "--help"|"-h")
        echo "Tele-Adora 遥操作系统启动选项："
        echo ""
        echo "基本用法:"
        echo "  bash start.sh                    # 启动主菜单"
        echo "  bash start.sh --show-terminals   # 启动时显示终端窗口"
        echo ""
        echo "设备配置选项:"
        echo "  bash start.sh --config-devices   # 运行设备识别配置向导"
        echo "  bash start.sh --demo-config      # 演示设备配置功能"
        echo "  bash start.sh --test-config      # 测试设备配置系统"
        echo ""
        echo "其他选项:"
        echo "  bash start.sh --help             # 显示此帮助信息"
        echo ""
        exit 0
        ;;
esac

# 自动设置脚本权限
echo ""
echo "=== 检查和设置脚本权限 ==="
if [ -f "setup_permissions.sh" ]; then
    # 确保setup_permissions.sh本身有执行权限
    chmod +x setup_permissions.sh
    
    # 运行权限设置脚本（静默模式）
    echo "正在自动设置所有脚本权限..."
    bash setup_permissions.sh --quiet
    
    if [ $? -eq 0 ]; then
        echo "✅ 脚本权限设置完成"
    else
        echo "⚠️  权限设置过程中出现警告，但可以继续"
    fi
else
    echo "⚠️  权限设置脚本不存在，手动检查关键脚本权限..."
    chmod +x script/*.sh 2>/dev/null || true
fi

# 检查设备配置
echo ""
echo "=== 检查设备配置 ==="
DEVICE_CONFIG_FILE="config/device_mapping.txt"

if [ ! -f "$DEVICE_CONFIG_FILE" ]; then
    echo "⚠️  未发现设备配置文件"
    echo ""
    echo "建议首次使用前配置设备映射，以确保系统正常工作。"
    echo ""
    echo "请选择："
    echo "1. 现在配置设备 (推荐)"
    echo "2. 使用默认配置继续"
    echo "3. 查看设备配置演示"
    echo ""
    
    while true; do
        read -p "请输入选择 (1-3，或按回车选择1): " choice
        case "${choice:-1}" in
            1)
                echo "启动设备配置向导..."
                bash script/device_identification.sh
                if [ $? -eq 0 ]; then
                    echo ""
                    echo "✅ 设备配置完成！继续启动系统..."
                    echo ""
                else
                    echo ""
                    echo "⚠️  设备配置未完成，将使用默认配置"
                    echo ""
                fi
                break
                ;;
            2)
                echo "使用默认配置继续..."
                echo ""
                break
                ;;
            3)
                echo "启动设备配置演示..."
                bash script/demo_device_config.sh
                echo ""
                echo "演示完成，请选择继续方式："
                continue
                ;;
            *)
                echo "无效选择，请输入 1、2 或 3"
                ;;
        esac
    done
else
    echo "✅ 发现设备配置文件: $DEVICE_CONFIG_FILE"
    
    # 快速验证配置
    source script/load_device_config.sh
    if load_device_config > /dev/null 2>&1; then
        echo "✅ 设备配置加载成功"
        
        # 显示吸盘状态
        if [ "$SUCTION_PUMP_ENABLED" = "true" ]; then
            echo "ℹ️  吸盘功能: 已启用"
        else
            echo "ℹ️  吸盘功能: 已禁用"
        fi
    else
        echo "⚠️  设备配置加载失败，可能需要重新配置"
        echo ""
        echo "是否重新配置设备? (y/N): "
        read -p "" reconfig_choice
        case "$reconfig_choice" in
            [Yy]|[Yy][Ee][Ss])
                bash script/device_identification.sh
                ;;
            *)
                echo "继续使用现有配置..."
                ;;
        esac
    fi
    echo ""
fi

# 切换到 script 目录并执行主脚本，传递所有参数
cd script
chmod +x ./main_setup.sh
bash ./main_setup.sh "$@"
