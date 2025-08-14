#!/bin/bash

# 主启动脚本 - 用于选择不同的控制模式
# 作者: Tele-Adora Team
# 日期: 2025年8月13日
# 用法: bash main_setup.sh [--show-terminals]

# 处理命令行参数
TERMINAL_ARG=""
if [ "$1" = "--show-terminals" ] || [ "$1" = "-t" ]; then
    TERMINAL_ARG="--show-terminals"
    echo "注意: 将显示终端窗口模式"
    echo ""
fi

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 打印标题
print_header() {
    echo -e "${CYAN}=================================================${NC}"
    echo -e "${CYAN}         Tele-Adora 遥操作系统启动器           ${NC}"
    echo -e "${CYAN}=================================================${NC}"
    echo ""
}

# 打印错误信息
print_error() {
    echo -e "${RED}错误: $1${NC}"
}

# 打印成功信息
print_success() {
    echo -e "${GREEN}成功: $1${NC}"
}

# 打印警告信息
print_warning() {
    echo -e "${YELLOW}警告: $1${NC}"
}

# 打印信息
print_info() {
    echo -e "${BLUE}信息: $1${NC}"
}

# 检查脚本文件是否存在
check_scripts() {
    local missing_scripts=0
    
    if [ ! -f "./setup_keyboard.sh" ]; then
        print_error "setup_keyboard.sh 文件不存在"
        missing_scripts=1
    fi
    
    if [ ! -f "./setup_keyservice.sh" ]; then
        print_error "setup_keyservice.sh 文件不存在"
        missing_scripts=1
    fi
    
    if [ ! -f "./setup_vrservice.sh" ]; then
        print_error "setup_vrservice.sh 文件不存在"
        missing_scripts=1
    fi
    
    if [ ! -f "./setup_step.bash" ]; then
        print_error "setup_step.bash 文件不存在"
        missing_scripts=1
    fi
    
    if [ $missing_scripts -eq 1 ]; then
        print_error "缺少必要的脚本文件，请确保所有脚本文件都在正确位置"
        exit 1
    fi
}

# 显示设备类型选择菜单
show_device_menu() {
    # 显示设备配置状态
    echo ""
    echo -e "${CYAN}=== 设备配置状态 ===${NC}"
    DEVICE_CONFIG_FILE="../config/device_mapping.txt"
    
    if [ -f "$DEVICE_CONFIG_FILE" ]; then
        echo -e "${GREEN}✅ 设备配置: 已配置${NC}"
        
        # 加载并显示设备配置状态
        source ./load_device_config.sh
        if load_device_config > /dev/null 2>&1; then
            if [ "$SUCTION_PUMP_ENABLED" = "true" ]; then
                echo -e "${GREEN}✅ 吸盘功能: 已启用${NC}"
            else
                echo -e "${YELLOW}ℹ️  吸盘功能: 已禁用${NC}"
            fi
        else
            echo -e "${YELLOW}⚠️  配置加载异常${NC}"
        fi
    else
        echo -e "${YELLOW}⚠️  设备配置: 未配置 (将使用默认设置)${NC}"
        echo -e "${BLUE}💡 建议运行设备配置以获得最佳体验${NC}"
    fi
    
    echo ""
    echo -e "${YELLOW}请选择操作:${NC}"
    echo "1) 本体 (Robot Body) - 机器人本体端"
    echo "2) 遥操作设备 (Teleoperation Device) - 控制端设备"
    echo "3) 设备配置 (Device Configuration) - 配置设备映射"
    echo "0) 退出"
    echo ""
    echo -n "请输入您的选择 [0-3]: "
}

# 显示本体控制方式菜单
show_robot_control_menu() {
    echo ""
    echo -e "${YELLOW}本体控制方式选择:${NC}"
    echo "1) 键盘控制服务 (Keyboard Control Service)"
    echo "2) VR控制服务 (VR Control Service)"
    echo "0) 返回上级菜单"
    echo ""
    echo -n "请输入您的选择 [0-2]: "
}

# 显示遥操作设备菜单
show_teleop_menu() {
    echo ""
    echo -e "${YELLOW}遥操作设备选择:${NC}"
    echo "1) 足部控制器 (Foot Controller)"
    echo "2) 键盘设备 (Keyboard Device)"
    echo "0) 返回上级菜单"
    echo ""
    echo -n "请输入您的选择 [0-2]: "
}

# 执行键盘控制服务
run_keyboard_service() {
    print_info "启动键盘控制服务..."
    echo -e "${CYAN}正在执行: setup_keyservice.sh${NC}"
    echo ""
    sudo chmod +x ./setup_keyservice.sh
    # 传递终端显示参数
    if [ "$1" = "--show-terminals" ]; then
        bash ./setup_keyservice.sh --show-terminals
    else
        bash ./setup_keyservice.sh
    fi
}

# 执行VR控制服务
run_vr_service() {
    print_info "启动VR控制服务..."
    echo -e "${CYAN}正在执行: setup_vrservice.sh${NC}"
    echo ""
    pwd
    sudo chmod +x ./setup_vrservice.sh
    # 传递终端显示参数
    if [ "$1" = "--show-terminals" ]; then
        bash ./setup_vrservice.sh --show-terminals
    else
        bash ./setup_vrservice.sh
    fi
}

# 执行足部控制器
run_foot_controller() {
    print_info "启动足部控制器..."
    echo -e "${CYAN}正在执行: setup_step.bash${NC}"
    echo ""
    sudo chmod +x ./setup_step.bash
    bash ./setup_step.bash
}

# 显示设备配置菜单
show_device_config_menu() {
    echo ""
    echo -e "${YELLOW}设备配置选项:${NC}"
    echo "1) 运行设备识别向导 (Device Identification Wizard)"
    echo "2) 查看设备配置演示 (Configuration Demo)"
    echo "3) 测试设备配置系统 (Test Configuration)"
    echo "4) 查看当前设备状态 (Current Device Status)"
    echo "5) 云台零点校准 (Head Zero Calibration)"
    echo "0) 返回上级菜单"
    echo ""
    echo -n "请输入您的选择 [0-5]: "
}

# 执行设备配置向导
run_device_identification() {
    print_info "启动设备识别向导..."
    echo -e "${CYAN}正在执行: device_identification.sh${NC}"
    echo ""
    sudo chmod +x ./device_identification.sh
    bash ./device_identification.sh
}

# 执行设备配置演示
run_device_demo() {
    print_info "启动设备配置演示..."
    echo -e "${CYAN}正在执行: demo_device_config.sh${NC}"
    echo ""
    sudo chmod +x ./demo_device_config.sh
    bash ./demo_device_config.sh
}

# 执行设备配置测试
run_device_test() {
    print_info "启动设备配置测试..."
    echo -e "${CYAN}正在执行: test_device_config.sh${NC}"
    echo ""
    sudo chmod +x ./test_device_config.sh
    bash ./test_device_config.sh
}

# 云台零点校准
run_head_calibration() {
    print_info "启动云台零点校准..."
    
    # 云台校准脚本路径（使用测试版本）
    CALIBRATION_SCRIPT="../slave/adora/ros2_head_control/head_calibration_test.py"
    
    if [ ! -f "$CALIBRATION_SCRIPT" ]; then
        print_error "云台校准脚本不存在: $CALIBRATION_SCRIPT"
        return 1
    fi
    
    echo ""
    echo -e "${YELLOW}云台零点校准选项:${NC}"
    echo "1) 查看当前云台状态并进行配置测试"
    echo "2) 查看配置文件内容"
    echo "0) 返回"
    echo ""
    echo -n "请输入您的选择 [0-2]: "
    
    read -r calibration_choice
    
    case $calibration_choice in
        1)
            print_info "启动云台配置测试..."
            echo -e "${CYAN}正在执行: python3 $CALIBRATION_SCRIPT${NC}"
            echo ""
            python3 "$CALIBRATION_SCRIPT"
            ;;
        2)
            print_info "查看云台配置文件..."
            CONFIG_FILE="../slave/adora/ros2_head_control/config/head_zero_config.txt"
            if [ -f "$CONFIG_FILE" ]; then
                echo -e "${CYAN}配置文件内容:${NC}"
                echo "========================="
                cat "$CONFIG_FILE"
                echo "========================="
            else
                print_error "配置文件不存在: $CONFIG_FILE"
            fi
            ;;
        0)
            return 0
            ;;
        *)
            print_error "无效选择，请输入 0-2"
            return 1
            ;;
    esac
}

# 显示当前设备状态
show_device_status() {
    print_info "当前设备状态..."
    echo -e "${CYAN}正在执行: load_device_config.sh${NC}"
    echo ""
    sudo chmod +x ./load_device_config.sh
    bash ./load_device_config.sh
}

# 执行键盘设备
run_keyboard_device() {
    print_info "启动键盘设备..."
    echo -e "${CYAN}正在执行: setup_keyboard.sh${NC}"
    echo ""
    sudo chmod +x ./setup_keyboard.sh
    bash ./setup_keyboard.sh
}

# 确认执行
confirm_execution() {
    local script_name=$1
    echo ""
    echo -e "${YELLOW}确认要执行 $script_name 吗? (Y/n)${NC}"
    echo -n "请输入 [Y/n] (直接按回车确认): "
    read -r confirm
    
    # 检查是否为空输入（直接按回车）
    if [ -z "$confirm" ]; then
        print_info "检测到回车键，确认执行"
        return 0
    fi
    
    case $confirm in
        [yY]|[yY][eE][sS])
            return 0
            ;;
        [nN]|[nN][oO])
            print_info "操作已取消"
            return 1
            ;;
        *)
            print_warning "无效输入: '$confirm'，默认确认执行"
            return 0
            ;;
    esac
}

# 主函数
main() {
    # 切换到脚本所在目录
    cd "$(dirname "$0")"
    pwd
    
    print_header
    
    # 检查脚本文件
    print_info "检查脚本文件..."
    check_scripts
    print_success "所有脚本文件检查完成"
    echo ""
    
    while true; do
        show_device_menu
        read -r device_choice
        
        case $device_choice in
            1)
                # 本体选择
                while true; do
                    show_robot_control_menu
                    read -r robot_choice
                    
                    case $robot_choice in
                        1)
                            if confirm_execution "键盘控制服务"; then
                                run_keyboard_service "$TERMINAL_ARG"
                                exit 0
                            fi
                            ;;
                        2)
                            if confirm_execution "VR控制服务"; then
                                run_vr_service "$TERMINAL_ARG"
                                exit 0
                            fi
                            ;;
                        0)
                            break
                            ;;
                        *)
                            print_error "无效选择，请输入 0-2"
                            ;;
                    esac
                done
                ;;
            2)
                # 遥操作设备选择
                while true; do
                    show_teleop_menu
                    read -r teleop_choice
                    
                    case $teleop_choice in
                        1)
                            if confirm_execution "足部控制器"; then
                                run_foot_controller
                                exit 0
                            fi
                            ;;
                        2)
                            if confirm_execution "键盘设备"; then
                                run_keyboard_device
                                exit 0
                            fi
                            ;;
                        0)
                            break
                            ;;
                        *)
                            print_error "无效选择，请输入 0-2"
                            ;;
                    esac
                done
                ;;
            3)
                # 设备配置选择
                while true; do
                    show_device_config_menu
                    read -r config_choice
                    
                    case $config_choice in
                        1)
                            if confirm_execution "设备识别向导"; then
                                run_device_identification
                                echo ""
                                print_success "设备识别完成！"
                                echo ""
                                read -p "按回车继续..."
                            fi
                            ;;
                        2)
                            run_device_demo
                            echo ""
                            read -p "按回车继续..."
                            ;;
                        3)
                            run_device_test
                            echo ""
                            read -p "按回车继续..."
                            ;;
                        4)
                            show_device_status
                            echo ""
                            read -p "按回车继续..."
                            ;;
                        5)
                            run_head_calibration
                            echo ""
                            read -p "按回车继续..."
                            ;;
                        0)
                            break
                            ;;
                        *)
                            print_error "无效选择，请输入 0-5"
                            ;;
                    esac
                done
                ;;
            0)
                print_info "感谢使用 Tele-Adora 遥操作系统！"
                exit 0
                ;;
            *)
                print_error "无效选择，请输入 0-3"
                ;;
        esac
    done
}

# 错误处理
trap 'print_error "脚本执行被中断"; exit 1' INT TERM

# 执行主函数
main "$@"
