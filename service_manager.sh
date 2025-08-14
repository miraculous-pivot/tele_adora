#!/bin/bash

# Tele-Adora 服务管理脚本
# 用于查看、管理和关闭后台运行的服务
# 用法: bash service_manager.sh [stop|status|logs|kill-all]

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
    echo -e "${CYAN}         Tele-Adora 服务管理器               ${NC}"
    echo -e "${CYAN}=================================================${NC}"
    echo ""
}

# 打印错误信息
print_error() {
    echo -e "${RED}❌ $1${NC}"
}

# 打印成功信息
print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

# 打印警告信息
print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

# 打印信息
print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

# 检查是否有Tele-Adora相关的进程
check_services() {
    echo "=== 检查运行中的 Tele-Adora 相关服务 ==="
    echo ""
    
    local found_services=false
    
    # 检查ROS2节点
    echo "🔍 ROS2 节点:"
    if command -v ros2 >/dev/null 2>&1; then
        ros2_nodes=$(ros2 node list 2>/dev/null | grep -E "(lifting_motor|chassis|gimbal|head|arm|vr_arm|camera|suction)" || true)
        if [ -n "$ros2_nodes" ]; then
            echo "$ros2_nodes" | while read node; do
                echo "  📡 $node"
            done
            found_services=true
        else
            echo "  (未发现ROS2节点)"
        fi
    else
        echo "  (ROS2未安装或未配置)"
    fi
    echo ""
    
    # 检查后台进程
    echo "🔍 后台进程:"
    
    # 查找nohup启动的进程
    nohup_processes=$(ps aux | grep -E "(nohup.*ros2|nohup.*tele_adora)" | grep -v grep || true)
    if [ -n "$nohup_processes" ]; then
        echo "$nohup_processes" | while IFS= read -r line; do
            pid=$(echo "$line" | awk '{print $2}')
            cmd=$(echo "$line" | awk '{for(i=11;i<=NF;i++) printf "%s ", $i; print ""}')
            echo "  🔄 PID: $pid - $cmd"
        done
        found_services=true
    fi
    
    # 查找colcon和ros2相关进程
    ros_processes=$(ps aux | grep -E "(ros2 launch|ros2 run|colcon)" | grep -v grep || true)
    if [ -n "$ros_processes" ]; then
        echo "$ros_processes" | while IFS= read -r line; do
            pid=$(echo "$line" | awk '{print $2}')
            cmd=$(echo "$line" | awk '{for(i=11;i<=NF;i++) printf "%s ", $i; print ""}')
            echo "  🤖 PID: $pid - $cmd"
        done
        found_services=true
    fi
    
    # 查找gnome-terminal进程(显示模式)
    terminal_processes=$(ps aux | grep -E "gnome-terminal.*tele" | grep -v grep || true)
    if [ -n "$terminal_processes" ]; then
        echo "$terminal_processes" | while IFS= read -r line; do
            pid=$(echo "$line" | awk '{print $2}')
            echo "  🖥️  PID: $pid - gnome-terminal (显示模式)"
        done
        found_services=true
    fi
    
    if [ "$found_services" = false ]; then
        echo "  (未发现运行中的服务)"
    fi
    echo ""
}

# 显示日志文件
show_logs() {
    echo "=== Tele-Adora 服务日志 ==="
    echo ""
    
    log_files=$(ls /tmp/tele_adora_*.log 2>/dev/null || true)
    if [ -n "$log_files" ]; then
        for log_file in $log_files; do
            service_name=$(basename "$log_file" .log | sed 's/tele_adora_//')
            echo "📋 服务: $service_name"
            echo "📁 日志文件: $log_file"
            echo "📄 最新日志内容:"
            tail -10 "$log_file" 2>/dev/null | sed 's/^/   │ /' || echo "   │ (无法读取日志)"
            echo ""
        done
    else
        print_warning "未找到日志文件"
    fi
}

# 停止特定类型的服务
stop_services() {
    local service_type="$1"
    
    case "$service_type" in
        "ros2")
            echo "🛑 停止 ROS2 节点..."
            if command -v ros2 >/dev/null 2>&1; then
                # 尝试优雅关闭ROS2节点
                pkill -f "ros2 launch" 2>/dev/null || true
                pkill -f "ros2 run" 2>/dev/null || true
                sleep 2
                print_success "ROS2 节点停止命令已发送"
            else
                print_warning "ROS2 未安装或未配置"
            fi
            ;;
        "nohup")
            echo "🛑 停止后台进程..."
            # 停止nohup启动的进程
            pkill -f "nohup.*ros2" 2>/dev/null || true
            pkill -f "nohup.*tele_adora" 2>/dev/null || true
            sleep 2
            print_success "后台进程停止命令已发送"
            ;;
        "terminals")
            echo "🛑 关闭终端窗口..."
            # 关闭包含特定标题的gnome-terminal
            pkill -f "gnome-terminal.*LIFTING" 2>/dev/null || true
            pkill -f "gnome-terminal.*CHASSIS" 2>/dev/null || true
            pkill -f "gnome-terminal.*GIMBAL" 2>/dev/null || true
            pkill -f "gnome-terminal.*ARM" 2>/dev/null || true
            pkill -f "gnome-terminal.*CAMERA" 2>/dev/null || true
            pkill -f "gnome-terminal.*VR" 2>/dev/null || true
            pkill -f "gnome-terminal.*HEAD" 2>/dev/null || true
            pkill -f "gnome-terminal.*PUMP" 2>/dev/null || true
            pkill -f "gnome-terminal.*WEB" 2>/dev/null || true
            sleep 1
            print_success "终端窗口关闭命令已发送"
            ;;
        "all")
            echo "🛑 停止所有 Tele-Adora 相关服务..."
            stop_services "ros2"
            stop_services "nohup"
            stop_services "terminals"
            
            # 额外的清理
            sleep 2
            pkill -f "colcon" 2>/dev/null || true
            pkill -f "adora" 2>/dev/null || true
            
            print_success "所有服务停止命令已发送"
            ;;
        *)
            print_error "未知的服务类型: $service_type"
            ;;
    esac
}

# 强制杀死所有相关进程
kill_all_force() {
    echo "💥 强制终止所有 Tele-Adora 相关进程..."
    print_warning "这将强制杀死所有相关进程，可能导致数据丢失！"
    echo ""
    read -p "确认强制终止所有进程? (输入 'YES' 确认): " confirm
    
    if [ "$confirm" = "YES" ]; then
        echo ""
        echo "🔥 强制终止进程中..."
        
        # 强制杀死所有相关进程
        pkill -9 -f "ros2" 2>/dev/null || true
        pkill -9 -f "colcon" 2>/dev/null || true
        pkill -9 -f "adora" 2>/dev/null || true
        pkill -9 -f "nohup.*tele" 2>/dev/null || true
        pkill -9 -f "gnome-terminal.*LIFTING" 2>/dev/null || true
        pkill -9 -f "gnome-terminal.*CHASSIS" 2>/dev/null || true
        pkill -9 -f "gnome-terminal.*GIMBAL" 2>/dev/null || true
        pkill -9 -f "gnome-terminal.*ARM" 2>/dev/null || true
        pkill -9 -f "gnome-terminal.*CAMERA" 2>/dev/null || true
        pkill -9 -f "gnome-terminal.*VR" 2>/dev/null || true
        pkill -9 -f "gnome-terminal.*HEAD" 2>/dev/null || true
        pkill -9 -f "gnome-terminal.*PUMP" 2>/dev/null || true
        pkill -9 -f "gnome-terminal.*WEB" 2>/dev/null || true
        
        sleep 2
        print_success "强制终止完成"
        
        # 清理日志文件
        echo ""
        read -p "是否清理日志文件? (y/N): " clean_logs
        case "$clean_logs" in
            [Yy]|[Yy][Ee][Ss])
                rm -f /tmp/tele_adora_*.log 2>/dev/null || true
                print_success "日志文件已清理"
                ;;
            *)
                print_info "保留日志文件"
                ;;
        esac
    else
        print_info "已取消强制终止操作"
    fi
}

# 交互式菜单
interactive_menu() {
    while true; do
        print_header
        check_services
        
        echo "请选择操作："
        echo "1. 停止 ROS2 节点"
        echo "2. 停止后台进程"
        echo "3. 关闭终端窗口"
        echo "4. 停止所有服务"
        echo "5. 查看服务日志"
        echo "6. 强制终止所有进程"
        echo "7. 刷新状态"
        echo "0. 退出"
        echo ""
        
        read -p "请输入选择 (0-7): " choice
        echo ""
        
        case "$choice" in
            1)
                stop_services "ros2"
                ;;
            2)
                stop_services "nohup"
                ;;
            3)
                stop_services "terminals"
                ;;
            4)
                stop_services "all"
                ;;
            5)
                show_logs
                ;;
            6)
                kill_all_force
                ;;
            7)
                continue
                ;;
            0)
                print_info "退出服务管理器"
                exit 0
                ;;
            *)
                print_error "无效选择，请重试"
                ;;
        esac
        
        echo ""
        echo "按回车键继续..."
        read
    done
}

# 主程序
case "$1" in
    "stop")
        print_header
        stop_services "all"
        ;;
    "status")
        print_header
        check_services
        ;;
    "logs")
        print_header
        show_logs
        ;;
    "kill-all")
        print_header
        kill_all_force
        ;;
    "help"|"--help"|"-h")
        print_header
        echo "用法: bash service_manager.sh [命令]"
        echo ""
        echo "命令："
        echo "  status     - 查看运行中的服务状态"
        echo "  stop       - 停止所有 Tele-Adora 服务"
        echo "  logs       - 查看服务日志"
        echo "  kill-all   - 强制终止所有相关进程"
        echo "  (无参数)   - 启动交互式菜单"
        echo ""
        echo "示例："
        echo "  bash service_manager.sh status    # 查看服务状态"
        echo "  bash service_manager.sh stop      # 停止所有服务"
        echo "  bash service_manager.sh           # 交互式管理"
        ;;
    "")
        interactive_menu
        ;;
    *)
        print_error "未知命令: $1"
        echo "使用 'bash service_manager.sh help' 查看帮助"
        exit 1
        ;;
esac
