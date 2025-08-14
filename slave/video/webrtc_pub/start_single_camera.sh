#!/bin/bash

# 单相机HTTP流启动脚本
# 启动单个相机的HTTP MJPEG流服务

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$SCRIPT_DIR"
LAUNCH_FILE="http_camera_stream.launch.py"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的信息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查ROS2环境
check_ros_env() {
    print_info "检查ROS2环境..."
    
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2未安装或未设置环境变量"
        return 1
    fi
    
    # 检查工作空间
    if [ ! -d "$WORKSPACE" ]; then
        print_error "工作空间不存在: $WORKSPACE"
        return 1
    fi
    
    # 设置环境
    cd "$WORKSPACE"
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        print_success "已加载工作空间环境"
    else
        print_warning "工作空间未构建，尝试构建..."
        colcon build --packages-select webrtc_pub
        if [ $? -eq 0 ]; then
            source install/setup.bash
            print_success "构建并加载工作空间成功"
        else
            print_error "工作空间构建失败"
            return 1
        fi
    fi
    
    return 0
}

# 启动服务
start_service() {
    print_info "启动单相机HTTP流服务..."
    
    if ! check_ros_env; then
        exit 1
    fi
    
    print_info "使用launch文件: $LAUNCH_FILE"
    print_info "默认配置: /camera/color/image_raw -> http://localhost:8080"
    print_info "访问地址: http://localhost:8080"
    
    # 启动launch文件
    ros2 launch webrtc_pub "$LAUNCH_FILE"
}

# 显示使用说明
show_usage() {
    echo "单相机HTTP流启动脚本"
    echo "用法: $0"
    echo ""
    echo "功能:"
    echo "  - 启动单个相机的HTTP MJPEG流服务"
    echo "  - 默认监听端口: 8080"
    echo "  - 默认相机话题: /camera/color/image_raw"
    echo "  - 默认分辨率: 640x480"
    echo ""
    echo "访问方式:"
    echo "  - 浏览器访问: http://localhost:8080"
    echo "  - 纯流页面: http://localhost:8080/stream"
    echo "  - 状态API: http://localhost:8080/status"
    echo ""
    echo "快捷键:"
    echo "  - Ctrl+C: 停止服务"
    echo "  - F键: 全屏播放"
    echo "  - R键: 刷新页面"
}

# 主程序
main() {
    echo "================================================"
    echo "🎥 ROS2单相机HTTP流启动脚本"
    echo "================================================"
    
    # 如果有参数且是help，显示帮助
    if [ "$1" = "help" ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
        show_usage
        exit 0
    fi
    
    # 启动服务
    start_service
}

# 执行主程序
main "$@"
