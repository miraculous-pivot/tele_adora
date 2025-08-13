#!/bin/bash

# 多相机HTTP流启动脚本
# 启动根据用户指定配置的4路相机HTTP MJPEG流服务

WORKSPACE="/home/feng/tele_adora/slave/video/webrtc_pub"
LAUNCH_FILE="specified_multi_camera.launch.py"

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

# 显示相机配置
show_camera_config() {
    echo ""
    print_info "=== 4路相机配置信息 ==="
    echo "📹 相机1 (ZED): /zed/zed_node/rgb/image_rect_color"
    echo "   📺 分辨率: 960x540"
    echo "   🌐 端口: 8081"
    echo "   🔗 访问: http://localhost:8081"
    echo ""
    echo "📹 相机2: /camera1/camera1/color/image_rect_raw"
    echo "   📺 分辨率: 848x480"
    echo "   🌐 端口: 8082"
    echo "   🔗 访问: http://localhost:8082"
    echo ""
    echo "📹 相机3: /camera2/camera2/color/image_rect_raw"
    echo "   📺 分辨率: 848x480"
    echo "   🌐 端口: 8083"
    echo "   🔗 访问: http://localhost:8083"
    echo ""
    echo "📹 相机4 (主相机): /camera/color/image_raw"
    echo "   📺 分辨率: 640x480"
    echo "   🌐 端口: 8084"
    echo "   🔗 访问: http://localhost:8084"
    echo ""
}

# 启动服务
start_service() {
    print_info "启动多相机HTTP流服务..."
    
    if ! check_ros_env; then
        exit 1
    fi
    
    print_info "使用launch文件: $LAUNCH_FILE"
    show_camera_config
    
    print_warning "注意: 确保所有相机话题都已发布，否则相应端口会显示等待画面"
    print_info "按 Ctrl+C 停止所有相机流服务"
    echo ""
    
    # 启动launch文件
    ros2 launch webrtc_pub "$LAUNCH_FILE"
}

# 显示使用说明
show_usage() {
    echo "多相机HTTP流启动脚本"
    echo "用法: $0 [help]"
    echo ""
    echo "功能:"
    echo "  - 同时启动4路相机的HTTP MJPEG流服务"
    echo "  - 每个相机使用独立的端口"
    echo "  - 支持不同的分辨率配置"
    echo ""
    show_camera_config
    echo ""
    echo "访问方式:"
    echo "  - 浏览器直接访问上述端口"
    echo "  - 纯流页面: http://localhost:端口/stream"
    echo "  - 状态API: http://localhost:端口/status"
    echo ""
    echo "控制:"
    echo "  - Ctrl+C: 停止所有服务"
    echo "  - F键: 全屏播放"
    echo "  - R键: 刷新页面"
}

# 主程序
main() {
    echo "================================================"
    echo "🎥 ROS2多相机HTTP流启动脚本"
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
