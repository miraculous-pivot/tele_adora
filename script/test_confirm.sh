#!/bin/bash

# 测试确认函数的回车检测功能

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印信息
print_info() {
    echo -e "${BLUE}信息: $1${NC}"
}

# 打印警告信息
print_warning() {
    echo -e "${YELLOW}警告: $1${NC}"
}

# 确认执行函数（与主脚本相同）
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

echo "=== 测试确认函数的回车检测功能 ==="
echo "可以尝试以下输入："
echo "1. 直接按回车键 (默认确认)"
echo "2. 输入 y 然后回车 (确认)"
echo "3. 输入 n 然后回车 (取消)"
echo "4. 输入无效字符然后回车 (默认确认)"

if confirm_execution "测试脚本"; then
    echo "用户确认执行！"
else
    echo "用户取消执行。"
fi

echo "测试完成。"
