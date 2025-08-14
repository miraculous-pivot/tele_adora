#!/bin/bash

# 测试VR服务的init.sh脚本功能
echo "=== 测试VR服务init.sh脚本 ==="

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT/slave/VR_service"

echo "脚本位置: $(pwd)/script/init.sh"
echo "脚本权限: $(ls -la script/init.sh)"
echo ""

echo "测试脚本开头部分..."
head -20 script/init.sh

echo ""
echo "=== 测试完成 ==="
