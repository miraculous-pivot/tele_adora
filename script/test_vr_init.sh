#!/bin/bash

# 测试VR服务的init.sh脚本功能
echo "=== 测试VR服务init.sh脚本 ==="

cd /home/feng/tele_adora/slave/VR_service

echo "脚本位置: $(pwd)/script/init.sh"
echo "脚本权限: $(ls -la script/init.sh)"
echo ""

echo "测试脚本开头部分..."
head -20 script/init.sh

echo ""
echo "=== 测试完成 ==="
