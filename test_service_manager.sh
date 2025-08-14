#!/bin/bash

# 服务管理器测试脚本

# 获取脚本所在目录的绝对路径  
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Service Manager 测试 ==="

# 检查 service_manager.sh 是否存在且可执行
if [ -x "$SCRIPT_DIR/service_manager.sh" ]; then
    echo "✓ service_manager.sh 存在且可执行"
else
    echo "✗ service_manager.sh 不存在或不可执行"
fi

# 检查 stop.sh 是否存在且可执行
if [ -x "$SCRIPT_DIR/stop.sh" ]; then
    echo "✓ stop.sh 存在且可执行"
else
    echo "✗ stop.sh 不存在或不可执行"
fi

echo ""

# 测试2: 模拟启动一个后台服务
echo "2. 模拟启动后台服务..."
nohup bash -c "sleep 30; echo '模拟服务运行中'" > /tmp/tele_adora_TEST_SERVICE.log 2>&1 &
test_pid=$!
echo "   启动测试服务 PID: $test_pid"
echo ""

# 等待1秒
sleep 1

# 测试3: 检查服务状态
echo "3. 测试服务状态查询..."
bash "$SCRIPT_DIR/service_manager.sh" status
echo ""

# 测试4: 测试日志查看
echo "4. 测试日志查看..."
bash "$SCRIPT_DIR/service_manager.sh" logs
echo ""

# 测试5: 停止测试服务
echo "5. 清理测试服务..."
kill $test_pid 2>/dev/null || true
rm -f /tmp/tele_adora_TEST_SERVICE.log 2>/dev/null || true
echo "   ✅ 测试服务已清理"
echo ""

echo "=== 服务管理功能测试完成 ==="
echo ""
echo "可用的服务管理命令："
echo "  bash stop.sh                   # 快速停止所有服务"
echo "  bash service_manager.sh        # 交互式管理界面"
echo "  bash service_manager.sh status # 查看服务状态"
echo "  bash service_manager.sh stop   # 停止所有服务"
echo "  bash service_manager.sh logs   # 查看日志"
