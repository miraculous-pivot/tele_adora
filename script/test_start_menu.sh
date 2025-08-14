#!/bin/bash

# 快速测试start.sh的菜单功能
# 模拟用户选择并快速退出

echo "=== 测试start.sh主菜单功能 ==="
echo ""

echo "模拟启动主菜单（将自动选择退出）..."
echo ""

# 使用 expect 来自动化交互，如果没有 expect 则手动测试
if command -v expect >/dev/null 2>&1; then
    expect -c "
        spawn bash start.sh
        expect \"请输入您的选择\"
        send \"0\r\"
        expect eof
    "
else
    echo "未安装 expect，请手动测试:"
    echo "运行: bash start.sh"
    echo "然后选择 0 退出"
    echo ""
    echo "您应该能看到:"
    echo "1. 设备配置状态检查"
    echo "2. 包含设备配置选项的主菜单"
    echo "3. 设备配置子菜单功能"
fi

echo ""
echo "测试完成！"
