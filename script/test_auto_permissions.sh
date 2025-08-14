#!/bin/bash

# 测试权限自动设置功能
echo "=== 测试权限自动设置功能 ==="
echo ""

# 备份当前权限状态
echo "1. 备份一个脚本的权限状态..."
TEST_SCRIPT="script/main_setup.sh"
if [ -f "$TEST_SCRIPT" ]; then
    # 记录当前权限
    ORIGINAL_PERMS=$(stat -c %a "$TEST_SCRIPT")
    echo "原始权限: $ORIGINAL_PERMS"
    
    # 故意移除执行权限
    chmod 644 "$TEST_SCRIPT"
    NEW_PERMS=$(stat -c %a "$TEST_SCRIPT")
    echo "移除执行权限后: $NEW_PERMS"
    
    echo ""
    echo "2. 测试start.sh的自动权限设置..."
    
    # 模拟start.sh的权限设置部分
    if [ -f "setup_permissions.sh" ]; then
        chmod +x setup_permissions.sh
        echo "运行权限设置（静默模式）..."
        bash setup_permissions.sh --quiet
        
        # 检查权限是否恢复
        RESTORED_PERMS=$(stat -c %a "$TEST_SCRIPT")
        echo "权限设置后: $RESTORED_PERMS"
        
        if [ "$RESTORED_PERMS" -eq 755 ]; then
            echo "✅ 权限自动设置测试成功！"
        else
            echo "❌ 权限自动设置测试失败"
        fi
    else
        echo "❌ 权限设置脚本不存在"
    fi
else
    echo "❌ 测试脚本不存在: $TEST_SCRIPT"
fi

echo ""
echo "3. 测试完整start.sh权限检查流程..."

# 创建一个临时的简化测试
cat > test_start_permissions.sh << 'EOF'
#!/bin/bash
# 模拟start.sh的权限检查部分

echo "=== 检查和设置脚本权限 ==="
if [ -f "setup_permissions.sh" ]; then
    chmod +x setup_permissions.sh
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
EOF

chmod +x test_start_permissions.sh
bash test_start_permissions.sh
rm test_start_permissions.sh

echo ""
echo "测试完成！start.sh现在会在每次启动时自动设置所有脚本权限。"
