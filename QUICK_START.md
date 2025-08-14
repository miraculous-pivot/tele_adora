# Tele-Adora 一键式启动使用指南

## 概述

Tele-Adora机器人系统现已集成完整的一键式启动功能，包括设备配置、系统启动和管理功能。

## 🚀 快速启动

### 最简单的使用方式
```bash
cd /home/feng/tele_adora
bash start.sh
```

系统将自动：
1. **自动设置脚本权限** - 确保所有脚本具有正确的执行权限
2. **检查设备配置状态** - 验证设备映射和连接状态
3. **首次使用时引导您完成设备配置** - 交互式设备识别向导
4. **显示直观的主菜单界面** - 清晰的功能分类和状态显示
5. **提供完整的系统管理功能** - 一站式控制和配置

## 📋 启动选项

### 基本启动选项
```bash
bash start.sh                    # 启动主菜单
bash start.sh --show-terminals   # 启动时显示终端窗口
bash start.sh --help             # 显示帮助信息
```

### 设备配置选项
```bash
bash start.sh --config-devices   # 直接运行设备识别向导
bash start.sh --demo-config      # 演示设备配置功能
bash start.sh --test-config      # 测试设备配置系统
```

## 🔧 主菜单功能

启动后，您将看到包含以下选项的主菜单：

### 1. 本体 (Robot Body)
- **键盘控制服务**: 启动键盘控制的机器人服务端
- **VR控制服务**: 启动VR控制的机器人服务端

### 2. 遥操作设备 (Teleoperation Device)  
- **足部控制器**: 启动足部控制器
- **键盘设备**: 启动键盘设备控制

### 3. 设备配置 (Device Configuration)
- **设备识别向导**: 交互式识别和配置串口设备
- **配置演示**: 查看设备配置功能演示
- **配置测试**: 测试设备配置系统完整性
- **设备状态**: 查看当前设备连接和配置状态

## 🔌 设备配置流程

### 首次使用
1. 运行 `bash start.sh`
2. 系统检测到无配置文件时，会提示选择：
   - **现在配置设备** (推荐) - 立即进入设备识别向导
   - **使用默认配置继续** - 使用硬编码的默认设备路径
   - **查看设备配置演示** - 了解配置功能

### 设备识别向导流程
1. **云台控制设备识别**
   - 拔出设备 → 记录状态 → 插入设备 → 自动识别

2. **底盘控制设备识别**
   - 拔出设备 → 记录状态 → 插入设备 → 自动识别

3. **升降电机设备识别**
   - 拔出设备 → 记录状态 → 插入设备 → 自动识别

4. **吸盘功能选择**
   - 选择是否启用吸盘功能
   - 如启用，继续识别吸盘控制设备

5. **配置保存**
   - 自动保存到 `config/device_mapping.txt`
   - 后续启动将自动使用该配置

## 📊 设备配置状态显示

主菜单会实时显示设备配置状态：

```
=== 设备配置状态 ===
✅ 设备配置: 已配置
✅ 吸盘功能: 已启用
```

或者：

```
=== 设备配置状态 ===
⚠️  设备配置: 未配置 (将使用默认设置)
💡 建议运行设备配置以获得最佳体验
```

## 🔄 重新配置设备

当设备连接发生变化时：

### 方法1: 通过主菜单
1. 运行 `bash start.sh`
2. 选择 `3) 设备配置`
3. 选择 `1) 运行设备识别向导`

### 方法2: 直接运行
```bash
bash start.sh --config-devices
```

### 方法3: 手动运行脚本
```bash
bash script/device_identification.sh
```

## 🎯 支持的机器人配置

### 有吸盘配置
```
HEAD_CONTROL_PORT=/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C021229-if00
CHASSIS_CONTROL_PORT=/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C027770-if00
LIFTING_MOTOR_PORT=/dev/serial/by-id/usb-1a86_USB_Single_Serial_56D0001775-if00
SUCTION_PUMP_ENABLED=true
SUCTION_PUMP_PORT=/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_5573530303535180C0C1-if00
```

### 无吸盘配置
```
HEAD_CONTROL_PORT=/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C021229-if00
CHASSIS_CONTROL_PORT=/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C027770-if00
LIFTING_MOTOR_PORT=/dev/serial/by-id/usb-1a86_USB_Single_Serial_56D0001775-if00
SUCTION_PUMP_ENABLED=false
```

## 🚨 故障排除

### 问题1: 设备配置加载失败
**解决方案:**
```bash
bash start.sh --test-config  # 运行诊断测试
bash start.sh --config-devices  # 重新配置设备
```

### 问题2: 设备未找到
**解决方案:**
1. 检查设备物理连接
2. 运行 `ls -la /dev/serial/by-id/` 查看可用设备
3. 重新运行设备识别向导

### 问题3: 权限问题
**解决方案:**
```bash
# 系统会自动设置权限，但如果遇到问题可以手动运行：
bash setup_permissions.sh  # 详细输出模式
bash setup_permissions.sh --quiet  # 静默模式
```

**注意:** `start.sh` 每次启动时都会自动检查和设置脚本权限，通常无需手动处理权限问题。

### 问题4: 脚本执行被拒绝
**解决方案:**
1. `start.sh` 会自动处理权限问题
2. 如果仍有问题，检查文件系统是否支持执行权限
3. 确认用户有权限修改文件

## 💡 最佳实践

1. **首次部署**: 始终运行设备配置向导
2. **设备更换**: 立即重新配置设备映射
3. **定期检查**: 使用配置测试功能验证系统状态
4. **备份配置**: 定期备份 `config/device_mapping.txt`
5. **自动化权限**: `start.sh` 会自动处理权限设置，无需手动管理

## � 服务管理

Tele-Adora 系统提供完整的服务管理功能，帮助您控制后台运行的服务：

### 快速关闭所有服务
```bash
bash stop.sh  # 一键停止所有 Tele-Adora 服务
```

### 服务管理器（完整功能）
```bash
# 交互式管理界面
bash service_manager.sh

# 命令行模式
bash service_manager.sh status     # 查看运行中的服务
bash service_manager.sh stop       # 停止所有服务
bash service_manager.sh logs       # 查看服务日志
bash service_manager.sh kill-all   # 强制终止所有进程
```

### 服务运行模式

#### 显示终端模式 (`--show-terminals`)
- 在可见的gnome-terminal窗口中运行
- 直接关闭窗口即可停止服务
- 适合开发和调试

#### 后台静默模式 (默认)
- 使用`nohup`在后台运行，无可见窗口
- 需要使用 `stop.sh` 或 `service_manager.sh` 来停止
- 适合生产环境和长时间运行

### 服务状态查看

服务管理器会显示：
- 🤖 **ROS2节点**: 运行中的 ROS2 节点列表
- 🔄 **后台进程**: nohup 启动的后台进程
- 🖥️ **终端窗口**: 显示模式的终端进程

### 日志查看

后台运行的服务日志保存在：
```
/tmp/tele_adora_LIFTING_MOTOR_CONTROL.log
/tmp/tele_adora_CHASSIS_CONTROL.log
/tmp/tele_adora_GIMBAL_CONTROL.log
/tmp/tele_adora_ARM_CONTROL.log
/tmp/tele_adora_VR_CONTROL.log
... (等等)
```

使用 `bash service_manager.sh logs` 可以查看所有日志的最新内容。

## �📖 相关文档

- `DEVICE_CONFIG_GUIDE.md` - 详细的设备配置系统指南
- `DEVICE_CONFIG_IMPLEMENTATION_REPORT.md` - 实现技术报告
- `TERMINAL_MODE_GUIDE.md` - 终端模式控制指南

## 🎉 总结

现在您可以通过简单的 `bash start.sh` 命令启动整个Tele-Adora系统：

1. ✅ **自动权限管理** - 启动时自动设置所有脚本执行权限
2. ✅ **智能配置检查** - 自动检测并引导设备配置
3. ✅ **直观主菜单** - 清晰的功能分类和状态显示
4. ✅ **一键式操作** - 无需记忆复杂的命令和路径
5. ✅ **完整功能** - 包含所有启动、配置和管理功能
6. ✅ **灵活配置** - 支持不同硬件配置的机器人

享受您的Tele-Adora机器人系统吧！🤖
