# Tele-Adora 服务管理指南

## 概述

Tele-Adora 系统提供了完整的服务管理解决方案，让您可以轻松控制和监控所有后台运行的服务。

## 🎯 问题解决

### 您提到的问题："静默启动的东西，应该怎么关"

**解决方案：**
1. **快速关闭**: `bash stop.sh` - 一键停止所有服务
2. **详细管理**: `bash service_manager.sh` - 完整的服务管理界面

## 🔄 服务运行模式

### 显示终端模式
```bash
bash start.sh --show-terminals
```
- ✅ 每个服务在独立的 gnome-terminal 窗口中运行
- ✅ 可以直接关闭窗口来停止服务
- ✅ 适合开发和调试
- ✅ 可以看到实时输出和错误信息

### 后台静默模式（默认）
```bash
bash start.sh
```
- ✅ 所有服务在后台静默运行
- ✅ 不会弹出终端窗口，界面更清洁
- ✅ 适合生产环境和长时间运行
- ❗ 需要使用专门的工具来停止服务

## 🛠️ 服务管理工具

### 1. 快速停止脚本 (stop.sh)

**最简单的解决方案：**
```bash
bash stop.sh
```

**功能：**
- 停止所有 ROS2 节点
- 停止所有后台进程
- 关闭所有相关终端窗口
- 停止构建进程
- 检查剩余进程并报告

### 2. 服务管理器 (service_manager.sh)

**完整的管理界面：**
```bash
bash service_manager.sh  # 交互式菜单
```

**命令行模式：**
```bash
bash service_manager.sh status     # 查看服务状态
bash service_manager.sh stop       # 停止所有服务
bash service_manager.sh logs       # 查看服务日志
bash service_manager.sh kill-all   # 强制终止所有进程
```

## 📊 服务状态监控

### 运行中的服务类型

1. **ROS2 节点**
   ```
   📡 /lifting_motor_control_node
   📡 /chassis_control_node
   📡 /gimbal_control_node
   📡 /arm_control_node
   📡 /vr_control_node
   ```

2. **后台进程**
   ```
   🔄 PID: 12345 - nohup ros2 launch adora_lifting_motor_control...
   🔄 PID: 12346 - nohup ros2 launch adora_chassis_bringup...
   ```

3. **终端窗口（显示模式）**
   ```
   🖥️ PID: 12347 - gnome-terminal --title="LIFTING MOTOR CONTROL"
   🖥️ PID: 12348 - gnome-terminal --title="CHASSIS CONTROL"
   ```

### 状态查看示例

```bash
$ bash service_manager.sh status

=== 检查运行中的 Tele-Adora 相关服务 ===

🔍 ROS2 节点:
  📡 /lifting_motor_control_node
  📡 /chassis_control_node
  📡 /gimbal_control_node

🔍 后台进程:
  🔄 PID: 12345 - nohup ros2 launch adora_lifting_motor_control
  🔄 PID: 12346 - nohup ros2 launch adora_chassis_bringup
```

## 📄 日志管理

### 日志文件位置
后台运行的服务日志保存在 `/tmp/` 目录：
```
/tmp/tele_adora_LIFTING_MOTOR_CONTROL.log
/tmp/tele_adora_CHASSIS_CONTROL.log
/tmp/tele_adora_GIMBAL_CONTROL.log
/tmp/tele_adora_ARM_CONTROL.log
/tmp/tele_adora_VR_CONTROL.log
/tmp/tele_adora_ORBBEC_CAMERA.log
/tmp/tele_adora_REALSENSE_CAMERAS.log
/tmp/tele_adora_ZED_CAMERA.log
/tmp/tele_adora_WEB_RTC.log
```

### 查看日志
```bash
# 查看所有服务的最新日志
bash service_manager.sh logs

# 手动查看特定服务日志
tail -f /tmp/tele_adora_LIFTING_MOTOR_CONTROL.log
```

## 🆘 故障排除

### 场景1: 服务无响应
```bash
# 1. 首先尝试正常停止
bash stop.sh

# 2. 如果还有进程残留，强制终止
bash service_manager.sh kill-all
```

### 场景2: 检查特定服务状态
```bash
# 查看详细状态
bash service_manager.sh status

# 查看特定服务日志
bash service_manager.sh logs
```

### 场景3: 系统混乱需要完全清理
```bash
# 强制终止所有相关进程（谨慎使用）
bash service_manager.sh kill-all

# 清理日志文件
rm -f /tmp/tele_adora_*.log
```

## 🔧 高级功能

### 选择性停止服务

服务管理器的交互式菜单提供：
1. 停止 ROS2 节点
2. 停止后台进程
3. 关闭终端窗口
4. 停止所有服务
5. 查看服务日志
6. 强制终止所有进程

### 实时监控

使用交互式菜单可以：
- 实时查看服务状态
- 持续监控进程变化
- 快速响应问题

## 💡 最佳实践

### 日常使用
1. **启动**: `bash start.sh`
2. **停止**: `bash stop.sh`
3. **监控**: `bash service_manager.sh status`

### 开发调试
1. **启动**: `bash start.sh --show-terminals`
2. **观察**: 在终端窗口中查看实时输出
3. **停止**: 直接关闭相关终端窗口

### 生产环境
1. **启动**: `bash start.sh` (后台模式)
2. **监控**: 定期运行 `bash service_manager.sh status`
3. **日志**: 定期检查 `bash service_manager.sh logs`
4. **停止**: `bash stop.sh`

### 故障处理
1. **轻度问题**: `bash stop.sh` → 重新启动
2. **严重问题**: `bash service_manager.sh kill-all` → 清理 → 重新启动

## 🎉 总结

现在您有了完整的服务管理解决方案：

- **🛑 快速停止**: `bash stop.sh`
- **📊 状态查看**: `bash service_manager.sh status`
- **📄 日志检查**: `bash service_manager.sh logs`
- **🔧 完整管理**: `bash service_manager.sh`
- **💥 强制清理**: `bash service_manager.sh kill-all`

无论是后台静默运行还是显示终端模式，您都可以轻松管理所有 Tele-Adora 服务！
