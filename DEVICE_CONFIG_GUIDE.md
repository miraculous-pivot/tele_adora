# 设备配置系统使用指南

## 概述

本指南介绍如何使用Tele-Adora机器人的设备配置系统，该系统支持：
- 自动识别串口设备并建立映射关系
- 交互式配置机器人硬件组件
- 支持有/无吸盘两种配置模式
- 参数化启动ROS2节点

## 快速开始

### 1. 首次配置

```bash
# 运行设备识别脚本
cd /home/feng/tele_adora
bash script/device_identification.sh
```

该脚本将引导您：
1. 识别云台控制设备
2. 识别底盘控制设备  
3. 识别升降电机控制设备
4. 选择是否配置吸盘功能
5. 如启用吸盘，识别吸盘控制设备

### 2. 启动机器人服务

配置完成后，您可以直接使用现有的启动脚本：

```bash
# 键盘控制模式
bash script/setup_keyservice.sh

# VR控制模式  
bash script/setup_vrservice.sh

# 显示终端窗口模式
bash script/setup_keyservice.sh --show-terminals
bash script/setup_vrservice.sh --show-terminals
```

### 3. 参数化ROS2启动

所有硬件控制模块现在都支持参数传递：

- **adora_lifting_motor_control**: 支持dev参数指定设备端口
- **adora_chassis_bringup**: 支持dt_port参数指定设备端口  
- **ros2_head_control**: 支持port参数指定设备端口
- **adora_suction_pump_control**: 支持port参数指定设备端口

## 使用方法

### 首次运行

1. **运行服务脚本**:
   ```bash
   # 键盘控制模式
   bash script/setup_keyservice.sh
   
   # VR控制模式  
   bash script/setup_vrservice.sh
   ```

2. **设备配置过程**:
   - 系统会自动检测连接的串口设备
   - 提示选择是否启用吸盘功能
   - 为每个设备分配功能角色
   - 确认并保存配置

### 重新配置设备

如果需要重新配置设备分配：

```bash
# 删除配置文件
rm /tmp/tele_adora_device_config.sh

# 重新运行服务脚本
bash script/setup_keyservice.sh
```

### 手动配置设备

也可以单独运行设备配置脚本：

```bash
bash script/device_config.sh
```

## 设备映射示例

当前系统检测到的设备：
- `56D0001775` -> 提升电机控制
- `594C027770` -> 底盘控制
- `594C021229` -> 云台控制

如果启用吸盘功能，需要额外的设备用于吸盘控制。

## 配置文件格式

生成的配置文件 `/tmp/tele_adora_device_config.sh` 包含：

```bash
# 设备端口配置
LIFTING_MOTOR_PORT="/dev/serial/by-id/usb-1a86_USB_Single_Serial_56D0001775-if00"
CHASSIS_CONTROL_PORT="/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C027770-if00"
HEAD_CONTROL_PORT="/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C021229-if00"
SUCTION_PUMP_PORT="/dev/serial/by-id/usb-1a86_USB_Single_Serial_XXXXXXXX-if00"

# 功能开关
SUCTION_PUMP_ENABLED=true
```

## 启动参数

现在可以通过参数启动具体的ROS2模块：

```bash
# 提升电机控制
ros2 launch adora_lifting_motor_control adora_a2_max_ros2_node.py dev:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_56D0001775-if00

# 底盘控制
ros2 launch adora_chassis_bringup adora_a2_max_ros2.launch.py dt_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C027770-if00

# 云台控制
ros2 launch ros2_head_control head_control.launch.py port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_594C021229-if00

# 吸盘控制
ros2 launch adora_suction_pump_control suction_pump_control.launch.py port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_XXXXXXXX-if00
```

## 故障排除

### 设备未检测到

1. 检查USB连接
2. 验证设备权限: `ls -l /dev/serial/by-id/`
3. 重新运行配置: `rm /tmp/tele_adora_device_config.sh`

### 吸盘功能异常

1. 确认吸盘功能已启用: `grep SUCTION_PUMP_ENABLED /tmp/tele_adora_device_config.sh`
2. 检查吸盘设备端口配置
3. 验证串口权限

### 参数传递失败

确保启动文件已更新到最新版本，包含参数声明。

## 注意事项

1. **设备顺序**: USB设备的检测顺序可能会变化，建议使用by-id路径而非ttyACM编号
2. **权限管理**: 脚本会自动设置串口设备权限，需要sudo权限
3. **配置持久化**: 当前配置保存在临时文件中，系统重启后需要重新配置
