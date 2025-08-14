# 吸盘控制包说明

## 概述
`adora_suction_pump_control` 是一个ROS2包，用于控制Adora机器人的吸盘系统。该包通过订阅ROS话题来接收控制命令，并通过串口与吸盘硬件进行通信。

## 功能特性
- 基于ROS2话题的吸盘控制
- 支持多通道吸盘控制
- 提供状态反馈
- 可配置串口参数
- 安全的启动和关闭流程

## 话题接口

### 订阅话题
- `/adora_robot/suction_pump/cmd` (std_msgs/Bool)
  - 吸盘控制命令
  - `true`: 开启吸盘
  - `false`: 关闭吸盘

### 发布话题
- `/adora_robot/suction_pump/status` (std_msgs/Bool)
  - 吸盘当前状态反馈
  - `true`: 吸盘开启
  - `false`: 吸盘关闭

## 参数配置
- `port` (string, 默认: '/dev/ttyACM0')
  - 串口设备路径
- `baudrate` (int, 默认: 9600)
  - 串口波特率
- `channel` (int, 默认: 0)
  - 吸盘控制通道号

## 使用方法

### 1. 构建包
```bash
cd /home/feng/tele_adora/slave/adora
colcon build --packages-select adora_suction_pump_control
source install/setup.bash
```

### 2. 启动节点
使用launch文件（推荐）：
```bash
ros2 launch adora_suction_pump_control suction_pump_control.launch.py
```

直接运行节点：
```bash
ros2 run adora_suction_pump_control suction_pump_node
```

### 3. 自定义参数启动
```bash
ros2 run adora_suction_pump_control suction_pump_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=115200 \
  -p channel:=1
```

### 4. 测试吸盘控制
```bash
# 开启吸盘
ros2 topic pub /adora_robot/suction_pump/cmd std_msgs/Bool "data: true"

# 关闭吸盘
ros2 topic pub /adora_robot/suction_pump/cmd std_msgs/Bool "data: false"

# 查看状态反馈
ros2 topic echo /adora_robot/suction_pump/status
```

## 硬件要求
- 兼容Modbus RTU协议的继电器控制器
- 串口连接（USB转串口或直接串口）
- 支持的波特率：1200, 2400, 4800, 9600, 19200, 115200

## 故障排除

### 常见问题
1. **串口无法打开**
   - 检查设备路径是否正确
   - 确认用户有串口访问权限：`sudo usermod -a -G dialout $USER`
   - 重新登录或重启系统

2. **吸盘无响应**
   - 检查串口连接
   - 验证波特率设置
   - 确认硬件设备地址

3. **权限问题**
   ```bash
   sudo chmod 666 /dev/ttyACM0  # 临时解决方案
   ```

### 调试信息
节点会输出详细的调试信息，包括：
- 串口初始化状态
- 命令发送和接收情况
- 错误信息和警告

## 安全注意事项
- 程序退出时会自动关闭吸盘
- 建议在操作前确认吸盘位置安全
- 长时间运行时注意设备温度
