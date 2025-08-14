# Tele-Adora 启动脚本使用说明

## 概述
本目录包含了 Tele-Adora 遥操作系统的所有启动脚本，通过主脚本 `main_setup.sh` 可以方便地选择不同的控制模式。

## 文件结构
```
script/
├── main_setup.sh           # 主启动脚本（交互式菜单）
├── setup_keyboard.sh       # 键盘设备控制端脚本
├── setup_keyservice.sh     # 键盘控制服务端脚本
├── setup_vrservice.sh      # VR控制服务端脚本
├── setup_step.bash         # 足部控制器控制端脚本
└── README.md               # 本说明文件
```

## 使用方法

### 快速启动
```bash
cd script
./main_setup.sh
```

### 选择说明

1. **设备类型选择**
   - **本体 (Robot Body)**: 机器人本体端，负责执行控制指令
   - **遥操作设备 (Teleoperation Device)**: 控制端设备，负责发送控制指令

2. **本体控制方式**
   - **键盘控制服务**: 接收来自键盘的控制指令
   - **VR控制服务**: 接收来自VR设备的控制指令

3. **遥操作设备类型**
   - **足部控制器**: 使用足式编码器进行控制
   - **键盘设备**: 使用键盘进行双臂遥操作控制

## 脚本功能说明

### setup_keyboard.sh
- 启动键盘双臂遥操作控制端
- 构建 my_teleop_pkg 包
- 运行 bimanual_teleop 节点

### setup_keyservice.sh
- 启动键盘控制方式的服务端
- 包含底盘控制、升降电机控制、机械臂控制
- 启动多种摄像头（RealSense、Orbbec、ZED）
- 启动WebRTC视频流

### setup_vrservice.sh
- 启动VR控制方式的服务端
- 包含完整的机器人控制模块
- 包含云台控制、头部控制
- 启动多种摄像头和WebRTC视频流

### setup_step.bash
- 启动足式控制器控制端
- 构建编码器通信包
- 运行编码器数据读取节点

## 注意事项

1. **权限要求**: 脚本需要 sudo 权限来访问串口设备
2. **设备连接**: 确保所有硬件设备已正确连接
3. **环境依赖**: 确保 ROS2 环境已正确安装和配置
4. **网络配置**: WebRTC功能需要正确的网络配置

## 故障排除

1. **串口权限问题**: 
   ```bash
   sudo usermod -a -G dialout $USER
   # 重新登录后生效
   ```

2. **ROS2 环境问题**:
   ```bash
   source /opt/ros/humble/setup.bash  # 根据你的ROS2版本调整
   ```

3. **依赖包缺失**: 确保所有必要的ROS2包已安装

## 支持
如有问题，请检查各个模块的日志输出，或联系开发团队。
