# 视频流发布配置说明

## 修改概述

已将所有视频流发布文件修改为**仅支持RGB图像模式**。每个相机现在只发布RGB彩色图像到指定端口。

## 修改的文件

### Launch文件（已修改）：

1. **multi_camera_stream.launch.py** - 多路摄像头RGB图像流
   - 每个相机只发布RGB图像
   - 端口：8081-8084

2. **configurable_multi_camera_stream.launch.py** - 可配置多路摄像头RGB图像流
   - 通过参数配置，只支持RGB图像
   - 端口：8081-8084

3. **dual_camera_stream.launch.py** - 双摄像头RGB图像流
   - 删除了深度和红外流
   - 只保留两个RGB图像流
   - 端口：8081-8082

4. **real_system_multi_camera.launch.py** - 实际系统多摄像头RGB图像流
   - 删除了所有深度和红外流
   - 只保留RGB彩色图像流
   - 端口：8081-8082

5. **http_camera_stream.launch.py** - 单摄像头RGB图像流
   - 端口：8080

### 新增文件：

6. **rgb_only_multi_camera.launch.py** - 纯RGB多摄像头流
   - 明确标识仅支持RGB模式
   - 端口：8081-8084

## 端口分配

- **8080**: 单摄像头RGB图像流
- **8081**: Camera1 RGB图像流
- **8082**: Camera2 RGB图像流  
- **8083**: Camera3 RGB图像流
- **8084**: 主摄像头RGB图像流

## 删除的模式

以下模式已完全删除：
- ❌ 深度图像流（depth）
- ❌ 红外图像流（infra1/infra2）
- ❌ 多模式支持

## 使用方法

### 启动多摄像头RGB流：
```bash
# 使用新的RGB专用launch文件
ros2 launch webrtc_pub rgb_only_multi_camera.launch.py

# 或使用修改后的现有文件
ros2 launch webrtc_pub multi_camera_stream.launch.py
ros2 launch webrtc_pub configurable_multi_camera_stream.launch.py
ros2 launch webrtc_pub real_system_multi_camera.launch.py
```

### 启动单摄像头RGB流：
```bash
ros2 launch webrtc_pub http_camera_stream.launch.py
```

### 使用控制脚本：
```bash
./multi_camera_control.sh start
```

## 话题订阅

所有节点现在只订阅以下类型的话题：
- `/camera/color/image_raw`
- `/camera1/color/image_raw`
- `/camera2/color/image_raw`
- `/camera3/color/image_raw`
- `/left_arm/camera/d405_left/color/image_rect_raw`

## 输出格式

- **图像格式**: MJPEG
- **传输协议**: HTTP
- **图像质量**: 75-85% JPEG压缩
- **分辨率**: 
  - 标准摄像头: 640x480
  - 主摄像头: 1280x720

所有配置现在专门用于RGB图像流，提供了更简洁和专注的视频流发布系统。
