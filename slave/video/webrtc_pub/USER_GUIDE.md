# 🎥 多视频话题HTTP流系统

这是一个基于ROS2的多摄像头HTTP MJPEG流发布系统，根据用户指定的配置实现4路视频话题的获取和发布。

## 📋 系统配置

系统支持以下4路视频话题的HTTP流发布：

| 摄像头 | ROS话题 | 分辨率 | HTTP端口 | 访问地址 |
|--------|---------|--------|----------|----------|
| ZED相机 | `/zed/zed_node/rgb/image_rect_color` | 960x540 | 8081 | http://localhost:8081 |
| 相机1 | `/camera1/camera1/color/image_rect_raw` | 848x480 | 8082 | http://localhost:8082 |
| 相机2 | `/camera2/camera2/color/image_rect_raw` | 848x480 | 8083 | http://localhost:8083 |
| 主相机 | `/camera/color/image_raw` | 640x480 | 8084 | http://localhost:8084 |

## 🚀 快速启动

### 单相机模式
启动单个相机的HTTP流服务：
```bash
./start_single_camera.sh
```

### 多相机模式
同时启动4路相机的HTTP流服务：
```bash
./start_multi_camera.sh
```

### 查看帮助
```bash
./start_single_camera.sh help
./start_multi_camera.sh help
```

## 🌐 访问方式

每个相机流可以通过以下方式访问：

### 主页面 (带控制界面)
- http://localhost:端口/

### 纯流页面 (全屏播放)
- http://localhost:端口/stream

### 状态API (JSON数据)
- http://localhost:端口/status

## 📦 Launch文件

系统提供以下launch文件：

### 1. 单相机启动
```bash
ros2 launch webrtc_pub http_camera_stream.launch.py
```

### 2. 多相机启动
```bash
ros2 launch webrtc_pub specified_multi_camera.launch.py
```

## 🛠 构建说明

如果需要重新构建包：
```bash
cd /home/feng/tele_adora/slave/video/webrtc_pub
colcon build --packages-select webrtc_pub
source install/setup.bash
```

## 🔧 自定义配置

如需修改配置，可以编辑launch文件：
- `src/webrtc_pub/launch/specified_multi_camera.launch.py` - 多相机配置
- `src/webrtc_pub/launch/http_camera_stream.launch.py` - 单相机配置

可调整的参数：
- `camera_topic`: ROS图像话题名称
- `http_port`: HTTP服务端口
- `target_width`: 目标宽度
- `target_height`: 目标高度
- `jpeg_quality`: JPEG压缩质量 (0-100)
- `host`: 监听地址 ('0.0.0.0' 或 'localhost')

## 💡 使用技巧

### 1. 快捷键
- **F键**: 全屏播放
- **R键**: 刷新页面
- **ESC键**: 退出全屏 (在流页面)

### 2. 网络访问
系统默认配置为 `0.0.0.0`，支持局域网访问。可通过以下方式访问：
- 本地访问: http://localhost:端口
- 局域网访问: http://本机IP:端口

### 3. 故障排除
- 如果某个端口显示"等待相机"，请检查对应的ROS话题是否正在发布
- 使用 `ros2 topic list` 查看可用话题
- 使用 `ros2 topic echo 话题名称` 测试话题数据

## 📝 系统要求

- ROS2 (Humble推荐)
- Python 3.8+
- OpenCV (python3-opencv)
- cv_bridge
- sensor_msgs

## 🔍 状态监控

系统会显示以下状态信息：
- 已处理帧数
- 相机话题状态
- HTTP服务端口
- 图像分辨率
- JPEG质量设置

## 📞 技术支持

如遇问题，请检查：
1. ROS2环境是否正确设置
2. 相机话题是否正在发布
3. 端口是否被其他程序占用
4. 网络防火墙设置
