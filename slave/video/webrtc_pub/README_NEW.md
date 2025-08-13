# WebRTC_Pub - ROS2 RGB图像HTTP流

## 📋 简介
ROS2包，专门用于将相机RGB图像通过HTTP MJPEG流进行实时传输。每个相机只发布RGB彩色图像到独立端口。

## ✨ 功能特性
- 🎥 **RGB图像流** - 专注于RGB彩色图像传输
- 🌐 **多相机支持** - 每个相机独立端口
- 📱 **浏览器观看** - 无需插件，直接访问
- 🎛️ **简化配置** - 一种模式，易于使用
- ⚙️ **可配置参数** - 端口、分辨率、质量等
- 🚀 **快速启动** - 一键启动多摄像头RGB流

## 🔄 系统架构
- **每个相机 = 一个端口 = 一个RGB流**
- **删除了深度和红外模式**
- **简化的配置和部署**

## 🛠️ 安装

```bash
# 编译包
cd /home/feng/tele_adora/webrtc_pub
colcon build

# 环境配置
source install/setup.bash
```

## 🚀 快速开始

### 方法1: 多摄像头RGB流 (推荐)
```bash
# 启动多摄像头RGB流
ros2 launch webrtc_pub multi_camera_stream.launch.py

# 或使用控制脚本
./multi_camera_control.sh start
```

### 方法2: 单摄像头RGB流
```bash
# 启动单摄像头RGB流
ros2 launch webrtc_pub http_camera_stream.launch.py
```

### 方法3: 可配置多摄像头
```bash
# 使用自定义参数启动
ros2 launch webrtc_pub configurable_multi_camera_stream.launch.py \
    camera1_topic:=/your_camera1/color/image_raw \
    camera1_port:=8081
```

## 📋 端口分配

| 端口 | 功能 | 话题 |
|-----|------|------|
| 8080 | 单摄像头RGB流 | `/camera/color/image_raw` |
| 8081 | Camera1 RGB流 | `/camera1/color/image_raw` |
| 8082 | Camera2 RGB流 | `/camera2/color/image_raw` |
| 8083 | Camera3 RGB流 | `/camera3/color/image_raw` |
| 8084 | 主摄像头RGB流 | `/camera/color/image_raw` |

## 🌐 访问地址

### RGB图像流访问
- **Camera1**: http://localhost:8081
- **Camera2**: http://localhost:8082  
- **Camera3**: http://localhost:8083
- **主摄像头**: http://localhost:8084
- **单摄像头**: http://localhost:8080

### 页面类型
- `/` - 主控制台页面
- `/stream` - 全屏流页面
- `/mjpeg` - 纯MJPEG流
- `/status` - 状态API

## 📂 可用的Launch文件

- `http_camera_stream.launch.py` - 单摄像头RGB流
- `dual_camera_stream.launch.py` - 双摄像头RGB流
- `multi_camera_stream.launch.py` - 多摄像头RGB流
- `configurable_multi_camera_stream.launch.py` - 可配置多摄像头RGB流
- `real_system_multi_camera.launch.py` - 实际系统多摄像头RGB流

## 🔧 配置参数

- `camera_topic` - RGB图像话题名称
- `http_port` - HTTP端口号
- `target_width` - 目标宽度 (默认: 640)
- `target_height` - 目标高度 (默认: 480)
- `jpeg_quality` - JPEG质量 (默认: 80)
- `host` - 监听地址 (默认: "0.0.0.0")

## 📝 注意事项

1. **只支持RGB图像流** - 已删除深度和红外模式
2. **每个相机一个端口** - 简化的架构
3. **话题格式**: 必须是`sensor_msgs/Image`类型的RGB图像话题
4. **网络访问**: 设置`host=0.0.0.0`允许远程访问

## 🛠️ 故障排除

### 没有图像显示
```bash
# 检查相机话题
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
```

### 端口被占用
```bash
# 修改端口参数
ros2 launch webrtc_pub http_camera_stream.launch.py http_port:=8083
```

### 远程无法访问
- 确认`host`参数设置为`"0.0.0.0"`
- 检查防火墙设置
- 确认网络连通性

详细配置说明请参考：[RGB_STREAM_CONFIG.md](RGB_STREAM_CONFIG.md)
