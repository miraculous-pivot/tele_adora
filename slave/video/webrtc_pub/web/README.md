# 🌐 Web 文件夹

这个文件夹包含WebRTC相关的测试文件和网页界面。

## 📂 文件说明

### WebRTC 测试文件

#### Python 服务器文件
- **`fixed_signaling_server.py`** - WebRTC信令服务器
  - **状态**: ⚠️ 实验性，存在ICE连接问题
  - **用途**: WebRTC双向通信的信令协调
  - **运行**: `python3 fixed_signaling_server.py`
  - **端口**: 默认8765 (WebSocket)

- **`video_only_stream.py`** - WebRTC视频流服务端
  - **状态**: ⚠️ 实验性，ICE失败率高
  - **用途**: 单向视频流的WebRTC实现
  - **依赖**: aiortc, opencv-python
  - **问题**: 本地环境ICE候选协商失败

#### HTML 界面文件
- **`debug_webrtc.html`** - WebRTC调试页面
  - **功能**: 显示WebRTC连接状态和调试信息
  - **用途**: 分析ICE候选和连接过程
  - **特点**: 详细的日志输出

- **`local_test_viewer.html`** - 本地WebRTC测试查看器
  - **功能**: 简化的WebRTC客户端
  - **目标**: 连接本地WebRTC服务器
  - **状态**: 存在连接稳定性问题

- **`video_only_viewer.html`** - WebRTC视频查看器
  - **功能**: 专用于视频流的WebRTC客户端
  - **特点**: 优化的视频显示界面
  - **问题**: ICE连接经常超时

## ⚠️ 重要说明

### WebRTC的问题
1. **ICE候选失败**: 在本地环境中WebRTC的ICE协商经常失败
2. **连接不稳定**: 即使连接成功，也容易断开
3. **复杂性高**: WebRTC需要STUN/TURN服务器支持
4. **调试困难**: 错误信息复杂，难以排查

### 为什么使用HTTP MJPEG替代
1. **简单可靠**: HTTP流更加稳定
2. **低延迟**: 对于实时监控场景延迟可接受
3. **兼容性好**: 所有浏览器都支持
4. **易于调试**: 标准HTTP协议，问题排查简单

## 🔧 使用说明

### 如果要测试WebRTC (不推荐)

1. **启动信令服务器**:
```bash
cd web
python3 fixed_signaling_server.py
```

2. **启动视频流服务器**:
```bash
python3 video_only_stream.py
```

3. **打开测试页面**:
```bash
# 在浏览器中打开
firefox debug_webrtc.html
# 或者
chromium local_test_viewer.html
```

### 推荐的替代方案

使用主项目的HTTP MJPEG流:
```bash
# 启动ROS2相机流节点
./script/start_camera_stream.sh

# 浏览器访问
# http://localhost:8080
```

## 📊 性能对比

| 技术 | 延迟 | 稳定性 | 复杂性 | 浏览器支持 | 推荐度 |
|------|------|--------|--------|------------|--------|
| WebRTC | 极低 | ⚠️ 低 | 🔴 高 | ✅ 好 | ❌ 不推荐 |
| HTTP MJPEG | 低 | ✅ 高 | 🟢 低 | ✅ 完美 | ✅ 推荐 |

## 🗑️ 清理建议

这些WebRTC文件主要用于技术验证，如果确认不需要WebRTC功能，可以考虑删除:

```bash
# 删除所有WebRTC测试文件
rm -rf web/
```

保留的理由:
- 技术参考价值
- 未来可能的WebRTC优化
- 学习和研究用途
