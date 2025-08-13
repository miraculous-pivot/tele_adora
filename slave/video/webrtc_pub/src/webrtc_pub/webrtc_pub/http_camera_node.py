#!/usr/bin/env python3
"""
HTTP MJPEG相机流        self.get_logger().info(f"🎥 HTTP相机流节点已启动")
        self.get_logger().info(f"📡 监听话题: {self.camera_topic}")
        self.get_logger().info(f"🌐 HTTP端口: {self.http_port}")
        self.get_logger().info(f"🔗 监听地址: {self.host}")
        self.get_logger().info(f"📏 目标分辨率: {self.target_width}x{self.target_height}")
        self.get_logger().info(f"🔄 JPEG质量: {self.jpeg_quality}%") ROS2节点实现
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
import urllib.parse

class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('http_camera_stream_node')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_count = 0
        
        # 配置参数
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('http_port', 8080)
        self.declare_parameter('target_width', 640)
        self.declare_parameter('target_height', 480)
        self.declare_parameter('host', '0.0.0.0')  # 监听所有IP地址
        self.declare_parameter('jpeg_quality', 80)  # JPEG质量
        
        # 获取参数
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.http_port = self.get_parameter('http_port').get_parameter_value().integer_value
        self.target_width = self.get_parameter('target_width').get_parameter_value().integer_value
        self.target_height = self.get_parameter('target_height').get_parameter_value().integer_value
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        
        # 创建订阅
        self.subscription = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 1
        )
        
        self.get_logger().info(f'🎥 HTTP相机流节点已启动')
        self.get_logger().info(f'📡 监听话题: {self.camera_topic}')
        self.get_logger().info(f'🌐 HTTP端口: {self.http_port}')
        self.get_logger().info(f'📏 目标分辨率: {self.target_width}x{self.target_height}')
        
    def image_callback(self, msg):
        try:
            self.frame_count += 1
            
            if self.frame_count % 60 == 0:
                self.get_logger().info(f'📸 已处理 {self.frame_count} 帧')
            
            # 处理图像
            if msg.encoding == "rgb8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 缩放到目标分辨率
            if cv_image.shape[:2] != (self.target_height, self.target_width):
                cv_image = cv2.resize(cv_image, (self.target_width, self.target_height))
            
            self.latest_frame = cv_image
            
        except Exception as e:
            self.get_logger().error(f'❌ 图像处理错误: {e}')

class StreamHandler(BaseHTTPRequestHandler):
    def __init__(self, *args, camera_node=None, **kwargs):
        self.camera_node = camera_node
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        if self.path == '/':
            self.send_main_page()
        elif self.path == '/stream':
            self.send_stream_page()
        elif self.path == '/mjpeg':
            self.send_mjpeg_stream()
        elif self.path == '/status':
            self.send_status()
        else:
            self.send_error(404)
    
    def send_main_page(self):
        # 获取服务器IP
        server_ip = self.camera_node.host if self.camera_node.host != '0.0.0.0' else 'localhost'
        
        html = f'''
<!DOCTYPE html>
<html>
<head>
    <title>🎥 ROS2相机直播控制台</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body {{ 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1a1a1a, #2d2d2d);
            color: white;
            margin: 0;
            padding: 20px;
            text-align: center;
            min-height: 100vh;
        }}
        .container {{
            max-width: 1200px;
            margin: 0 auto;
        }}
        h1 {{
            color: #4fc3f7;
            margin-bottom: 30px;
            font-size: 2.5em;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.5);
        }}
        .controls {{
            margin: 30px 0;
            display: flex;
            justify-content: center;
            flex-wrap: wrap;
            gap: 15px;
        }}
        .btn {{
            background: linear-gradient(45deg, #007acc, #0099ff);
            color: white;
            border: none;
            padding: 15px 30px;
            border-radius: 25px;
            cursor: pointer;
            font-size: 16px;
            text-decoration: none;
            display: inline-block;
            transition: all 0.3s ease;
            box-shadow: 0 4px 15px rgba(0, 122, 204, 0.3);
        }}
        .btn:hover {{
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(0, 122, 204, 0.4);
            background: linear-gradient(45deg, #005a99, #007acc);
        }}
        .btn-secondary {{
            background: linear-gradient(45deg, #555, #777);
        }}
        .btn-secondary:hover {{
            background: linear-gradient(45deg, #333, #555);
        }}
        .btn-fullscreen {{
            background: linear-gradient(45deg, #ff6b35, #f7931e);
        }}
        .btn-fullscreen:hover {{
            background: linear-gradient(45deg, #e55a2b, #de831a);
        }}
        .video-container {{
            margin: 30px 0;
            position: relative;
            display: inline-block;
        }}
        img {{ 
            width: 100%; 
            max-width: 800px;
            height: auto;
            border-radius: 15px;
            border: 3px solid #007acc;
            box-shadow: 0 8px 32px rgba(0, 122, 204, 0.3);
        }}
        .info {{
            background: rgba(51, 51, 51, 0.9);
            padding: 20px;
            border-radius: 15px;
            margin: 30px 0;
            text-align: left;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255, 255, 255, 0.1);
        }}
        .info-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 15px;
        }}
        .info-item {{
            padding: 10px;
            background: rgba(0, 122, 204, 0.1);
            border-radius: 8px;
            border-left: 4px solid #4fc3f7;
        }}
        .success {{ color: #4caf50; }}
        .highlight {{ color: #4fc3f7; }}
        .warning {{ color: #ff9800; }}
        .fullscreen {{
            position: fixed !important;
            top: 0 !important;
            left: 0 !important;
            width: 100vw !important;
            height: 100vh !important;
            z-index: 9999 !important;
            background: black !important;
            object-fit: contain !important;
            max-width: none !important;
            border: none !important;
            border-radius: 0 !important;
        }}
        .status-indicator {{
            display: inline-block;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: #4caf50;
            animation: pulse 2s infinite;
            margin-right: 8px;
        }}
        @keyframes pulse {{
            0% {{ opacity: 1; }}
            50% {{ opacity: 0.5; }}
            100% {{ opacity: 1; }}
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>🎥 ROS2相机实时流控制台</h1>
        
        <div class="controls">
            <a href="/stream" target="_blank" class="btn">🔗 打开纯流页面</a>
            <a href="/status" target="_blank" class="btn btn-secondary">📊 查看状态API</a>
            <button onclick="enterFullscreen()" class="btn btn-fullscreen">� 全屏相机画面</button>
            <button onclick="location.reload()" class="btn btn-secondary">🔄 刷新页面</button>
        </div>
        
        <div class="video-container">
            <img src="/mjpeg" alt="相机流" id="stream">
        </div>
        
        <div class="info">
            <h3><span class="status-indicator"></span>系统状态</h3>
            <div class="info-grid">
                <div class="info-item">
                    <div class="success">✅ ROS2 HTTP MJPEG流节点</div>
                </div>
                <div class="info-item">
                    <div class="highlight">📡 MJPEG流: http://{server_ip}:{self.camera_node.http_port}/mjpeg</div>
                </div>
                <div class="info-item">
                    <div class="highlight">🎯 分辨率: {self.camera_node.target_width}x{self.camera_node.target_height}</div>
                </div>
                <div class="info-item">
                    <div class="highlight">📸 已处理帧数: <span id="frameCount">{self.camera_node.frame_count}</span></div>
                </div>
                <div class="info-item">
                    <div class="highlight">🔄 JPEG质量: {self.camera_node.jpeg_quality}%</div>
                </div>
                <div class="info-item">
                    <div class="highlight">🌐 监听地址: {self.camera_node.host}:{self.camera_node.http_port}</div>
                </div>
            </div>
        </div>
        
        <script>
            const img = document.getElementById('stream');
            
            // 全屏功能
            function enterFullscreen() {{
                if (img.requestFullscreen) {{
                    img.requestFullscreen();
                }} else if (img.webkitRequestFullscreen) {{
                    img.webkitRequestFullscreen();
                }} else if (img.mozRequestFullScreen) {{
                    img.mozRequestFullScreen();
                }} else if (img.msRequestFullscreen) {{
                    img.msRequestFullscreen();
                }}
            }}
            
            // 监听加载状态
            img.onload = function() {{
                console.log('📸 帧已加载');
            }};
            
            img.onerror = function() {{
                console.log('❌ 帧加载失败，1秒后重试');
                setTimeout(() => {{
                    img.src = '/mjpeg?' + Date.now();
                }}, 1000);
            }};
            
            // 定期更新帧计数
            setInterval(() => {{
                fetch('/status')
                    .then(response => response.json())
                    .then(data => {{
                        document.getElementById('frameCount').textContent = data.frame_count;
                    }})
                    .catch(console.error);
            }}, 5000);
            
            // 键盘快捷键
            document.addEventListener('keydown', function(e) {{
                if (e.key === 'f' || e.key === 'F') {{
                    enterFullscreen();
                }}
                if (e.key === 'r' || e.key === 'R') {{
                    location.reload();
                }}
            }});
        </script>
    </div>
</body>
</html>
        '''
        
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(html.encode('utf-8'))
    
    def send_stream_page(self):
        # 纯流页面 - 全屏视频
        html = f'''
<!DOCTYPE html>
<html>
<head>
    <title>🎥 相机实时流</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        * {{
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }}
        body {{
            background: #000;
            overflow: hidden;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
        }}
        img {{
            width: 100vw;
            height: 100vh;
            object-fit: contain;
            background: #000;
        }}
        .overlay {{
            position: fixed;
            top: 20px;
            right: 20px;
            background: rgba(0, 0, 0, 0.7);
            color: white;
            padding: 10px 15px;
            border-radius: 5px;
            font-family: Arial, sans-serif;
            font-size: 14px;
            z-index: 1000;
            opacity: 0.8;
        }}
        .overlay:hover {{
            opacity: 1;
        }}
    </style>
</head>
<body>
    <img src="/mjpeg" alt="相机实时流" id="stream">
    <div class="overlay">
        🎥 实时流 | 按ESC返回
    </div>
    
    <script>
        const img = document.getElementById('stream');
        
        // 监听加载状态
        img.onload = function() {{
            console.log('📸 流帧已加载');
        }};
        
        img.onerror = function() {{
            console.log('❌ 流加载失败，重试中...');
            setTimeout(() => {{
                img.src = '/mjpeg?' + Date.now();
            }}, 1000);
        }};
        
        // ESC键返回主页
        document.addEventListener('keydown', function(e) {{
            if (e.key === 'Escape') {{
                window.close();
            }}
        }});
        
        // 防止右键菜单
        document.addEventListener('contextmenu', function(e) {{
            e.preventDefault();
        }});
    </script>
</body>
</html>
        '''
        
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(html.encode('utf-8'))
    
    def send_status(self):
        status = {
            'frame_count': self.camera_node.frame_count,
            'camera_topic': self.camera_node.camera_topic,
            'http_port': self.camera_node.http_port,
            'resolution': f'{self.camera_node.target_width}x{self.camera_node.target_height}',
            'jpeg_quality': self.camera_node.jpeg_quality,
            'has_frame': self.camera_node.latest_frame is not None
        }
        
        import json
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(status).encode('utf-8'))
    
    def send_mjpeg_stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        
        try:
            while True:
                if self.camera_node and self.camera_node.latest_frame is not None:
                    frame = self.camera_node.latest_frame.copy()
                    
                    # 编码为JPEG
                    ret, buffer = cv2.imencode('.jpg', frame, [
                        cv2.IMWRITE_JPEG_QUALITY, self.camera_node.jpeg_quality
                    ])
                    
                    if ret:
                        # 发送MJPEG帧
                        self.wfile.write(b'\r\n--frame\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(buffer)))
                        self.end_headers()
                        self.wfile.write(buffer.tobytes())
                else:
                    # 发送占位图像
                    placeholder = self.create_placeholder()
                    ret, buffer = cv2.imencode('.jpg', placeholder, [
                        cv2.IMWRITE_JPEG_QUALITY, self.camera_node.jpeg_quality
                    ])
                    
                    if ret:
                        self.wfile.write(b'\r\n--frame\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(buffer)))
                        self.end_headers()
                        self.wfile.write(buffer.tobytes())
                
                time.sleep(1/30)  # 30fps
                
        except Exception as e:
            self.camera_node.get_logger().error(f"❌ 流传输错误: {e}")
    
    def create_placeholder(self):
        # 创建占位图像
        import numpy as np
        frame = np.zeros((self.camera_node.target_height, self.camera_node.target_width, 3), dtype='uint8')
        
        # 添加文字
        cv2.putText(frame, 'Waiting for camera...', (50, 200), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100), 2)
        cv2.putText(frame, 'ROS2 Camera Stream', (50, 250), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f'Time: {time.strftime("%H:%M:%S")}', (50, 300), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        return frame
    
    def log_message(self, format, *args):
        # 禁用HTTP请求日志以减少输出
        pass

def create_handler(camera_node):
    def handler(*args, **kwargs):
        return StreamHandler(*args, camera_node=camera_node, **kwargs)
    return handler

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # 创建相机节点
        camera_node = CameraStreamNode()
        
        # ROS线程
        def run_ros():
            rclpy.spin(camera_node)
        
        ros_thread = threading.Thread(target=run_ros, daemon=True)
        ros_thread.start()
        
        # HTTP服务器
        handler = create_handler(camera_node)
        
        # 服务器绑定逻辑：
        # - 如果host是"0.0.0.0"，绑定到所有接口
        # - 如果host是特定IP，仍然绑定到0.0.0.0以确保兼容性
        # - 如果host是"localhost"，只绑定到localhost
        if camera_node.host == "localhost":
            bind_address = "localhost"
            server_description = "localhost（仅本地访问）"
        else:
            bind_address = "0.0.0.0"
            server_description = f"0.0.0.0（所有网络接口）"
        
        server = HTTPServer((bind_address, camera_node.http_port), handler)
        
        camera_node.get_logger().info(f"✅ HTTP服务器启动在 {server_description}:{camera_node.http_port}")
        
        # 显示访问地址
        if bind_address == "0.0.0.0":
            # 获取本机IP
            import socket
            try:
                # 连接到外部地址来获取本机IP
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("8.8.8.8", 80))
                local_ip = s.getsockname()[0]
                s.close()
                
                camera_node.get_logger().info(f"🎥 本地访问: http://localhost:{camera_node.http_port}")
                camera_node.get_logger().info(f"🌐 局域网访问: http://{local_ip}:{camera_node.http_port}")
                camera_node.get_logger().info(f"📺 流页面: http://localhost:{camera_node.http_port}/stream")
                camera_node.get_logger().info(f"📊 状态API: http://localhost:{camera_node.http_port}/status")
            except:
                camera_node.get_logger().info(f"🎥 访问地址: http://localhost:{camera_node.http_port}")
        else:
            camera_node.get_logger().info(f"🎥 访问地址: http://localhost:{camera_node.http_port}")
            camera_node.get_logger().info(f"📺 流页面: http://localhost:{camera_node.http_port}/stream")
            camera_node.get_logger().info(f"📊 状态API: http://localhost:{camera_node.http_port}/status")
        
        server.serve_forever()
        
    except KeyboardInterrupt:
        camera_node.get_logger().info("\n🛑 服务器停止")
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
