#!/usr/bin/env python3
"""
相机WebRTC流 - 最终版本
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import asyncio
import websockets
import json
import threading
import av
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaStreamTrack

class CameraVideoStreamTrack(MediaStreamTrack):
    """相机视频流轨道"""
    kind = "video"
    
    def __init__(self):
        super().__init__()
        self.latest_frame = None
        self.frame_ready = asyncio.Event()
        self.frame_count = 0
        
    def set_frame(self, frame):
        """从ROS回调设置新帧"""
        self.latest_frame = frame
        self.frame_ready.set()
        
    async def recv(self):
        """异步获取视频帧"""
        # 等待新帧或超时
        try:
            await asyncio.wait_for(self.frame_ready.wait(), timeout=1.0)
            self.frame_ready.clear()
            
            if self.latest_frame is not None:
                frame = self.latest_frame
                frame.pts = self.frame_count
                frame.time_base = av.Fraction(1, 30)
                self.frame_count += 1
                return frame
        except asyncio.TimeoutError:
            pass
            
        # 如果没有新帧，生成一个默认帧
        return self.create_default_frame()
        
    def create_default_frame(self):
        """创建默认帧（用于没有相机数据时）"""
        width, height = 640, 480
        image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 添加文本指示
        try:
            import cv2
            cv2.putText(image, 'Waiting for camera...', (50, height//2), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        except ImportError:
            # 简单的渐变背景
            for y in range(height):
                for x in range(width):
                    image[y, x] = [100, 100, 100]
        
        frame = av.VideoFrame.from_ndarray(image, format="rgb24")
        frame = frame.reformat(format="yuv420p")
        frame.pts = self.frame_count
        frame.time_base = av.Fraction(1, 30)
        self.frame_count += 1
        return frame

class CameraWebRTCNode(Node):
    def __init__(self, video_track):
        super().__init__('camera_webrtc_node')
        self.bridge = CvBridge()
        self.video_track = video_track
        
        # 订阅相机图像
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('相机WebRTC节点已启动')
        
    def image_callback(self, msg):
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 确保内存连续
            if not cv_image.flags['C_CONTIGUOUS']:
                cv_image = cv_image.copy()
            
            # 转换为RGB（WebRTC需要）
            import cv2
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # 创建VideoFrame
            frame = av.VideoFrame.from_ndarray(rgb_image, format="rgb24")
            frame = frame.reformat(format="yuv420p")
            
            # 设置到视频轨道
            self.video_track.set_frame(frame)
            
        except Exception as e:
            self.get_logger().error(f'处理图像失败: {e}')

async def run_webrtc_with_camera():
    """运行相机WebRTC流"""
    print("🚀 启动相机WebRTC流")
    
    # 初始化ROS2
    rclpy.init()
    
    # 创建视频轨道和节点
    video_track = CameraVideoStreamTrack()
    node = CameraWebRTCNode(video_track)
    
    # 在单独线程中运行ROS2
    def run_ros():
        try:
            rclpy.spin(node)
        except Exception as e:
            print(f"ROS2错误: {e}")
    
    ros_thread = threading.Thread(target=run_ros, daemon=True)
    ros_thread.start()
    
    # 创建WebRTC连接
    pc = RTCPeerConnection()
    pc.addTrack(video_track)
    
    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"📡 连接状态: {pc.connectionState}")
    
    try:
        async with websockets.connect("ws://localhost:8080") as websocket:
            print("✅ 连接到信令服务器")
            
            # 发送offer
            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)
            
            await websocket.send(json.dumps({
                "type": "offer",
                "sdp": offer.sdp
            }))
            print("✅ Offer已发送")
            
            # 处理信令
            async for message in websocket:
                try:
                    data = json.loads(message)
                    if data.get("type") == "answer":
                        print("✅ 收到Answer，建立连接中...")
                        await pc.setRemoteDescription(RTCSessionDescription(
                            sdp=data["sdp"], 
                            type=data["type"]
                        ))
                        print("✅ 相机视频流应该已经开始传输！")
                        
                except Exception as e:
                    print(f"处理消息错误: {e}")
                    
    except Exception as e:
        print(f"连接错误: {e}")
    finally:
        await pc.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        asyncio.run(run_webrtc_with_camera())
    except KeyboardInterrupt:
        print("⏹️  用户中断")
    except Exception as e:
        print(f"❌ 错误: {e}")
