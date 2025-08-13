#!/usr/bin/env python3
"""
详细调试版本 - 检查每个连接阶段
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

class DebugCameraVideoStreamTrack(MediaStreamTrack):
    """调试版相机视频流轨道"""
    kind = "video"
    
    def __init__(self):
        super().__init__()
        self.latest_frame = None
        self.frame_ready = asyncio.Event()
        self.frame_count = 0
        self.last_frame_time = None
        
    def set_frame(self, frame):
        """从ROS回调设置新帧"""
        import time
        self.latest_frame = frame
        self.last_frame_time = time.time()
        self.frame_ready.set()
        
    async def recv(self):
        """异步获取视频帧"""
        try:
            # 等待新帧
            await asyncio.wait_for(self.frame_ready.wait(), timeout=0.1)
            self.frame_ready.clear()
            
            if self.latest_frame is not None:
                frame = self.latest_frame
                frame.pts = self.frame_count
                frame.time_base = av.Fraction(1, 30)
                self.frame_count += 1
                
                # 每100帧打印一次调试信息
                if self.frame_count % 100 == 0:
                    print(f"📹 已发送 {self.frame_count} 帧")
                
                return frame
        except asyncio.TimeoutError:
            pass
            
        # 生成测试帧
        return self.create_test_frame()
        
    def create_test_frame(self):
        """创建测试帧"""
        width, height = 640, 480
        image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 添加动画
        t = self.frame_count * 0.1
        center_x = int(width/2 + 100 * np.sin(t))
        center_y = int(height/2 + 50 * np.cos(t))
        
        # 绘制圆形
        y, x = np.ogrid[:height, :width]
        mask = (x - center_x)**2 + (y - center_y)**2 <= 50**2
        image[mask] = [255, 255, 255]
        
        # 添加帧计数
        try:
            import cv2
            cv2.putText(image, f'Frame: {self.frame_count}', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(image, 'Test Mode', (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        except ImportError:
            pass
        
        frame = av.VideoFrame.from_ndarray(image, format="rgb24")
        frame = frame.reformat(format="yuv420p")
        frame.pts = self.frame_count
        frame.time_base = av.Fraction(1, 30)
        self.frame_count += 1
        return frame

class DebugCameraWebRTCNode(Node):
    def __init__(self, video_track):
        super().__init__('debug_camera_webrtc_node')
        self.bridge = CvBridge()
        self.video_track = video_track
        self.frame_count = 0
        
        # 订阅相机图像
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('调试相机WebRTC节点已启动')
        
    def image_callback(self, msg):
        try:
            self.frame_count += 1
            
            # 每30帧打印一次
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'收到第 {self.frame_count} 帧相机图像')
            
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 转换为RGB
            import cv2
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # 创建VideoFrame
            frame = av.VideoFrame.from_ndarray(rgb_image, format="rgb24")
            frame = frame.reformat(format="yuv420p")
            
            # 设置到视频轨道
            self.video_track.set_frame(frame)
            
        except Exception as e:
            self.get_logger().error(f'处理图像失败: {e}')

async def run_debug_webrtc():
    """运行调试版WebRTC"""
    print("🚀 启动调试版相机WebRTC流")
    
    # 初始化ROS2
    rclpy.init()
    
    # 创建视频轨道和节点
    video_track = DebugCameraVideoStreamTrack()
    node = DebugCameraWebRTCNode(video_track)
    
    # ROS2线程
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
    
    # 详细的状态监听
    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"📡 连接状态变化: {pc.connectionState}")
        if pc.connectionState == "connected":
            print("🎉 WebRTC连接已建立！")
        elif pc.connectionState == "failed":
            print("❌ WebRTC连接失败")
        elif pc.connectionState == "disconnected":
            print("📵 WebRTC连接断开")
    
    @pc.on("iceconnectionstatechange") 
    async def on_iceconnectionstatechange():
        print(f"🧊 ICE连接状态变化: {pc.iceConnectionState}")
        if pc.iceConnectionState == "connected":
            print("✅ ICE连接成功！")
        elif pc.iceConnectionState == "failed":
            print("❌ ICE连接失败")
    
    @pc.on("icegatheringstatechange")
    async def on_icegatheringstatechange():
        print(f"🔍 ICE收集状态: {pc.iceGatheringState}")
    
    @pc.on("signalingstatechange")
    async def on_signalingstatechange():
        print(f"📶 信令状态: {pc.signalingState}")
    
    try:
        async with websockets.connect("ws://localhost:8080") as websocket:
            print("✅ 连接到信令服务器")
            
            # 发送offer
            print("🔄 创建Offer...")
            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)
            print("✅ LocalDescription已设置")
            
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
                        print("✅ 收到Answer，设置RemoteDescription...")
                        await pc.setRemoteDescription(RTCSessionDescription(
                            sdp=data["sdp"], 
                            type=data["type"]
                        ))
                        print("✅ RemoteDescription已设置，等待ICE连接...")
                        
                except Exception as e:
                    print(f"处理消息错误: {e}")
                    
                # 定期打印状态
                await asyncio.sleep(1)
                
    except Exception as e:
        print(f"连接错误: {e}")
    finally:
        await pc.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        asyncio.run(run_debug_webrtc())
    except KeyboardInterrupt:
        print("⏹️  用户中断")
    except Exception as e:
        print(f"❌ 错误: {e}")
