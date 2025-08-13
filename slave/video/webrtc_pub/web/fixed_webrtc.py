#!/usr/bin/env python3
"""
修复ICE candidates交换的WebRTC版本
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
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate
from aiortc.contrib.media import MediaStreamTrack

class FixedCameraVideoStreamTrack(MediaStreamTrack):
    """修复版相机视频流轨道"""
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
        try:
            # 等待新帧，但不要等太久
            await asyncio.wait_for(self.frame_ready.wait(), timeout=0.1)
            self.frame_ready.clear()
            
            if self.latest_frame is not None:
                frame = self.latest_frame
                frame.pts = self.frame_count
                frame.time_base = av.Fraction(1, 30)
                self.frame_count += 1
                return frame
        except asyncio.TimeoutError:
            pass
            
        # 生成测试帧（如果没有相机帧）
        return self.create_test_frame()
        
    def create_test_frame(self):
        """创建测试帧"""
        width, height = 640, 480
        
        # 创建一个简单的动画
        t = self.frame_count * 0.1
        image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 彩色条纹
        for i in range(0, width, 20):
            color = [(i * 2) % 255, (i * 3) % 255, (i * 5) % 255]
            image[:, i:i+10] = color
            
        # 移动的圆点
        center_x = int(width/2 + 200 * np.sin(t))
        center_y = int(height/2 + 100 * np.cos(t))
        
        y, x = np.ogrid[:height, :width]
        mask = (x - center_x)**2 + (y - center_y)**2 <= 30**2
        image[mask] = [255, 255, 255]
        
        frame = av.VideoFrame.from_ndarray(image, format="rgb24")
        frame = frame.reformat(format="yuv420p")
        frame.pts = self.frame_count
        frame.time_base = av.Fraction(1, 30)
        self.frame_count += 1
        return frame

class FixedCameraWebRTCNode(Node):
    def __init__(self, video_track):
        super().__init__('fixed_camera_webrtc_node')
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
        self.get_logger().info('修复版相机WebRTC节点已启动')
        
    def image_callback(self, msg):
        try:
            self.frame_count += 1
            
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

async def run_fixed_webrtc():
    """运行修复版WebRTC"""
    print("🚀 启动修复版相机WebRTC流")
    
    # 初始化ROS2
    rclpy.init()
    
    # 创建视频轨道和节点
    video_track = FixedCameraVideoStreamTrack()
    node = FixedCameraWebRTCNode(video_track)
    
    # ROS2线程
    def run_ros():
        try:
            rclpy.spin(node)
        except Exception as e:
            print(f"ROS2错误: {e}")
    
    ros_thread = threading.Thread(target=run_ros, daemon=True)
    ros_thread.start()
    
    # 创建WebRTC连接 - 配置STUN服务器
    from aiortc import RTCConfiguration, RTCIceServer
    
    config = RTCConfiguration([
        RTCIceServer("stun:stun.l.google.com:19302"),
        RTCIceServer("stun:stun1.l.google.com:19302")
    ])
    
    pc = RTCPeerConnection(config)
    
    # 添加视频轨道
    pc.addTrack(video_track)
    
    # 状态监听
    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"📡 连接状态: {pc.connectionState}")
        if pc.connectionState == "connected":
            print("🎉 WebRTC连接成功！")
        elif pc.connectionState == "failed":
            print("❌ WebRTC连接失败")
    
    @pc.on("iceconnectionstatechange") 
    async def on_iceconnectionstatechange():
        print(f"🧊 ICE状态: {pc.iceConnectionState}")
        if pc.iceConnectionState == "connected":
            print("✅ ICE连接建立！")
        elif pc.iceConnectionState == "failed":
            print("❌ ICE连接失败")
    
    @pc.on("icegatheringstatechange")
    async def on_icegatheringstatechange():
        print(f"🔍 ICE收集状态: {pc.iceGatheringState}")
    
    ice_candidates_buffer = []
    
    @pc.on("icecandidate")
    async def on_icecandidate(candidate):
        if candidate:
            print(f"🧊 生成ICE候选: {candidate.candidate[:50]}...")
            ice_candidates_buffer.append(candidate)
    
    try:
        # 连接WebSocket
        async with websockets.connect("ws://localhost:8080") as websocket:
            print("✅ 连接到信令服务器")
            
            # 创建offer
            print("🔄 创建Offer...")
            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)
            
            # 等待ICE收集完成
            print("⏳ 等待ICE收集...")
            while pc.iceGatheringState != "complete":
                await asyncio.sleep(0.1)
            print("✅ ICE收集完成")
            
            # 发送完整的offer（包含ICE candidates）
            await websocket.send(json.dumps({
                "type": "offer",
                "sdp": pc.localDescription.sdp
            }))
            print("✅ Offer已发送")
            
            # 发送所有ICE candidates
            for candidate in ice_candidates_buffer:
                await websocket.send(json.dumps({
                    "type": "ice-candidate",
                    "candidate": candidate.candidate,
                    "sdpMLineIndex": candidate.sdpMLineIndex,
                    "sdpMid": candidate.sdpMid
                }))
                print(f"🧊 发送ICE候选")
            
            # 处理消息
            async for message in websocket:
                try:
                    data = json.loads(message)
                    
                    if data.get("type") == "answer":
                        print("✅ 收到Answer")
                        answer = RTCSessionDescription(
                            sdp=data["sdp"], 
                            type=data["type"]
                        )
                        await pc.setRemoteDescription(answer)
                        print("✅ RemoteDescription已设置")
                        
                    elif data.get("type") == "ice-candidate":
                        print("✅ 收到ICE候选")
                        candidate = RTCIceCandidate(
                            candidate=data["candidate"],
                            sdpMLineIndex=data["sdpMLineIndex"],
                            sdpMid=data["sdpMid"]
                        )
                        await pc.addIceCandidate(candidate)
                        print("✅ ICE候选已添加")
                        
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
        asyncio.run(run_fixed_webrtc())
    except KeyboardInterrupt:
        print("⏹️  用户中断")
    except Exception as e:
        print(f"❌ 错误: {e}")
