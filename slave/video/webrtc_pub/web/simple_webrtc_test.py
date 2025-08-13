#!/usr/bin/env python3
"""
简化版WebRTC测试 - 忽略ICE candidates
"""
import asyncio
import websockets
import json
import av
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaStreamTrack

class StaticVideoStreamTrack(MediaStreamTrack):
    """发送静态测试图像的视频轨道"""
    kind = "video"
    
    def __init__(self):
        super().__init__()
        self.frame_count = 0
        
    async def recv(self):
        """生成测试视频帧"""
        width, height = 640, 480
        
        # 创建渐变背景
        image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 添加颜色渐变和动画
        for y in range(height):
            for x in range(width):
                image[y, x, 0] = (x * 255) // width  # Red
                image[y, x, 1] = (y * 255) // height  # Green
                image[y, x, 2] = ((x + y + self.frame_count) * 255) // (width + height + 100)  # Blue with animation
        
        # 添加移动的方块
        center_x = (width // 2) + int(50 * np.sin(self.frame_count * 0.1))
        center_y = (height // 2) + int(50 * np.cos(self.frame_count * 0.1))
        
        # 绘制方块
        size = 50
        x1, y1 = max(0, center_x - size), max(0, center_y - size)
        x2, y2 = min(width, center_x + size), min(height, center_y + size)
        image[y1:y2, x1:x2] = [255, 255, 255]
        
        self.frame_count += 1
        
        # 转换为VideoFrame
        frame = av.VideoFrame.from_ndarray(image, format="rgb24")
        frame = frame.reformat(format="yuv420p")
        frame.pts = self.frame_count
        frame.time_base = av.Fraction(1, 30)
        
        await asyncio.sleep(1/30)  # 30 FPS
        return frame

async def run_webrtc():
    """运行简化的WebRTC测试"""
    print("🚀 启动简化WebRTC测试")
    
    # 创建PeerConnection和视频轨道
    pc = RTCPeerConnection()
    
    video_track = StaticVideoStreamTrack()
    pc.addTrack(video_track)
    
    # 连接状态监听
    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"📡 连接状态: {pc.connectionState}")
    
    @pc.on("iceconnectionstatechange") 
    async def on_iceconnectionstatechange():
        print(f"🧊 ICE状态: {pc.iceConnectionState}")
    
    try:
        async with websockets.connect("ws://localhost:8080") as websocket:
            print("✅ 连接到信令服务器")
            
            # 创建并发送offer
            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)
            
            await websocket.send(json.dumps({
                "type": "offer",
                "sdp": offer.sdp
            }))
            print("✅ Offer已发送")
            
            # 处理信令消息（忽略ICE candidates）
            async for message in websocket:
                try:
                    data = json.loads(message)
                    msg_type = data.get("type")
                    
                    if msg_type == "answer":
                        print("✅ 收到Answer")
                        await pc.setRemoteDescription(RTCSessionDescription(
                            sdp=data["sdp"], 
                            type=data["type"]
                        ))
                        print("✅ Answer已处理，等待连接建立...")
                        
                    elif msg_type == "ice-candidate":
                        print("🔄 忽略ICE candidate（简化模式）")
                        # 在简化模式下跳过ICE candidates
                        pass
                        
                except Exception as e:
                    print(f"❌ 处理消息错误: {e}")
                    
    except Exception as e:
        print(f"❌ 连接错误: {e}")
    finally:
        await pc.close()

if __name__ == '__main__':
    try:
        asyncio.run(run_webrtc())
    except KeyboardInterrupt:
        print("⏹️  用户中断")
    except Exception as e:
        print(f"❌ 错误: {e}")
