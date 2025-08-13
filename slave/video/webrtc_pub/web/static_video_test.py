#!/usr/bin/env python3
"""
最简化的WebRTC测试节点 - 只发送静态图像
"""
import rclpy
from rclpy.node import Node
import as                    elif msg_type == "ice-candidate":
                        print("✅ 收到ICE candidate")
                        candidate = data.get("candidate")
                        if candidate:
                            try:
                                await pc.addIceCandidate(candidate)
                                print("✅ ICE candidate已添加")
                            except Exception as ice_error:
                                print(f"⚠️  ICE candidate添加失败: {ice_error}")
                        else:
                            print("⚠️  收到空的ICE candidate")rt websockets
import json
import threading
import av
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate
from aiortc.contrib.media import MediaStreamTrack

class StaticVideoStreamTrack(MediaStreamTrack):
    """发送静态测试图像的视频轨道"""
    kind = "video"
    
    def __init__(self):
        super().__init__()
        # 创建一个简单的测试图像 (640x480, RGB)
        self.frame_count = 0
        
    async def recv(self):
        """生成测试视频帧"""
        # 创建一个彩色渐变测试图像
        width, height = 640, 480
        
        # 创建渐变背景
        image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 添加颜色渐变
        for y in range(height):
            for x in range(width):
                image[y, x, 0] = (x * 255) // width  # Red gradient
                image[y, x, 1] = (y * 255) // height  # Green gradient
                image[y, x, 2] = ((x + y) * 255) // (width + height)  # Blue gradient
        
        # 添加帧计数器文本效果
        center_x, center_y = width // 2, height // 2
        radius = 50 + (self.frame_count % 100)
        
        # 简单的圆形动画
        cv2_available = True
        try:
            import cv2
            # 添加圆形和文本
            cv2.circle(image, (center_x, center_y), radius, (255, 255, 255), 3)
            cv2.putText(image, f'Frame: {self.frame_count}', (50, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        except ImportError:
            # 如果没有cv2，就添加简单的方块
            x1, y1 = center_x - radius, center_y - radius
            x2, y2 = center_x + radius, center_y + radius
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(width, x2), min(height, y2)
            image[y1:y2, x1:x2] = [255, 255, 255]
        
        self.frame_count += 1
        
        # 转换为VideoFrame
        frame = av.VideoFrame.from_ndarray(image, format="rgb24")
        frame = frame.reformat(format="yuv420p")
        frame.pts = self.frame_count
        frame.time_base = av.Fraction(1, 30)  # 30 FPS
        
        # 控制帧率
        await asyncio.sleep(1/30)  # 30 FPS
        return frame

async def run_signaling(pc, video_track):
    """WebRTC信令处理"""
    pc.addTrack(video_track)
    
    uri = "ws://localhost:8080"
    try:
        async with websockets.connect(uri) as websocket:
            print("✅ 已连接到信令服务器")
            
            # 创建并发送 Offer
            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)
            
            offer_message = {
                "type": "offer",
                "sdp": offer.sdp
            }
            
            await websocket.send(json.dumps(offer_message))
            print("✅ Offer已发送")
            
            # 监听信令消息
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
                        print("✅ RemoteDescription已设置")
                        
                    elif msg_type == "ice-candidate":
                        print("✅ 收到ICE candidate")
                        candidate = data.get("candidate")
                        if candidate:
                            ice_candidate = RTCIceCandidate(
                                candidate["candidate"],
                                candidate["sdpMid"]
                            )
                            ice_candidate.sdpMLineIndex = candidate.get("sdpMLineIndex")
                            await pc.addIceCandidate(ice_candidate)
                            
                except Exception as e:
                    print(f"❌ 处理消息错误: {e}")
                    
    except Exception as e:
        print(f"❌ 信令连接错误: {e}")

def main():
    print("🚀 启动静态视频WebRTC测试")
    
    # 创建视频轨道和PeerConnection
    video_track = StaticVideoStreamTrack()
    pc = RTCPeerConnection()
    
    # 添加连接状态监听
    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"📡 连接状态: {pc.connectionState}")
    
    @pc.on("iceconnectionstatechange") 
    async def on_iceconnectionstatechange():
        print(f"🧊 ICE连接状态: {pc.iceConnectionState}")
    
    # 运行信令
    try:
        asyncio.run(run_signaling(pc, video_track))
    except KeyboardInterrupt:
        print("⏹️  用户中断")
    finally:
        print("🛑 关闭连接")
        try:
            asyncio.run(pc.close())
        except:
            pass

if __name__ == '__main__':
    main()
