#!/usr/bin/env python3
"""
纯视频WebRTC流 - 避免音频相关问题
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
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate, RTCConfiguration, RTCIceServer
from aiortc.contrib.media import MediaStreamTrack

class VideoOnlyTrack(MediaStreamTrack):
    """仅视频轨道"""
    kind = "video"
    
    def __init__(self):
        super().__init__()
        self.frame = None
        self.frame_count = 0
        
    def set_frame(self, frame_data):
        self.frame = frame_data
        
    async def recv(self):
        if self.frame is not None:
            pts, time_base = await self.next_timestamp()
            frame = self.frame.copy()
            frame.pts = pts
            frame.time_base = time_base
            return frame
        else:
            return self.create_test_frame()
            
    def create_test_frame(self):
        """创建640x480测试帧"""
        width, height = 640, 480
        image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 简单渐变
        for y in range(height):
            for x in range(width):
                image[y, x] = [
                    int(128 + 127 * np.sin(x * 0.01 + self.frame_count * 0.1)),
                    int(128 + 127 * np.sin(y * 0.01 + self.frame_count * 0.1)),
                    int(128 + 127 * np.sin((x+y) * 0.005 + self.frame_count * 0.1))
                ]
        
        # 添加标识
        import cv2
        cv2.putText(image, f'Camera Frame {self.frame_count}', (50, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(image, 'WebRTC Stream', (50, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        frame = av.VideoFrame.from_ndarray(image, format="rgb24")
        frame = frame.reformat(format="yuv420p", width=640, height=480)
        self.frame_count += 1
        return frame

class VideoNode(Node):
    def __init__(self, track):
        super().__init__('video_node')
        self.bridge = CvBridge()
        self.track = track
        self.frame_count = 0
        
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 1
        )
        
        self.get_logger().info('🎥 视频节点已启动')
        
    def image_callback(self, msg):
        try:
            self.frame_count += 1
            
            if self.frame_count % 60 == 0:
                self.get_logger().info(f'📸 已处理 {self.frame_count} 帧视频')
            
            # 处理图像
            if msg.encoding == "rgb8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                import cv2
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # 确保是640x480
            import cv2
            if cv_image.shape[:2] != (480, 640):
                cv_image = cv2.resize(cv_image, (640, 480))
            
            # 创建帧
            frame = av.VideoFrame.from_ndarray(cv_image, format="rgb24")
            frame = frame.reformat(format="yuv420p")
            
            self.track.set_frame(frame)
            
        except Exception as e:
            self.get_logger().error(f'❌ 视频处理错误: {e}')

async def main():
    print("🚀 启动纯视频WebRTC流")
    
    rclpy.init()
    
    # 创建视频轨道
    track = VideoOnlyTrack()
    node = VideoNode(track)
    
    # ROS线程
    def run_ros():
        rclpy.spin(node)
    
    ros_thread = threading.Thread(target=run_ros, daemon=True)
    ros_thread.start()
    
    # WebRTC连接 - 添加TURN服务器
    pc = RTCPeerConnection(RTCConfiguration([
        RTCIceServer("stun:stun.l.google.com:19302"),
        RTCIceServer("stun:stun1.l.google.com:19302"),
    ]))
    pc.addTrack(track)
    
    # 连接状态跟踪
    connection_established = False
    
    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        nonlocal connection_established
        print(f"📡 连接: {pc.connectionState}")
        if pc.connectionState == "connected":
            print("🎉 视频连接成功！")
            connection_established = True
        elif pc.connectionState in ["failed", "disconnected", "closed"]:
            connection_established = False
    
    @pc.on("iceconnectionstatechange") 
    async def on_iceconnectionstatechange():
        print(f"🧊 ICE: {pc.iceConnectionState}")
    
    try:
        async with websockets.connect("ws://localhost:8080") as websocket:
            print("✅ 信令连接成功")
            
            async for message in websocket:
                try:
                    data = json.loads(message)
                    print(f"📨 收到: {data.get('type')}")
                    
                    if data.get("type") == "offer":
                        # 检查是否已经建立连接，避免重复处理
                        if pc.signalingState != "stable":
                            print("⚠️ 忽略重复的offer，连接正在进行中")
                            continue
                            
                        # 设置远程描述
                        await pc.setRemoteDescription(RTCSessionDescription(
                            sdp=data["sdp"], type=data["type"]
                        ))
                        print("✅ 远程描述已设置")
                        
                        # 创建answer
                        answer = await pc.createAnswer()
                        await pc.setLocalDescription(answer)
                        print("✅ 本地描述已设置")
                        
                        # 发送answer
                        await websocket.send(json.dumps({
                            "type": "answer",
                            "sdp": answer.sdp
                        }))
                        print("✅ Answer已发送")
                        
                    elif data.get("type") == "ice-candidate":
                        if data.get("candidate"):
                            try:
                                # 从浏览器接收的ICE候选格式
                                candidate_data = data["candidate"]
                                
                                # 如果是字符串格式，需要解析
                                if isinstance(candidate_data, str):
                                    # 解析candidate字符串 "candidate:xxx"
                                    parts = candidate_data.split()
                                    if len(parts) >= 6:
                                        foundation = parts[0].split(':')[1]
                                        component = int(parts[1])
                                        protocol = parts[2]
                                        priority = int(parts[3])
                                        ip = parts[4]
                                        port = int(parts[5])
                                        typ = parts[7] if len(parts) > 7 else "host"
                                        
                                        candidate = RTCIceCandidate(
                                            component=component,
                                            foundation=foundation,
                                            ip=ip,
                                            port=port,
                                            priority=priority,
                                            protocol=protocol,
                                            type=typ
                                        )
                                        
                                        if data.get("sdpMid"):
                                            candidate.sdpMid = data["sdpMid"]
                                        if data.get("sdpMLineIndex") is not None:
                                            candidate.sdpMLineIndex = data["sdpMLineIndex"]
                                        
                                        await pc.addIceCandidate(candidate)
                                        print("✅ ICE候选已添加")
                                    else:
                                        print(f"⚠️ ICE候选格式不正确: {candidate_data}")
                                else:
                                    print(f"⚠️ 未知ICE候选格式: {type(candidate_data)}")
                                    
                            except Exception as e:
                                print(f"⚠️ ICE候选处理失败: {e}")
                                print(f"   候选数据: {data.get('candidate')}")
                                # 继续处理，不要因为ICE候选失败而中断
                        
                except json.JSONDecodeError:
                    print("❌ JSON解析错误")
                except Exception as e:
                    print(f"❌ 消息处理错误: {e}")
                    # 不要在这里退出，继续处理下一个消息
                    
    except Exception as e:
        print(f"❌ 连接错误: {e}")
    finally:
        try:
            await pc.close()
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    asyncio.run(main())
