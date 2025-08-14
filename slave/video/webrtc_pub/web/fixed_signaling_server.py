#!/usr/bin/env python3
"""
修复ICE候选交换的信令服务器
"""
import asyncio
import websockets
import json
import logging
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import os

# 设置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SignalingServer:
    def __init__(self):
        self.clients = {}
        self.pending_offers = {}
        self.pending_ice_candidates = {}

    async def register(self, websocket):
        """注册新客户端"""
        client_id = id(websocket)
        self.clients[client_id] = websocket
        self.pending_ice_candidates[client_id] = []
        logger.info(f"📱 客户端 {client_id} 已连接")
        
        try:
            async for message in websocket:
                await self.handle_message(client_id, message)
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"📱 客户端 {client_id} 已断开")
        finally:
            # 清理
            if client_id in self.clients:
                del self.clients[client_id]
            if client_id in self.pending_offers:
                del self.pending_offers[client_id]
            if client_id in self.pending_ice_candidates:
                del self.pending_ice_candidates[client_id]

    async def handle_message(self, client_id, message):
        """处理客户端消息"""
        try:
            data = json.loads(message)
            msg_type = data.get("type")
            
            logger.info(f"📨 收到消息类型: {msg_type} 来自客户端 {client_id}")
            
            if msg_type == "offer":
                await self.handle_offer(client_id, data)
            elif msg_type == "answer":
                await self.handle_answer(client_id, data)
            elif msg_type == "ice-candidate":
                await self.handle_ice_candidate(client_id, data)
            else:
                logger.warning(f"⚠️  未知消息类型: {msg_type}")
                
        except json.JSONDecodeError as e:
            logger.error(f"❌ JSON解析错误: {e}")
        except Exception as e:
            logger.error(f"❌ 处理消息错误: {e}")

    async def handle_offer(self, client_id, data):
        """处理offer"""
        logger.info(f"🔄 处理Offer from {client_id}")
        
        # 存储offer
        self.pending_offers[client_id] = data
        
        # 转发给其他客户端
        offer_message = json.dumps(data)
        await self.broadcast_to_others(client_id, offer_message)
        logger.info(f"✅ Offer已转发")

    async def handle_answer(self, client_id, data):
        """处理answer"""
        logger.info(f"🔄 处理Answer from {client_id}")
        
        # 转发给其他客户端
        answer_message = json.dumps(data)
        await self.broadcast_to_others(client_id, answer_message)
        logger.info(f"✅ Answer已转发")

    async def handle_ice_candidate(self, client_id, data):
        """处理ICE候选"""
        logger.info(f"🧊 处理ICE候选 from {client_id}: {data.get('candidate', '')[:50]}...")
        
        # 存储ICE候选
        self.pending_ice_candidates[client_id].append(data)
        
        # 立即转发给其他客户端
        ice_message = json.dumps(data)
        await self.broadcast_to_others(client_id, ice_message)
        logger.info(f"✅ ICE候选已转发")

    async def broadcast_to_others(self, sender_id, message):
        """向除发送者外的所有客户端广播消息"""
        disconnected = []
        
        for client_id, websocket in self.clients.items():
            if client_id != sender_id:
                try:
                    await websocket.send(message)
                    logger.info(f"📤 消息已发送到客户端 {client_id}")
                except websockets.exceptions.ConnectionClosed:
                    logger.info(f"📵 客户端 {client_id} 连接已关闭")
                    disconnected.append(client_id)
                except Exception as e:
                    logger.error(f"❌ 发送消息到客户端 {client_id} 失败: {e}")
                    disconnected.append(client_id)
        
        # 清理断开的连接
        for client_id in disconnected:
            if client_id in self.clients:
                del self.clients[client_id]

class CORSHTTPRequestHandler(SimpleHTTPRequestHandler):
    """支持CORS的HTTP请求处理器"""
    
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        super().end_headers()
    
    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()
        
    def do_GET(self):
        """处理GET请求，确保HTML文件使用UTF-8编码"""
        if self.path.endswith('.html'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            
            # 读取HTML文件并以UTF-8编码发送
            try:
                file_path = self.path.lstrip('/')
                if not file_path:
                    file_path = 'index.html'
                
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                self.wfile.write(content.encode('utf-8'))
            except FileNotFoundError:
                self.send_error(404, "File not found")
            except Exception as e:
                self.send_error(500, f"Server error: {e}")
        else:
            # 对于其他文件类型，使用默认处理
            super().do_GET()

def run_http_server():
    """运行HTTP服务器"""
    try:
        # 获取脚本所在目录并切换到webrtc_pub目录
        script_dir = os.path.dirname(os.path.abspath(__file__))
        webrtc_pub_dir = os.path.join(script_dir, '..')
        os.chdir(webrtc_pub_dir)
        
        server = HTTPServer(('localhost', 8081), CORSHTTPRequestHandler)
        logger.info("🌐 HTTP服务器启动在 http://localhost:8081")
        server.serve_forever()
    except Exception as e:
        logger.error(f"❌ HTTP服务器错误: {e}")

async def main():
    """主函数"""
    # 启动HTTP服务器线程
    http_thread = threading.Thread(target=run_http_server, daemon=True)
    http_thread.start()
    
    # 创建信令服务器
    signaling_server = SignalingServer()
    
    # 启动WebSocket服务器
    logger.info("🚀 启动WebSocket信令服务器在 ws://localhost:8080")
    
    async with websockets.serve(
        signaling_server.register,
        "localhost",
        8080,
        ping_interval=30,
        ping_timeout=10
    ):
        logger.info("✅ 信令服务器已启动")
        logger.info("📝 HTTP服务: http://localhost:8081")
        logger.info("🔌 WebSocket服务: ws://localhost:8080")
        
        # 保持服务器运行
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("⏹️  服务器已停止")
    except Exception as e:
        logger.error(f"❌ 服务器错误: {e}")
