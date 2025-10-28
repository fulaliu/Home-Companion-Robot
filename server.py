#!/usr/bin/env python3
import asyncio
import json
import threading
import argparse
import time
from typing import Set
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import websockets
from websockets.server import WebSocketServerProtocol
                
import asyncio, json, threading, time
from typing import Set
import numpy as np, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from websockets.server import WebSocketServerProtocol
import websockets

class RosWebsocketBridge(Node):
    def __init__(self, topic_in, topic_out, jpeg_quality=80, max_fps=30):
        super().__init__('ros_websocket_bridge')
        self.topic_in = topic_in
        self.topic_out = topic_out
        self.jpeg_quality = int(jpeg_quality)
        self.max_interval = 1.0/float(max_fps) if max_fps>0 else 0.0

        self._latest_jpeg = None
        self._latest_ts = 0.0
        self._ws_clients: Set[WebSocketServerProtocol] = set()   # <== 修复：避免与 Node._clients 冲突
        self._frame_event = asyncio.Event()

        self._warned_bad_step = False
        self._warned_bad_enc  = False

        self.image_sub = self.create_subscription(Image, self.topic_in, self._image_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, self.topic_out, 10)
        self.get_logger().info(f"Subscribed to {self.topic_in}, publishing cmd to {self.topic_out}")
        
        self._loop = None 
        
    
    def bind_asyncio_loop(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop

    def _image_cb(self, msg: Image):
        h, w = msg.height, msg.width
        enc = (msg.encoding or "").lower()
        is_nv21 = ('nv21' in enc)

        if enc and ('nv12' not in enc and 'nv21' not in enc and 'yuv420' not in enc) and not self._warned_bad_enc:
            self.get_logger().warn(f"Image encoding '{msg.encoding}', trying NV12/NV21 path.")
            self._warned_bad_enc = True

        data = np.frombuffer(msg.data, dtype=np.uint8)
        if data.size < h*w*3//2:
            self.get_logger().warn(f"[YUV] size {data.size} < expected {h*w*3//2} for {w}x{h}")
            return
        step = msg.step if getattr(msg, 'step', 0) else w
        if step < w and not self._warned_bad_step:
            self.get_logger().warn(f"[YUV] step {step} < width {w}, reshape may be invalid.")
            self._warned_bad_step = True
            step = w

        y_bytes = h*step
        y_flat = data[:y_bytes]
        y = y_flat.reshape((h, step))[:, :w]

        uv_bytes = (h//2)*step
        uv_flat = data[y_bytes:y_bytes+uv_bytes]
        uv_2c = uv_flat.reshape((h//2, step//2, 2))[:, : (w//2), :]

        code = cv2.COLOR_YUV2BGR_NV21 if is_nv21 else cv2.COLOR_YUV2BGR_NV12
        try:
            bgr = cv2.cvtColorTwoPlane(y, uv_2c, code)
        except Exception as e:
            if not hasattr(self, "_warned_twoplane"):
                self.get_logger().warn(f"[YUV] TwoPlane failed ({e}); fallback single-plane.")
                self._warned_twoplane = True
            uv_single = uv_2c.reshape((h//2, w))
            yuv = np.vstack((y, uv_single))
            bgr = cv2.cvtColor(yuv, code)

        ok, jpg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])       
        if not ok:
            self.get_logger().warn("cv2.imencode failed.")
            return
        self._latest_jpeg = jpg.tobytes()
        #self.get_logger().info(f"JPEG size={len(self._latest_jpeg)} bytes")
        now = time.time()
        if now - self._latest_ts >= self.max_interval:
            self._latest_ts = now
            try:                
                loop = self._loop
                if loop and loop.is_running():
                    loop.call_soon_threadsafe(self._frame_event.set)
                    #self.get_logger().info("Frame event triggered")
                else:
                    # 只有第一次告警，避免刷屏
                    if not hasattr(self, "_warned_no_loop"):
                        self.get_logger().warn(
                            "[WS] Asyncio loop not bound or not running; frame event not set."
                        )
                        self._warned_no_loop = True

            except RuntimeError:
                pass

    def publish_twist(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    async def register(self, ws: WebSocketServerProtocol):
        self._ws_clients.add(ws)   # <== 修复：使用 _ws_clients
        self.get_logger().info(f"Web client connected. Total={len(self._ws_clients)}")

    async def unregister(self, ws: WebSocketServerProtocol):
        self._ws_clients.discard(ws)  # <== 修复
        self.get_logger().info(f"Web client disconnected. Total={len(self._ws_clients)}")

    async def ws_handler(self, ws: WebSocketServerProtocol):
        await self.register(ws)
        try:
            async for msg in ws:
                try:
                    payload = json.loads(msg) if isinstance(msg, str) else {}
                except Exception:
                    payload = {"action": str(msg)}
                action = (payload.get("action") or "").lower()
                v_lin, v_ang = 0.0, 0.0
                if action == "forward": v_lin = 0.5
                elif action == "backward": v_lin = -0.5
                elif action == "left": v_ang = +0.3
                elif action == "right": v_ang = -0.3
                elif action == "stop": v_lin = v_ang = 0.0
                elif "linear" in payload or "angular" in payload:
                    v_lin, v_ang = float(payload.get("linear", 0.0)), float(payload.get("angular", 0.0))
                else:
                    continue
                self.publish_twist(v_lin, v_ang)
        finally:
            await self.unregister(ws)

    async def broadcast_frames(self):
        while True:
            await self._frame_event.wait()
            self._frame_event.clear()
            if not self._latest_jpeg or not self._ws_clients:
                continue
            dead = []
            for ws in list(self._ws_clients):   # <== 修复
                try:
                    await ws.send(self._latest_jpeg)
                except Exception:
                    dead.append(ws)
                #self.get_logger().info(f"Broadcasting frame to {len(self._ws_clients)} clients")
            for ws in dead:
                await self.unregister(ws)

def spin_ros(node: RosWebsocketBridge):
    rclpy.spin(node)


async def main_async(args):
    rclpy.init()
    node = RosWebsocketBridge(args.topic_in, args.topic_out, args.jpeg_quality, args.max_fps)

    # 1) 先拿到当前正在运行的 asyncio 事件循环
    loop = asyncio.get_running_loop()
    node.bind_asyncio_loop(loop)            # <== 关键：把 loop 传给 node

    # 2) 启动 ROS spin（后台线程）
    ros_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    ros_thread.start()

    # 3) 启动广播任务（不要直接 await，避免阻塞）
    asyncio.create_task(node.broadcast_frames())

    # 4) 启动 WebSocket 服务并常驻
    async with websockets.serve(node.ws_handler, args.host, args.port, max_size=None,subprotocols=None):
        node.get_logger().info(f"WebSocket listening on ws://{args.host}:{args.port}")
        await asyncio.Future()  # 永不完成，用来“挂起”主协程

    rclpy.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic-in", default="/cam0_stream1")
    parser.add_argument("--topic-out", default="/cmd_vel", help="可改为 /cmd_vel2")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--max-fps", type=int, default=30)
    args = parser.parse_args()
    asyncio.run(main_async(args))
