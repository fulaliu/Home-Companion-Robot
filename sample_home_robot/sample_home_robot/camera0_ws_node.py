#!/usr/bin/env python3
import asyncio, threading, time
import numpy as np, cv2, websockets
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class Camera0WS(Node):
    """
    /cam0_stream1 (NV12) -> JPEG（二进制） -> WebSocket(8765)
    - Python 3.12 兼容：在 run_ws() 中保存 asyncio loop + Event
    - ROS 回调线程通过 loop.call_soon_threadsafe 唤醒广播协程
    - 广播协程中按 max_fps 节流，避免“卡死式限流”
    """
    def __init__(self):
        super().__init__('camera0_ws')

        # 参数
        self.declare_parameter('input_topic', '/cam0_stream1')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8765)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('max_fps', 15)

        self.input_topic = self.get_parameter('input_topic').value
        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.max_interval = 1.0 / max(1, int(self.get_parameter('max_fps').value))

        self.get_logger().info(
            f"[cam0] subscribe {self.input_topic} -> ws://{self.host}:{self.port}, "
            f"jpgQ={self.jpeg_quality}, max_fps={1.0/self.max_interval:.1f}"
        )

        # 运行时对象（在 run_ws() 的 asyncio 事件循环里初始化）
        self._loop: asyncio.AbstractEventLoop | None = None
        self._frame_event: asyncio.Event | None = None

        self.latest_jpeg: bytes | None = None
        self._last_send_t = 0.0
        self._ws_clients = set()
        self._warned_bad_step = False
        self._warned_enc_hint = False

        # ROS 订阅
        self.sub = self.create_subscription(Image, self.input_topic, self._image_cb_nv12, 10)

    # NV12 专用解码回调
    def _image_cb_nv12(self, msg: Image):
        h, w = msg.height, msg.width
        enc = (msg.encoding or "").lower()
        if 'nv12' not in enc and 'yuv' not in enc and not self._warned_enc_hint:
            self.get_logger().warn(f"[cam0] encoding '{msg.encoding}', assuming NV12 path.")
            self._warned_enc_hint = True

        data = np.frombuffer(msg.data, dtype=np.uint8)
        if data.size < h * w * 3 // 2:
            self.get_logger().warn(f"[cam0] data size {data.size} < expected {h*w*3//2} for {w}x{h}")
            return

        step = msg.step if getattr(msg, 'step', 0) else w
        if step < w and not self._warned_bad_step:
            self.get_logger().warn(f"[cam0] step {step} < width {w}, using step=w")
            self._warned_bad_step = True
            step = w

        y_bytes = h * step
        y = data[:y_bytes].reshape((h, step))[:, :w]
        uv_flat = data[y_bytes:y_bytes + (h // 2) * step]
        # NV12: UV 交错（UVUV...）
        uv_2c = uv_flat.reshape((h // 2, step // 2, 2))[:, : w // 2, :]

        try:
            bgr = cv2.cvtColorTwoPlane(y, uv_2c, cv2.COLOR_YUV2BGR_NV12)
        except Exception as e:
            self.get_logger().warn(f"[cam0] TwoPlane failed {e}, fallback single-plane")
            uv_single = uv_2c.reshape((h // 2, w))
            yuv = np.vstack((y, uv_single))
            bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)

        ok, jpg = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self.get_logger().warn("[cam0] cv2.imencode failed.")
            return

        self.latest_jpeg = jpg.tobytes()
        # 从 ROS 回调线程安全地唤醒 asyncio 广播协程
        if self._loop and self._frame_event:
            try:
                self._loop.call_soon_threadsafe(self._frame_event.set)
            except Exception as e:
                self.get_logger().warn(f"[cam0] call_soon_threadsafe failed: {e}")

    async def _ws_handler(self, ws):
        self._ws_clients.add(ws)
        self.get_logger().info(f"[cam0] client connected. total={len(self._ws_clients)}")
        try:
            # cam0 视频只下行，不读消息
            while True:
                await asyncio.sleep(0.1)
        except Exception:
            pass
        finally:
            self._ws_clients.discard(ws)
            self.get_logger().info(f"[cam0] client disconnected. total={len(self._ws_clients)}")

    async def _broadcaster(self):
        while True:
            await self._frame_event.wait()
            self._frame_event.clear()

            now = time.time()
            # 按 max_fps 节流（这里节流而不是在回调里节流，避免“卡死限流”）
            if now - self._last_send_t < self.max_interval:
                continue
            self._last_send_t = now

            if not self.latest_jpeg or not self._ws_clients:
                continue
            dead = []
            for ws in list(self._ws_clients):
                try:
                    await ws.send(self.latest_jpeg)  # 直接发送二进制 JPEG
                except Exception:
                    dead.append(ws)
            for ws in dead:
                try:
                    await ws.close()
                except Exception:
                    pass
                self._ws_clients.discard(ws)

    async def run_ws(self):
        # 在 asyncio 主协程里保存 loop，并创建 Event
        self._loop = asyncio.get_running_loop()
        self._frame_event = asyncio.Event()

        self.get_logger().info(f"[cam0] WebSocket listening on ws://{self.host}:{self.port}")
        async with websockets.serve(self._ws_handler, self.host, self.port, max_size=None):
            asyncio.create_task(self._broadcaster())
            await asyncio.Future()  # keep running

def main():
    rclpy.init()
    node = Camera0WS()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    try:
        asyncio.run(node.run_ws())
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
