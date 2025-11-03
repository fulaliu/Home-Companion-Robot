#!/usr/bin/env python3
import asyncio, threading, time
import numpy as np, cv2, websockets
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class Camera1WS(Node):
    """
    /image_raw(通用编码) -> JPEG（二进制） -> WebSocket(8766)
    支持 rgb8/bgr8/rgba8/bgra8/mono8/nv12/nv21/yuv420 等常见编码
    """
    def __init__(self):
        super().__init__('camera1_ws')

        self.declare_parameter('input_topic', '/image_raw')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8766)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('max_fps', 15)

        self.input_topic = self.get_parameter('input_topic').value
        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.max_interval = 1.0 / max(1, int(self.get_parameter('max_fps').value))

        self.get_logger().info(
            f"[cam1] subscribe {self.input_topic} -> ws://{self.host}:{self.port}, "
            f"jpgQ={self.jpeg_quality}, max_fps={1.0/self.max_interval:.1f}"
        )

        self._loop: asyncio.AbstractEventLoop | None = None
        self._frame_event: asyncio.Event | None = None

        self.latest_jpeg: bytes | None = None
        self._last_send_t = 0.0
        self._ws_clients = set()
        self._warned = set()

        self.sub = self.create_subscription(Image, self.input_topic, self._image_cb_generic, 10)

    def _image_cb_generic(self, msg: Image):
        h, w = msg.height, msg.width
        enc = (msg.encoding or '').lower().strip()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        try:
            if 'rgb' in enc and '8' in enc and 'a' not in enc:
                step = msg.step if getattr(msg, 'step', 0) else (w * 3)
                arr = data.reshape((h, step))[:, :w*3].reshape((h, w, 3))
                bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)

            elif 'bgr' in enc and '8' in enc and 'a' not in enc:
                step = msg.step if getattr(msg, 'step', 0) else (w * 3)
                bgr = data.reshape((h, step))[:, :w*3].reshape((h, w, 3))

            elif 'rgba' in enc and '8' in enc:
                step = msg.step if getattr(msg, 'step', 0) else (w * 4)
                arr = data.reshape((h, step))[:, :w*4].reshape((h, w, 4))
                bgr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)

            elif 'bgra' in enc and '8' in enc:
                step = msg.step if getattr(msg, 'step', 0) else (w * 4)
                arr = data.reshape((h, step))[:, :w*4].reshape((h, w, 4))
                bgr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)

            elif 'mono8' in enc:
                step = msg.step if getattr(msg, 'step', 0) else w
                gray = data.reshape((h, step))[:, :w]
                bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            elif ('nv12' in enc) or ('nv21' in enc) or ('yuv420' in enc):
                is_nv21 = ('nv21' in enc)
                step = msg.step if getattr(msg, 'step', 0) else w
                if data.size < h * w * 3 // 2:
                    if 'size' not in self._warned:
                        self.get_logger().warn(f"[cam1] YUV size {data.size} < expected {h*w*3//2} for {w}x{h}")
                        self._warned.add('size')
                    return
                y_bytes = h * step
                y = data[:y_bytes].reshape((h, step))[:, :w]
                uv_flat = data[y_bytes:y_bytes + (h // 2) * step]
                uv_2c = uv_flat.reshape((h // 2, step // 2, 2))[:, : w // 2, :]
                code = cv2.COLOR_YUV2BGR_NV21 if is_nv21 else cv2.COLOR_YUV2BGR_NV12
                try:
                    bgr = cv2.cvtColorTwoPlane(y, uv_2c, code)
                except Exception:
                    uv_single = uv_2c.reshape((h // 2, w))
                    yuv = np.vstack((y, uv_single))
                    bgr = cv2.cvtColor(yuv, code)

            else:
                if enc not in self._warned:
                    self.get_logger().warn(f"[cam1] unsupported encoding '{msg.encoding}', try reshape as bgr8")
                    self._warned.add(enc or 'empty')
                step = msg.step if getattr(msg, 'step', 0) else (w * 3)
                if step < w * 3 or data.size < h * step:
                    return
                bgr = data.reshape((h, step))[:, :w*3].reshape((h, w, 3))

            ok, jpg = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if ok:
                self.latest_jpeg = jpg.tobytes()
                if self._loop and self._frame_event:
                    try:
                        self._loop.call_soon_threadsafe(self._frame_event.set)
                    except Exception as e:
                        self.get_logger().warn(f"[cam1] call_soon_threadsafe failed: {e}")

        except Exception as e:
            self.get_logger().warn(f"[cam1] convert error: {e}")

    async def _ws_handler(self, ws):
        self._ws_clients.add(ws)
        self.get_logger().info(f"[cam1] client connected. total={len(self._ws_clients)}")
        try:
            while True:
                await asyncio.sleep(0.1)
        except Exception:
            pass
        finally:
            self._ws_clients.discard(ws)
            self.get_logger().info(f"[cam1] client disconnected. total={len(self._ws_clients)}")

    async def _broadcaster(self):
        while True:
            await self._frame_event.wait()
            self._frame_event.clear()

            now = time.time()
            if now - self._last_send_t < self.max_interval:
                continue
            self._last_send_t = now

            if not self.latest_jpeg or not self._ws_clients:
                continue
            dead = []
            for ws in list(self._ws_clients):
                try:
                    await ws.send(self.latest_jpeg)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                try:
                    await ws.close()
                except Exception:
                    pass
                self._ws_clients.discard(ws)

    async def run_ws(self):
        self._loop = asyncio.get_running_loop()
        self._frame_event = asyncio.Event()

        self.get_logger().info(f"[cam1] WebSocket listening on ws://{self.host}:{self.port}")
        async with websockets.serve(self._ws_handler, self.host, self.port, max_size=None):
            asyncio.create_task(self._broadcaster())
            await asyncio.Future()

def main():
    rclpy.init()
    node = Camera1WS()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    try:
        asyncio.run(node.run_ws())
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()