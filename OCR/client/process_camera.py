import asyncio
import json
import time
import base64
import cv2
import numpy as np
import websockets  # pip install websockets

SERVER_WS = "ws://10.64.38.233:8000/ws"  # 改成你的服务端地址
CHANNEL = "default"
CAM_INDEX = "/dev/video2"             # USB 摄像头序号
WIDTH, HEIGHT = 640, 480  # 分辨率
FPS = 10                  # 发送帧率
JPEG_QUALITY = 70         # 0-100

async def producer():
    # 连接 WebSocket
    async with websockets.connect(SERVER_WS, max_size=10 * 1024 * 1024) as ws:
        # 发送 join
        await ws.send(json.dumps({"type": "join", "role": "producer", "channel": CHANNEL}))

        cap = cv2.VideoCapture(CAM_INDEX)
        if WIDTH and HEIGHT:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        interval = 1.0 / FPS if FPS > 0 else 0.1

        async def recv_loop():
            try:
                async for message in ws:
                    try:
                        data = json.loads(message)
                        if data.get("type") == "ack":
                            print(f"[ACK] Saved at: {data.get('saved_path')}")
                    except Exception:
                        pass
            except Exception:
                pass
        recv_task = asyncio.create_task(recv_loop())

        try:
            while True:
                ok, frame = cap.read()
                if not ok:
                    await asyncio.sleep(0.2)
                    continue
                # BGR -> JPEG
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
                ok, buf = cv2.imencode(".jpg", frame, encode_param)
                if not ok:
                    continue
                b64 = base64.b64encode(buf.tobytes()).decode("ascii")
                payload = {
                    "type": "frame",
                    "channel": CHANNEL,
                    "ts": time.time(),
                    "image_b64": b64
                }
                await ws.send(json.dumps(payload))
                await asyncio.sleep(interval)
        finally:
            cap.release()
            recv_task.cancel()

if __name__ == "__main__":
    asyncio.run(producer())
