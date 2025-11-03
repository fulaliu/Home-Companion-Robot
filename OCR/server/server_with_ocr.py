import os
import time
import json
import base64
from typing import Dict, Set, Optional
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pathlib import Path

# === 按你的 OCR 代码要求的导入（保持一致，不做删改） ===
import glob
import cv2
import numpy as np
from paddleocr import PaddleOCR
import data_process

# === （源代码保留）应用与频道状态 ===
app = FastAPI(title="Camera WebSocket Hub")

class ChannelState:
    def __init__(self, name: str):
        self.name = name
        self.producers: Set[WebSocket] = set()
        self.viewers: Set[WebSocket] = set()
        self.last_frame_b64: Optional[str] = None
        self.last_ts: float = 0.0

channels: Dict[str, ChannelState] = {}

def get_channel(name: str) -> ChannelState:
    if name not in channels:
        channels[name] = ChannelState(name)
    return channels[name]

# === 静态文件（前端页面）（源代码保留，仅修复 index 返回写法） ===
BASE_DIR = Path(__file__).resolve().parent
WEB_DIR = BASE_DIR.parent / "web"
if WEB_DIR.exists():
    app.mount("/web", StaticFiles(directory=str(WEB_DIR), html=True), name="web")

@app.get("/")
def index():
    # 访问 http://<server>:8000/web/index.html
    if WEB_DIR.joinpath("index.html").exists():
        return HTMLResponse(f'<p>Open /web/index.html/web/index.html</a></p>')
    return HTMLResponse("<p>Upload web/index.html first.</p>")

# === 你提供的 OCR 初始化（原样参数，不改动） ===
# 同时保留你原文中的保存目录定义（即使本服务保存到 captures），不改变其语义
SAVE_DIR = Path("./received")
SAVE_DIR.mkdir(parents=True, exist_ok=True)

ocr = PaddleOCR(
    text_detection_model_name="PP-OCRv5_mobile_det",
    text_recognition_model_name="PP-OCRv5_mobile_rec",
    use_doc_orientation_classify=False,
    use_doc_unwarping=False,
    use_textline_orientation=False,
    text_rec_score_thresh=0.6,
)

# === 将你给出的 OCR 流程封装为函数（仅做变量对接，不改动流程） ===
def run_ocr_and_return_results(image_path: str) -> str:
    """
    严格按你的 OCR 逻辑：
      1) results = ocr.predict(img)
      2) for res in results: res.print(); res.save_to_json(filename)
      3) for file in glob.glob(f"{filename}/*.json"): ack = data_process.run_on_file(file, True, False)
      4) results = json.dumps(ack, ensure_ascii=False, indent=2); return results
    仅对 img/filename 两个变量做对接（从磁盘路径读取图像、以同名目录作为保存目录）。
    """
    # 对接你的变量 img：从磁盘读取刚保存的图片
    img = cv2.imread(image_path)

    # 调用你给出的推理接口
    results = ocr.predict(img)

    # 对接你的变量 filename：以图像“去扩展名”的路径作为保存 JSON 的目录
    filename = str(Path(image_path).with_suffix(""))

    # 按你的循环逻辑保存 OCR 结果 JSON
    for res in results:
        res.print()
        res.save_to_json(filename)

    # 按你的处理逻辑对生成的 JSON 逐个调用 data_process
    for file in glob.glob(f"{filename}/*.json"):
        ack = data_process.run_on_file(file, True, False)

    # 按你的返回格式，序列化为字符串 JSON 并返回
    results = json.dumps(ack, ensure_ascii=False, indent=2)
    return results

# === WebSocket 主入口（在 confirm 分支插入你的 OCR 步骤） ===
@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    role = None
    channel_name = "default"
    chan = get_channel(channel_name)

    try:
        # 首条消息必须是 join
        join_msg = await ws.receive_json()
        if join_msg.get("type") != "join":
            await ws.close(code=4000)
            return
        role = join_msg.get("role", "viewer")
        channel_name = join_msg.get("channel", "default")
        chan = get_channel(channel_name)

        if role == "producer":
            chan.producers.add(ws)
        else:
            role = "viewer"
            chan.viewers.add(ws)

        # 若是 viewer，新加入时可以推送一帧最新帧（可选）
        if role == "viewer" and chan.last_frame_b64:
            await safe_send_json(ws, {
                "type": "frame",
                "channel": channel_name,
                "ts": chan.last_ts,
                "image_b64": chan.last_frame_b64
            })

        # 主循环
        while True:
            message = await ws.receive_text()
            data = json.loads(message)
            mtype = data.get("type")

            if mtype == "frame" and role == "producer":
                # 更新并转发给所有 viewers
                img_b64 = data.get("image_b64")
                ts = data.get("ts", time.time())
                chan.last_frame_b64 = img_b64
                chan.last_ts = ts

                # 广播给所有 viewer
                await broadcast_json(chan.viewers, {
                    "type": "frame",
                    "channel": channel_name,
                    "ts": ts,
                    "image_b64": img_b64
                })

            elif mtype == "confirm" and role == "viewer":
                # 优先使用 viewer 回传的图像，保证用户所见即所得
                img_b64 = data.get("image_b64") or chan.last_frame_b64
                saved_path = save_image(channel_name, img_b64)

                # === 关键改动：在返回 ack 之前，执行你给出的 OCR 步骤，并带回你的 OCR 结果 ===
                ocr_json = run_ocr_and_return_results(saved_path)

                ack = {
                    "type": "ack",
                    "channel": channel_name,
                    "saved_path": saved_path,
                    "ts": time.time(),
                    # 将你的 OCR 返回结果字符串一起返回（新增字段，仅为返回你的结果所需）
                    "ocr_results": ocr_json
                }
                # 广播给 viewer 和 producer
                await broadcast_json(chan.viewers | chan.producers, ack)

            elif mtype == "ping":
                await safe_send_json(ws, {"type": "pong", "ts": time.time()})

            else:
                # 未知消息忽略或返回错误
                pass

    except WebSocketDisconnect:
        pass
    finally:
        # 清理
        if role == "producer":
            chan.producers.discard(ws)
        else:
            chan.viewers.discard(ws)

# === 工具函数（源代码保留） ===
async def broadcast_json(targets: Set[WebSocket], payload: dict):
    dead = []
    text = json.dumps(payload)
    for c in list(targets):
        try:
            await c.send_text(text)
        except Exception:
            dead.append(c)
    for d in dead:
        targets.discard(d)

async def safe_send_json(ws: WebSocket, payload: dict):
    try:
        await ws.send_text(json.dumps(payload))
    except Exception:
        pass
def save_image(channel_name: str, image_b64: Optional[str]) -> str:
    Path("captures").mkdir(exist_ok=True)
    save_dir = Path("captures") / channel_name
    save_dir.mkdir(parents=True, exist_ok=True)
    # 文件名
    ts_str = time.strftime("%Y%m%d_%H%M%S")
    file_path = save_dir / f"{ts_str}.jpg"
    if image_b64:
        raw = base64.b64decode(image_b64)
        file_path.write_bytes(raw)
    else:
        # 没有可保存帧也创建占位
        file_path.write_text("NO_FRAME")
    return str(file_path)
