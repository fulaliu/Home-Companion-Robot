#!/usr/bin/env python3
import asyncio
import websockets

async def test_ws_connection(uri: str):
    try:
        async with websockets.connect(uri) as ws:
            print(f"✅ Connected to {uri}")
            # 发送一个简单消息测试
            await ws.send("ping")
            print("✅ Sent: ping")
            # 等待服务端响应（如果有）
            try:
                response = await asyncio.wait_for(ws.recv(), timeout=5)
                print(f"✅ Received: {response}")
            except asyncio.TimeoutError:
                print("⚠ No response within 5 seconds (server may not echo messages)")
    except Exception as e:
        print(f"❌ Connection failed: {e}")

if __name__ == "__main__":
    uri = "ws://10.92.130.191:8765"  # 替换成你的服务端地址
    asyncio.run(test_ws_connection(uri))
