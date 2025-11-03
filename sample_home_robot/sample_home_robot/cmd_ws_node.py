#!/usr/bin/env python3
import asyncio, threading, json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import websockets

class CmdWS(Node):
    """
    仅负责：ws(8888) -> JSON 指令 -> /cmd_vel Twist
    支持：
      {"action":"forward/backward/left/right/stop"}
      或 {"linear":0.3, "angular":-0.2}
    """
    def __init__(self):
        super().__init__('cmd_ws')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8888)
        self.declare_parameter('cmd_topic', '/cmd_vel')

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.get_logger().info(f"[cmd] listening ws://{self.host}:{self.port} -> publish {self.cmd_topic}")

    def _publish_twist(self, lin, ang):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.pub.publish(msg)

    async def _ws_handler(self, ws):
        self.get_logger().info("[cmd] client connected")
        try:
            async for raw in ws:
                try:
                    payload = json.loads(raw) if isinstance(raw, str) else {}
                except Exception:
                    payload = {}
                action = (payload.get('action') or '').lower()
                lin, ang = 0.0, 0.0
                if action == 'forward': lin = 0.5
                elif action == 'backward': lin = -0.5
                elif action == 'left': ang = 0.3
                elif action == 'right': ang = -0.3
                elif action == 'stop': lin = ang = 0.0
                elif 'linear' in payload or 'angular' in payload:
                    lin = float(payload.get('linear', 0.0))
                    ang = float(payload.get('angular', 0.0))
                self._publish_twist(lin, ang)
        except Exception:
            pass
        finally:
            self.get_logger().info("[cmd] client disconnected")

    async def run_ws(self):
        async with websockets.serve(self._ws_handler, self.host, self.port, max_size=None):
            await asyncio.Future()

def main():
    rclpy.init()
    node = CmdWS()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    try:
        asyncio.run(node.run_ws())
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()