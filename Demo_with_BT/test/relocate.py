
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

try:
    from qrb_ros_amr_msgs.srv import Mapping
except Exception as e:
    print("ERROR: cannot import qrb_ros_amr_msgs.srv.Mapping.\n"
          "Please `source` your ROS 2 workspace where this interface is built.\n", e)
    raise


class AmrMappingServer(Node):
    def __init__(self):
        super().__init__('amr_mapping_server')
        # ✅ 注意：回调函数不要叫 handle，避免与 Node.handle 冲突
        self.srv = self.create_service(Mapping, '/amr_mapping', self.handle_mapping)
        self.get_logger().info(
            "Service [/amr_mapping] ready (qrb_ros_amr_msgs/srv/Mapping)"
        )

    # ✅ 使用不同名字的回调
    def handle_mapping(self, request, response):
        # 规则：cmd == 1 => True（成功），否则 False（失败）
        cmd_val = getattr(request, 'cmd', 0)
        ok = (cmd_val == 1)

        # 写入响应里的布尔/状态字段（根据你的 srv 响应定义，做常见字段名兼容）
        any_bool_set = False
        for field in ('success', 'ok', 'result', 'accepted'):
            if hasattr(response, field):
                setattr(response, field, ok)
                any_bool_set = True

        # 如有整型状态码字段（code/status/error_code），设为 0=成功, 1=失败
        for field in ('code', 'status', 'error_code'):
            if hasattr(response, field):
                setattr(response, field, 0 if ok else 1)

        # 如有 message 字段，附带文案
        if hasattr(response, 'message'):
            response.message = 'OK' if ok else 'FAIL'

        if not (any_bool_set or
                any(hasattr(response, f) for f in ('code', 'status', 'error_code', 'message'))):
            self.get_logger().warning(
                "Response has no known fields set. "
                "Customize handle_mapping() to match your Mapping.srv response definition."
            )

            self.get_logger().info(f"Received cmd={cmd_val} -> reply ok={ok}")
        return response


def main():
    rclpy.init()
    node = AmrMappingServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
