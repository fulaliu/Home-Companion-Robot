#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

try:
    from qrb_ros_amr_msgs.action import Cmd
except Exception as e:
    print("ERROR: cannot import qrb_ros_amr_msgs.action.Cmd.\n"
          "Please `source` your ROS 2 workspace where this interface is built.\n", e)
    raise


class CmdActionServer(Node):
    def __init__(self):
        super().__init__('cmd_action_server')
        self._server = ActionServer(
            self,
            Cmd,
            '/cmd',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info("Action [/cmd] ready (qrb_ros_amr_msgs/action/Cmd)")

    def goal_callback(self, goal_request):
        # 可选：只接受 command==5 的目标；若要全部接受，改为直接 ACCEPT
        cmd_val = getattr(goal_request, 'command', None)
        self.get_logger().info(f"Received goal: command={cmd_val}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel request received")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        # 可选：发布一次反馈（若反馈类型有可用字段）
        try:
            fb = Cmd.Feedback()
            # 常见字段名，如 progress；不存在也没关系
            if hasattr(fb, 'progress'):
                fb.progress = 1.0
            goal_handle.publish_feedback(fb)
        except Exception:
            pass

        # 立即标记成功
        goal_handle.succeed()

        # 构造结果，并把布尔结果设为 True（或 code/status 设为 0）
        result = Cmd.Result()
        assigned = False
        for bool_name in ('success', 'result', 'ok', 'accepted'):
            if hasattr(result, bool_name):
                setattr(result, bool_name, True)
                assigned = True

        for code_name in ('code', 'status'):
            if hasattr(result, code_name) and not assigned:
                setattr(result, code_name, 0)
                assigned = True

        self.get_logger().info("Goal succeeded, returning True result")
        return result


def main():
    rclpy.init()
    node = CmdActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

