#include "nodes/dock_and_charge.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <qrb_ros_amr_msgs/action/cmd.hpp>   // 注意：接口头文件名通常小写：cmd.hpp
#include <chrono>
#include <memory>
#include <functional>

using CmdAction = qrb_ros_amr_msgs::action::Cmd;
using GoalHandleCmd = rclcpp_action::ClientGoalHandle<CmdAction>;
using namespace std::chrono_literals;

DockAndCharge::DockAndCharge(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{}

BT::PortsList DockAndCharge::providedPorts()
{
  return {
    // action 名：默认为 /cmd
    BT::InputPort<std::string>("action_name", "/cmd", "ROS2 action name"),
    // 要发送的 command 值：默认 5（泊车充电）
    BT::InputPort<int>("command", 5, "Cmd.goal.command"),
    // 超时毫秒：默认 30000 ms
    BT::InputPort<int>("timeout_ms", 30000, "overall timeout in milliseconds")
  };
}

// 内部状态单例（避免每次 tick 反复创建 ROS 资源）
namespace {
  struct DAC_State {
    rclcpp::Node::SharedPtr node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
    rclcpp_action::Client<CmdAction>::SharedPtr client;

    std::string action_name;
    CmdAction::Goal goal;

    std::shared_ptr<const CmdAction::Feedback> last_feedback;
    GoalHandleCmd::SharedPtr goal_handle;
    GoalHandleCmd::WrappedResult wrapped_result;
    bool result_ready = false;

    std::chrono::steady_clock::time_point start_tp;
    int timeout_ms = 30000;

    void reset_soft() {
      last_feedback.reset();
      goal_handle.reset();
      result_ready = false;
    }
    void reset_all() {
      reset_soft();
      node.reset();
      exec.reset();
      client.reset();
      action_name.clear();
      timeout_ms = 30000;
    }
  };

  DAC_State& dac_state()
  {
    static DAC_State S;
    return S;
  }

  void dac_feedback_cb(GoalHandleCmd::SharedPtr /*gh*/,
                       const std::shared_ptr<const CmdAction::Feedback>& feedback,
                       DAC_State* S)
  {
    S->last_feedback = feedback;
    if (feedback) {
      RCLCPP_INFO(S->node->get_logger(), "[DockAndCharge] feedback received");
      // 如有反馈字段，按需打印：例如 feedback->progress
    }
  }

  // 结果回调：函数定义独立
  void dac_result_cb(const GoalHandleCmd::WrappedResult& result, DAC_State* S)
  {
    S->wrapped_result = result;
    S->result_ready   = true;

    const char* code_str =
      (result.code == rclcpp_action::ResultCode::SUCCEEDED) ? "SUCCEEDED" :
      (result.code == rclcpp_action::ResultCode::ABORTED)   ? "ABORTED"   :
      (result.code == rclcpp_action::ResultCode::CANCELED)  ? "CANCELED"  : "UNKNOWN";

    RCLCPP_INFO(S->node->get_logger(), "[DockAndCharge] result callback: %s", code_str);

    // 如果需要访问 Result 自定义字段    // 如果需要访问 Result 自定义字段：
    // auto res = result.result; // std::shared_ptr<CmdAction::Result>
    // if (res) { ... }
  }
}

BT::NodeStatus DockAndCharge::onStart()
{
  auto& S = dac_state();

  // 读取端口参数
  const std::string action_name =
      getInput<std::string>("action_name").value_or(std::string("/cmd"));
  const int cmd = getInput<int>("command").value_or(5);
  S.timeout_ms = getInput<int>("timeout_ms").value_or(30000);

  // 初始化 ROS 2
  if (!rclcpp::ok()) {
    int argc = 0; char** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  if (!S.node) {
    S.node = std::make_shared<rclcpp::Node>("bt_dock_and_charge_client");
  }
  if (!S.exec) {
    S.exec = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    S.exec->add_node(S.node);
  }

  // 创建/更新 action client
  if (!S.client || S.action_name != action_name) {
    S.client = rclcpp_action::create_client<CmdAction>(S.node, action_name);
    S.action_name = action_name;
  }

  // 等待 action server（短暂等待，不阻塞）
  if (!S.client->wait_for_action_server(500ms)) {
    RCLCPP_WARN(S.node->get_logger(),
                "[DockAndCharge] action server %s not available yet", action_name.c_str());
    // 即使当前不可用，也返回 RUNNING；让上层继续 tick，后续再尝试。
    S.start_tp = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  // 构造 goal（泊车充电：command=5）
  CmdAction::Goal goal;
  goal.command = cmd;
  S.goal = goal;

  // 发送 goal（异步），注册反馈/结果回调
  rclcpp_action::Client<CmdAction>::SendGoalOptions options;


 // 绑定反馈回调：占位符接收 action 的两个回调参数；&S 作为额外参数传入
  options.feedback_callback = std::bind(
      dac_feedback_cb,
      std::placeholders::_1,   // GoalHandleCmd::SharedPtr
      std::placeholders::_2,   // std::shared_ptr<const CmdAction::Feedback>
      &S                       // 你的状态指针
  );

  // 绑定结果回调：占位符接收结果；&S 作为额外参数传入
  options.result_callback = std::bind(
      dac_result_cb,
      std::placeholders::_1,   // const GoalHandleCmd::WrappedResult&
      &S
  );

  auto future_goal_handle = S.client->async_send_goal(S.goal, options);

  // 注意：goal_handle 在 result_callback 之前就可能可用。如果需要取消，可在 onRunning 里拿到。
  // 此处不等待，进入 RUNNING。
  S.start_tp = std::chrono::steady_clock::now();

  RCLCPP_INFO(S.node->get_logger(),
              "[DockAndCharge] goal sent: action=%s command=%d timeout=%d ms",
              action_name.c_str(), cmd, S.timeout_ms);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DockAndCharge::onRunning()
{
  auto& S = dac_state();

  if (!S.node || !S.exec) {
    return BT::NodeStatus::FAILURE;
  }

  // 驱动回调
  S.exec->spin_some();

  // 超时判断
  const auto now = std::chrono::steady_clock::now();
  if (now - S.start_tp >= std::chrono::milliseconds(S.timeout_ms)) {
    RCLCPP_ERROR(S.node->get_logger(),
                 "[DockAndCharge] timeout after %d ms", S.timeout_ms);
    // 尝试取消（如果已获得 goal_handle）
    if (S.goal_handle) {
      (void)S.client->async_cancel_goal(S.goal_handle);
    }
    S.reset_soft();
    return BT::NodeStatus::FAILURE;
  }

  // 尚未有最终结果 → 继续等待
  if (!S.result_ready) {
    return BT::NodeStatus::RUNNING;
  }

  // 有结果 → 判断代码
  switch (S.wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(S.node->get_logger(), "[DockAndCharge] SUCCEEDED");
      S.reset_soft();
      return BT::NodeStatus::SUCCESS;
    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::CANCELED:
    default:
      RCLCPP_WARN(S.node->get_logger(), "[DockAndCharge] FAILED (abort/cancel/unknown)");
      S.reset_soft();
      return BT::NodeStatus::FAILURE;
  }
}

void DockAndCharge::onHalted()
{
  auto& S = dac_state();
  RCLCPP_WARN(S.node ? S.node->get_logger() : rclcpp::get_logger("DockAndCharge"),
              "[DockAndCharge] halted by parent, attempting to cancel goal");

  // 若已拿  // 若已拿到 goal_handle，尝试取消
  if (S.goal_handle && S.client) {
    (void)S.client->async_cancel_goal(S.goal_handle);
  }
  S.reset_soft();
}

// 工厂注册
void Register_DockAndCharge(BT::BehaviorTreeFactory& f)
{
  f.registerNodeType<DockAndCharge>("DockAndCharge");
}
