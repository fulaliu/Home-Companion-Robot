#include "nodes/relocate.h"
#include "common/logger.h"
#include <rclcpp/rclcpp.hpp>
#include <qrb_ros_amr_msgs/srv/mapping.hpp>  // 注意：你的包名/服务名
#include <chrono>
#include <memory>
#include <functional>

using namespace std::chrono_literals;
using MappingSrv = qrb_ros_amr_msgs::srv::Mapping;
using SharedFuture = rclcpp::Client<MappingSrv>::SharedFuture;


RelocateMapping::RelocateMapping(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{}

BT::PortsList RelocateMapping::providedPorts()
{
  return {
    // 输入端口：服务名（默认 /amr_mapping）
    BT::InputPort<std::string>("service_name", "/amr_mapping",
                               "ROS2 service name for Mapping"),
    // 输入端口：cmd（默认 1）
    BT::InputPort<int>("cmd", 1, "Mapping request cmd"),
    // 输入端口：超时时间（毫秒，默认 15000）
    BT::InputPort<int>("timeout_ms", 15000, "overall timeout in milliseconds")
  };
}

// ====== 内部状态（放在匿名命名空间，或成员中都可以） ======
namespace {
  struct RelocateState {
    rclcpp::Node::SharedPtr node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
    rclcpp::Client<qrb_ros_amr_msgs::srv::Mapping>::SharedPtr client;

    qrb_ros_amr_msgs::srv::Mapping::Request::SharedPtr req;
    rclcpp::Client<qrb_ros_amr_msgs::srv::Mapping>::SharedFuture future;

    std::shared_ptr<qrb_ros_amr_msgs::srv::Mapping::Response> resp;
    bool request_sent = false;
    bool result_ready = false;
    std::chrono::steady_clock::time_point start_tp;
    std::string service_name;
  };

  // 单例缓存，避免频繁创建 ROS 资源
  RelocateState& relocate_state()
  {
    static RelocateState s;
    return s;
  }

  void reset_state(RelocateState& s)
  {
    s.req.reset();
    s.resp.reset();
    s.future = {};
    s.request_sent = false;
    s.result_ready = false;
    s.service_name.clear();
  }

void mapping_response_cb(SharedFuture future, RelocateState* state)
{
  LOG("Relocate","mapping response cb");
  state->resp = future.get();      // std::shared_ptr<MappingSrv::Response>
  state->result_ready = true;
}
}

BT::NodeStatus RelocateMapping::onStart()
{
  auto& S = relocate_state();

  // 读取端口参数
  const std::string service_name =
      getInput<std::string>("service_name").value_or(std::string("/amr_mapping"));
  const int cmd = getInput<int>("cmd").value_or(1);
  const int timeout_ms = getInput<int>("timeout_ms").value_or(15000);

  // 初始化 ROS 2
  if (!rclcpp::ok())
  {
    int argc = 0; char** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  if (!S.node)
  {
    S.node = std::make_shared<rclcpp::Node>("bt_relocate_mapping_client");
  }
  if (!S.exec)
  {
    S.exec = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    S.exec->add_node(S.node);
  }
  // 创建/更新 client
  if (!S.client || S.service_name != service_name)
  {
    S.client = S.node->create_client<qrb_ros_amr_msgs::srv::Mapping>(service_name);
    S.service_name = service_name;
  }

  // 非阻塞地等待服务就绪（短暂等待；不就绪则 RUNNING）
  if (!S.client->wait_for_service(1ms))
  {
    RCLCPP_WARN(S.node->get_logger(),
      "[RelocateMapping] service %s not available yet", service_name.c_str());
    S.start_tp = std::chrono::steady_clock::now();
    // 本次不发送请求，等待下次 tick
    LOG("Relocate", "running" );
    return BT::NodeStatus::RUNNING;
  }

    // 构造请求并异步发送（只发送一次）
  S.req = std::make_shared<qrb_ros_amr_msgs::srv::Mapping::Request>();
  S.req->cmd = cmd;  // 你的 srv 请求字段，默认 cmd=1

  // 绑定回调 + 发送（两参重载，返回 void）
  std::function<void(SharedFuture)> cb =
      std::bind(mapping_response_cb, std::placeholders::_1, &S);
  S.client->async_send_request(S.req, cb);

  S.request_sent = true;
  S.result_ready = false;
  S.start_tp = std::chrono::steady_clock::now();

  RCLCPP_INFO(S.node->get_logger(), "[RelocateMapping] request sent");
  return BT::NodeStatus::RUNNING;

}

BT::NodeStatus RelocateMapping::onRunning()
{
  auto& S = relocate_state();

  if (!S.node || !S.exec)
  {
    // 尚未初始化，要求重新 onStart
    LOG("Relocate", "fail");
    return BT::NodeStatus::FAILURE;
  }

  // executor 轮询一次，让回调触发
  S.exec->spin_some();

  const int timeout_ms = getInput<int>("timeout_ms").value_or(15000);
  const auto now = std::chrono::steady_clock::now();
  if (now - S.start_tp >= std::chrono::milliseconds(timeout_ms))
  {
    RCLCPP_ERROR(S.node->get_logger(),
      "[RelocateMapping] timeout after %d ms", timeout_ms);
    reset_state(S);
    LOG("Relocate", "fail");
    return BT::NodeStatus::FAILURE;
  }

  if (!S.request_sent)
  {
    // 未发送请求（例如服务未就绪时），继续等待
    LOG("Relocate", "no request sent, running");
    return BT::NodeStatus::RUNNING;
  }

  if (!S.result_ready)
  {
    // 还在等待响应
    LOG("Relocate","no result ready, running");
    return BT::NodeStatus::RUNNING;
  }

  // ====== 解析响应，区分成功 / 失败 ======
  bool ok = false;

  // ⚠️ 请根据你的 qrb_ros_amr_msgs/srv/Mapping 响应字段修改此处：
  // 典型情况 1：有布尔字段 success
   ok = S.resp->result;

  // 典型情况 2：有整型状态码 code==0 表示成功
  // ok = (S.resp->code == 0);

  // 如果你不知道字段名，临时将“收到响应”视为成功：
  //ok = true;

  if (ok)
  {
    RCLCPP_INFO(S.node->get_logger(),
      "[RelocateMapping] service %s succeeded", S.service_name.c_str());
    reset_state(S);
    LOG("Relocate", "success");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_WARN(S.node->get_logger(),
      "[RelocateMapping] service %s failed", S.service_name.c_str());
    reset_state(S);
    LOG("Relocate", "fail");
    return BT::NodeStatus::FAILURE;
  }
}

void RelocateMapping::onHalted()
{
  // 被上层打断，清理内部状态
  auto& S = relocate_state();
  RCLCPP_WARN(S.node ? S.node->get_logger() : rclcpp::get_logger("RelocateMapping"),
              "[RelocateMapping] halted by parent");
  reset_state(S);
}

//// 工厂注册
void Register_Relocate(BT::BehaviorTreeFactory& f)
{
    f.registerNodeType<RelocateMapping>("RelocateMapping");
}
