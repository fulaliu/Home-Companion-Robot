#include "nodes/command_conditions.h"
#include <functional>  // std::bind, std::placeholders
#include "std_msgs/msg/string.hpp"
#include "common/logger.h"

#include <memory>
#include <atomic>

namespace {

struct CommandStatsCache {
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec;

    // ✅ 改为 shared_ptr，并用原子自由函数来读写
    std::shared_ptr<const std::string> got;

    CommandStatsCache() {
        // 建议在 main 里先 rclcpp::init(argc, argv)，此处仅兜底
        if (!rclcpp::ok()) {
            int argc = 0; char** argv = nullptr;
            rclcpp::init(argc, argv);
        }

        node = std::make_shared<rclcpp::Node>("command_ok_client");

        exec = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        exec->add_node(node);

        using std::placeholders::_1;
        sub = node->create_subscription<std_msgs::msg::String>(
            "/command_control",
            rclcpp::SensorDataQoS(),
            std::bind(&CommandStatsCache::handle_command_msg, this, _1)
        );
    }

    // 在 tick 内调用，处理已到达的订阅消息
    void pump() {
        if (rclcpp::ok() && exec) {
            exec->spin_some();
        }
    }

private:
    void handle_command_msg(const std_msgs::msg::String::SharedPtr msg)
    {
        auto p = std::make_shared<const std::string>(msg->data);
        // ✅ 原子写入 shared_ptr
        std::atomic_store_explicit(&got, p, std::memory_order_relaxed);
    }
};

// ✅ 返回引用，避免对不可复制成员进行拷贝
CommandStatsCache& command_cache() {
    static CommandStatsCache s;
    return s;
}

struct CommandPublisherCache {
    rclcpp::Node::SharedPtr node;
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pubs;

    CommandPublisherCache() {
        if (!rclcpp::ok()) {
            int argc = 0; char** argv = nullptr;
            rclcpp::init(argc, argv);
        }
        node = std::make_shared<rclcpp::Node>("command_pub_client");
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr get(const std::string& topic) {
        auto it = pubs.find(topic);
        if (it != pubs.end()) return it->second;
        auto p = node->create_publisher<std_msgs::msg::String>(topic, rclcpp::SensorDataQoS());
        pubs.emplace(topic, p);
        return p;
    }

    rclcpp::Logger logger() const { return node->get_logger(); }
};

CommandPublisherCache& cmd_pub_cache() {
    static CommandPublisherCache c;
    return c;
}

} // anonymous namespace

ReadCommand::ReadCommand(const std::string& name, const BT::NodeConfig& config)
: BT::SyncActionNode(name, config)
{}

BT::NodeStatus ReadCommand::tick()
{
  auto& cache = command_cache();
  cache.pump();

  // ✅ 原子读取 shared_ptr，然后判空
  auto p = std::atomic_load_explicit(&cache.got, std::memory_order_relaxed);
  const std::string s = (p ? *p : std::string("0"));

  setOutput("cmd_out", s);
  //std::cout << "[ReadCommand] command=" << s << " -> SUCCESS\n";
  LOG("ReadCommand", s);
  return BT::NodeStatus::SUCCESS;
}

// ------------- 工厂注册函数 -------------
void Register_ReadCommand(BT::BehaviorTreeFactory& f)
{
  f.registerNodeType<ReadCommand>("ReadCommand");
}


SetCommand::SetCommand(const std::string& name, const BT::NodeConfig& config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList SetCommand::providedPorts()
{
  return {
    // 输入：欲设置的字符串值，默认 "2"
    BT::InputPort<std::string>("value", "2", "command value to write"),
    // 输入：话题名，默认 /command_control
    BT::InputPort<std::string>("topic", "/command_control", "ROS2 topic to publish"),
    // 输出：把值也写到黑板，供 Switch2 立即/下次使用
    BT::OutputPort<std::string>("command", "blackboard command")
  };
}

BT::NodeStatus SetCommand::tick()
{
  // 读取端口
  auto value = getInput<std::string>("value").value_or(std::string("2"));
  auto topic = getInput<std::string>("topic").value_or(std::string("/command_control"));

  // 写黑板（供 Switch2 使用）
  setOutput("command", value);

  // 发布到 ROS 话题（供 ReadCommand / 其他模块使用）
  auto& cache = cmd_pub_cache();
  auto pub = cache.get(topic);

  std_msgs::msg::String msg;
  msg.data = value;
  pub->publish(msg);
  LOG("SetCommand", value);
  RCLCPP_INFO(cache.logger(), "[SetCommand] published '%s' to %s and set BB 'command'",
              value.c_str(), topic.c_str());

  // 立即返回成功
  return BT::NodeStatus::SUCCESS;
}

// 工厂注册函数（在你现有的注册处调用）
void Register_SetCommand(BT::BehaviorTreeFactory& f)
{
   f.registerNodeType<SetCommand>("SetCommand");
}
