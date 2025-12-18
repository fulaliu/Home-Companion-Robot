#include "nodes/tts.h"
#include "common/logger.h"

TTS::TTS(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    // 1) 初始化独立的 ROS2 context，避免默认 context 未初始化而崩溃
    context_ = std::make_shared<rclcpp::Context>();
    //context_->init(0, nullptr);
    if (!rclcpp::ok())
    {
        context_->init(0, nullptr);
    }


    // 2) 使用同一 context 创建 Node
    rclcpp::NodeOptions node_opts;
    node_opts.context(context_);
    node_ = rclcpp::Node::make_shared("tts_bt_node", node_opts);

    // 3) 创建 Publisher：/qrb_ros_tts，消息类型 std_msgs::msg::String
    //    QoS 采用默认 keep_last(10) 即可；如需可靠传输，可在此调整 QoS。
    pub_ = node_->create_publisher<std_msgs::msg::String>("/qrb_ros_tts", rclcpp::QoS(10));

    LOG("TTS", "ROS2 context & node initialized, publisher on /qrb_ros_tts ready");
}

BT::NodeStatus TTS::tick()
{
    // 读取输入端口 text（不改变原端口）
    auto text = getInput<std::string>("text").value_or("");

    // 非阻塞发布：立即 publish，不等待
    std_msgs::msg::String msg;
    msg.data = text;
    pub_->publish(msg);

    LOG("TTS", std::string("say: ") + text + " (published to /qrb_ros_tts)");

    // 发送完立即返回成功
    return BT::NodeStatus::SUCCESS;
}

// 行为树工厂注册（保持原类型名 "TTS"）
void Register_TTS(BT::BehaviorTreeFactory& f)
{
    f.registerNodeType<TTS>("TTS");
}

