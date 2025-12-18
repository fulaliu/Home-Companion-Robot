#include "nodes/detect_fall.h"
#include "common/logger.h"
#include <functional>

DetectFall::DetectFall(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    // 1) 初始化独立的 ROS2 context（不依赖全局默认上下文）
    context_ = std::make_shared<rclcpp::Context>();
    if (!rclcpp::ok())
    {
        context_->init(0, nullptr);
    }

    //context_->init(0, nullptr);  // 若主程序未调用 rclcpp::init，这里自行初始化

    // 2) 用该 context 构造 Executor（非阻塞回调处理）
    rclcpp::ExecutorOptions opts;
    opts.context = context_;  // 关键修正：把 context 放到 options 里
    exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(opts);

    // 3) 用同一个 context 构造 Node，并加入执行器
    rclcpp::NodeOptions node_opts;
    node_opts.context(context_);
    node_ = rclcpp::Node::make_shared("detect_fall_bt_node", node_opts);
    exec_->add_node(node_);

    // 4) 订阅 /fall_detected（std_msgs::msg::Bool），队列长度 1
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/fall_detected",
        rclcpp::QoS(1),
        std::bind(&DetectFall::fallCallback, this, std::placeholders::_1));

    LOG("DetectFall", "ROS2 context & node initialized, subscribed to /fall_detected");
}

void DetectFall::fallCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    fall_.store(msg->data, std::memory_order_relaxed);
    has_msg_.store(true, std::memory_order_relaxed);
}

BT::NodeStatus DetectFall::tick()
{
    // 非阻塞：仅拉一次回调队列
    exec_->spin_some();

    const bool received     = has_msg_.load(std::memory_order_relaxed);
    const bool fall_state   = fall_.load(std::memory_order_relaxed);
    const std::size_t pubs  = node_->count_publishers("/fall_detected");

    // 返回 False 的三种情况：未收到消息 / 无发布者 / 值为 false
    if (!received || pubs == 0 || !fall_state)
    {
        LOG("DetectFall", "fall detection -> FALSE");
        return BT::NodeStatus::FAILURE;  // “返回 False”
    }

    LOG("DetectFall", "fall detection -> TRUE");
    return BT::NodeStatus::SUCCESS;      // “返回 True”
}

// 行为树工厂注册（保持类型名不变）
void Register_DetectFall(BT::BehaviorTreeFactory& f)
{
    f.registerNodeType<DetectFall>("DetectFall");
}

