
#pragma once

#include <behaviortree_cpp/condition_node.h>
#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <atomic>
#include <memory>
#include <string>


class DetectFall : public BT::ConditionNode
{
public:
    DetectFall(const std::string& name, const BT::NodeConfiguration& config);

    // 不改变原来的端口：保持为空
    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override;

private:
    // 订阅回调：更新最近一次状态
    void fallCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // 使用独立的 ROS2 context，避免默认 context 为空
    rclcpp::Context::SharedPtr context_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;

    // 最近一次消息状态（原子，确保线程安全）
    std::atomic<bool> has_msg_{false};
    std::atomic<bool> fall_{false};
};

// 工厂注册函数的声明（与原来一致）
void Register_DetectFall(BT::BehaviorTreeFactory& f);

