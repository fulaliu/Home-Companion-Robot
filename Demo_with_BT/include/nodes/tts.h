#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <memory>
#include <string>

class TTS : public BT::SyncActionNode
{
public:
    TTS(const std::string& name, const BT::NodeConfiguration& config);

    // 不改变原来的端口：保留输入端口 text
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("text") };
    }

    BT::NodeStatus tick() override;

private:
    // ROS2：独立的上下文、节点与发布器
    rclcpp::Context::SharedPtr context_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

// 工厂注册函数声明（与原来保持一致的类型名）
void Register_TTS(BT::BehaviorTreeFactory& f);

