#pragma once
#include "behaviortree_cpp/bt_factory.h"

class NavigateTo : public BT::SyncActionNode
{
public:
    NavigateTo(const std::string& name, const BT::NodeConfig& cfg)
    : BT::SyncActionNode(name, cfg) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("goal"),
            BT::InputPort<int>("timeout_sec"),
            BT::InputPort<int>("watchdog_sec"),
            BT::InputPort<double>("threshold"),
            BT::InputPort<double>("battery_pct")
        };
    }

    BT::NodeStatus tick() override;
};

// 将注册函数改为带参数，默认值为 "NavigateTo"
void Register_NavigateTo(BT::BehaviorTreeFactory& f);
