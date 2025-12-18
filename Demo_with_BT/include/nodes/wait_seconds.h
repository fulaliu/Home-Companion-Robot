#pragma once
#include <behaviortree_cpp/action_node.h>
#include "behaviortree_cpp/bt_factory.h"

class WaitSeconds : public BT::StatefulActionNode
{
public:
    WaitSeconds(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("seconds", 60, "Time to wait in seconds") };
    }

private:
    using Clock = std::chrono::steady_clock;
    int seconds_{60};
    Clock::time_point deadline_{};

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};
void Register_WaitSeconds(BT::BehaviorTreeFactory& f);
