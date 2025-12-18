#pragma once
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>

class BatteryOK : public BT::StatefulActionNode
{
public:
    BatteryOK(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
    return {
            BT::InputPort<double>("threshold"),
            BT::InputPort<double>("battery_pct")
        };
    }

private:
    double threshold_{40.0};
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

void Register_BatteryOK(BT::BehaviorTreeFactory& f);

