#pragma once
#include "behaviortree_cpp/bt_factory.h"

class ClearCostmaps : public BT::SyncActionNode
{
public:
    ClearCostmaps(const std::string& name, const BT::NodeConfig& cfg)
    : BT::SyncActionNode(name, cfg) {}
    static BT::PortsList providedPorts(){ return {}; }
    BT::NodeStatus tick() override;
};
void Register_ClearCostmaps(BT::BehaviorTreeFactory& f);
