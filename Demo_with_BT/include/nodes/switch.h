#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/control_node.h"

class SwitchNode : public BT::ControlNode
{
public:
    SwitchNode(const std::string& name, const BT::NodeConfig& config)
        : BT::ControlNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("variable") };
    }

    void halt() override { for (auto& child : children_nodes_) child->halt(); }

    BT::NodeStatus tick() override
    {
        auto var = getInput<std::string>("variable").value_or("");
        size_t index = 0;

        if (var == "1") index = 0; // 第一个 Case
        else if (var == "2") index = 1; // 第二个 Case
        else return BT::NodeStatus::FAILURE;

        if (index >= children_nodes_.size()) return BT::NodeStatus::FAILURE;

        return children_nodes_[index]->executeTick();
    }
};

