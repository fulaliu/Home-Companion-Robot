
// nodes/nav2_bt_nodes.h
#ifndef NODES_NAV2_BT_NODES_H
#define NODES_NAV2_BT_NODES_H

#include <string>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

// -----------------------------------------------------------------------------
// 节点1：发送 Nav2 目标（非阻塞，tick 立即 SUCCESS）
// -----------------------------------------------------------------------------
class SendNav2Goal : public BT::SyncActionNode
{
public:
  SendNav2Goal(const std::string& name, const BT::NodeConfig& config);

  // 输入端口：
  // - goal     : "x,y,yaw_deg" 字符串（例如 "1.0,2.0,90"）
  // - frame_id : 可选坐标系，默认 "map"
  static BT::PortsList providedPorts();

  // 非阻塞：发布 /goal_pose 并异步向 Nav2 发送目标，立即返回 SUCCESS
  BT::NodeStatus tick() override;
};

// -----------------------------------------------------------------------------
// 节点2：检查 Nav2 是否成功（非阻塞）
// 成功返回 SUCCESS；否则（运行中/失败/取消/空闲）返回 RUNNING
// -----------------------------------------------------------------------------
class CheckNav2Success : public BT::StatefulActionNode
{
public:
  CheckNav2Success(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();
private:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    BT::NodeStatus evaluate_once();
};

// -----------------------------------------------------------------------------
// 工厂注册函数（供 BehaviorTreeFactory 使用）
// -----------------------------------------------------------------------------
void Register_SendNav2Goal(BT::BehaviorTreeFactory& factory);
void Register_CheckNav2Success(BT::BehaviorTreeFactory& factory);

#endif // NODES_NAV2_BT_NODES_H

