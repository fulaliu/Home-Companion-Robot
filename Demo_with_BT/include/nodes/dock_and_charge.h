#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <string>

// 行为树节点：通过 action /cmd 发送 {command=5} 执行 Dock & Charge
class DockAndCharge : public BT::StatefulActionNode
{
public:
  DockAndCharge(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  // StatefulActionNode 三阶段
  BT::NodeStatus onStart() override;    // 第一次 tick：发送 goal
  BT::NodeStatus onRunning() override;  // 后续 tick：检查结果/反馈/超时
  void onHalted() override;             // 被上层打断：取消 goal

  // 如果你需要行为树打印端口信息
  static inline BT::NodeConfig config_;
};
void Register_DockAndCharge(BT::BehaviorTreeFactory& f);

