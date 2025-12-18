#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <string>

// 行为树节点：调用 /amr_mapping 服务（qrb_ros_amr_msgs/srv/Mapping）
class RelocateMapping : public BT::StatefulActionNode
{
public:
  RelocateMapping(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  // StatefulActionNode 三个生命周期钩子
  BT::NodeStatus onStart() override;   // 第一次 tick，发送请求
  BT::NodeStatus onRunning() override; // 后续 tick，轮询响应/超时
  void onHalted() override;            // 被上层打断时，重置状态
};
void Register_Relocate(BT::BehaviorTreeFactory& f);

