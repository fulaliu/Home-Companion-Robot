#pragma once
#include <optional>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/int32.hpp"
#include <behaviortree_cpp/bt_factory.h>
class ReadCommand : public BT::SyncActionNode
{
public:
  ReadCommand(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts(){
  return { BT::OutputPort<std::string>("cmd_out", "command to BB as string") };
  }
  BT::NodeStatus tick() override;
};


class SetCommand : public BT::SyncActionNode
{
public:
  SetCommand(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// 可选：把注册函数也写进头文件声明（如果你已有一个集中注册处）
void Register_SetCommand(BT::BehaviorTreeFactory& f);

// 工厂注册函数
void Register_ReadCommand(BT::BehaviorTreeFactory& f);

