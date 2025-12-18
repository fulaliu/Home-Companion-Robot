#include "nodes/wait_seconds.h"
#include "common/logger.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>

BT::NodeStatus WaitSeconds::onStart()
{
    seconds_  = getInput<int>("seconds").value_or(60);
    deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(seconds_);
    LOG("WaitSeconds", "wait " + std::to_string(seconds_) + "s ...");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitSeconds::onRunning()
{
    if (!rclcpp::ok() || isHalted()) {
        LOG("WaitSeconds", "aborted");
        return BT::NodeStatus::FAILURE;  // 或 SUCCESS
    }
    if (std::chrono::steady_clock::now() >= deadline_) {
        LOG("WaitSeconds", "done");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void WaitSeconds::onHalted()
{
    LOG("WaitSeconds", "halted");
}
void Register_WaitSeconds(BT::BehaviorTreeFactory& f)
{
    f.registerNodeType<WaitSeconds>("WaitSeconds");
}

