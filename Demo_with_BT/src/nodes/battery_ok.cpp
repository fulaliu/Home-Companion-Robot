#include "nodes/battery_ok.h"
#include "common/logger.h"

#include <sensor_msgs/msg/battery_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <atomic>
#include <cmath>
#include <limits>
#include <chrono>
#include <functional>   // for std::bind

using namespace std::chrono_literals;

// ---- 订阅缓存：首次使用时创建订阅；在节点 onStart/onRunning 中 pump 一下队列 ----
namespace {

struct BatteryStatsCache {
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec;

    std::atomic<bool>   got{false};
    std::atomic<double> pct{std::numeric_limits<double>::quiet_NaN()};

    BatteryStatsCache() {
        // 建议在 main 里先 rclcpp::init(argc, argv)，此处仅兜底
        if (!rclcpp::ok()) {
            int argc = 0; char** argv = nullptr;
            rclcpp::init(argc, argv);
        }

        node = std::make_shared<rclcpp::Node>("battery_ok_client");

        exec = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        exec->add_node(node);

        using std::placeholders::_1;
        sub = node->create_subscription<sensor_msgs::msg::BatteryState>(
            "/battery_stats",
            rclcpp::SensorDataQoS(),
            std::bind(&BatteryStatsCache::handle_battery_msg, this, _1)  // ✅ 单独函数体
        );
    }

    // 在 tick 内调用，处理已到达的订阅消息
    void pump() {
        if (rclcpp::ok() && exec) {
            exec->spin_some();
        }
    }

private:
    // ✅ 把原来的 lambda 逻辑分开写在成员函数里
    void handle_battery_msg(const sensor_msgs::msg::BatteryState::ConstSharedPtr& msg)
    {
        double p = static_cast<double>(msg->percentage);

        // 自适应比例/百分比：如果是 0~1 比例，转为百分比
        if (!std::isnan(p)) {
            if (p <= 1.0) {
                p *= 100.0;  // 0~1 → 0~100%
            }
            pct.store(p, std::memory_order_relaxed);
            got.store(true, std::memory_order_relaxed);
        }
    }
};

// ✅ 函数静态单例：对象符号不导出为“外部可绑定”，规避 AArch64 共享库危险重定位
BatteryStatsCache& battery_cache() {
    static BatteryStatsCache s;
    return s;
}

} // namespace

// -------------------- BatteryOK: StatefulActionNode 实现 --------------------

#include <behaviortree_cpp/bt_factory.h>  // 为注册函数提供声明

BT::NodeStatus BatteryOK::onStart()
{
    threshold_ = getInput<double>("threshold").value_or(40.0);

    auto& cache = battery_cache();
    cache.pump();

    if (!cache.got.load(std::memory_order_relaxed)) {
        LOG("BatteryOK", "no /battery_stats yet, threshold=" + std::to_string(threshold_));
        return BT::NodeStatus::RUNNING;  // ✅ stateful 节点可以返回 RUNNING
    }

    const double pct = cache.pct.load(std::memory_order_relaxed);
    LOG("BatteryOK", "threshold=" + std::to_string(threshold_) + ", battery_pct=" + std::to_string(pct));
    return (pct >= threshold_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus BatteryOK::onRunning()
{
    // 支持 Ctrl+C / shutdown 早停
    if (!rclcpp::ok()) {
        LOG("BatteryOK", "rclcpp shutdown, abort");
        return BT::NodeStatus::FAILURE; // 或 SUCCESS，按你的树语义决定
    }

    auto& cache = battery_cache();
    cache.pump();

    if (!cache.got.load(std::memory_order_relaxed)) {
        return BT::NodeStatus::RUNNING;
    }

    const double pct = cache.pct.load(std::memory_order_relaxed);
    LOG("BatteryOK", "threshold=" + std::to_string(threshold_) + ", battery_pct=" + std::to_string(pct));
    return (pct >= threshold_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void BatteryOK::onHalted()
{
    LOG("BatteryOK", "halted");
    // 若你在该节点里发起了 action/定时器，应在此取消；cache 只是订阅拉取，通常无需处理
}

void Register_BatteryOK(BT::BehaviorTreeFactory& f)
{
    f.registerNodeType<BatteryOK>("BatteryOK");
}

