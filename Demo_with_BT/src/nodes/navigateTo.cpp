
// navigator.cpp  (ROS 2 Jazzy / BehaviorTree.CPP v4 / C++17)
// 目标：两个非阻塞 BT 节点：SendNav2Goal（发送后立即 SUCCESS）、CheckNav2Success（成功 SUCCESS，否则 RUNNING）
// 无需使用 stateful_action_node.h

#include "common/logger.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include "nodes/navigateTo.h"
#include <atomic>
#include <thread>
#include <chrono>
#include <sstream>
#include <string>
#include <mutex>
#include <functional>
#include <cmath>     // C++17: 用于 std::sin / std::cos

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using namespace std::chrono_literals;

// ====================== Nav2 客户端（进程级单例） ======================
namespace {

enum class NavStatus { IDLE, RUNNING, SUCCEEDED, ABORTED, CANCELED };

class Nav2Client
{
public:
    Nav2Client();
    ~Nav2Client();

    // 发布可视化目标
    void publish_goal_pose(const geometry_msgs::msg::PoseStamped& pose);

    // 异步发送导航目标（不阻塞）
    void send_goal_async(const geometry_msgs::msg::PoseStamped& pose);

    // 查询状态 / 复位状态
    NavStatus get_status() const;
    void      reset_status();

    // 访问底层 rclcpp 节点
    rclcpp::Node::SharedPtr node() const;

private:
    // Action 回调（成员函数）
    void on_goal_response(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle);
    void on_feedback(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle,
                     const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void on_result(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result);

    // 关闭 / 取消
    void request_stop();
    void clear_last_goal(const std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>>& gh);

private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    std::thread spin_thread_;

    // 状态 & 最近 goal 句柄
    std::atomic<bool> stop_requested_{false};
    mutable std::mutex goal_mutex_;
    std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>> last_goal_handle_;
    std::atomic<NavStatus> last_status_{NavStatus::IDLE};
};

// ---------- Nav2Client 实现 ----------
Nav2Client::Nav2Client()
{
    if (!rclcpp::ok())
    {
        int argc = 0; char** argv = nullptr;
        rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>("bt_nav2_client");
    exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);

    goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    client_   = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");

    rclcpp::on_shutdown([this]() { request_stop(); });

    spin_thread_ = std::thread([this]() {
        while (rclcpp::ok() && !stop_requested_.load(std::memory_order_relaxed))
        {
            exec_->spin_some();
            std::this_thread::sleep_for(10ms);
        }
    });
}

Nav2Client::~Nav2Client()
{
    request_stop();
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

void Nav2Client::publish_goal_pose(const geometry_msgs::msg::PoseStamped& pose)
{
    if (!rclcpp::ok() || stop_requested_.load(std::memory_order_relaxed)) {
        return;
    }
    goal_pub_->publish(pose);
}

void Nav2Client::send_goal_async(const geometry_msgs::msg::PoseStamped& pose)
{
    if (!rclcpp::ok()) {
        return;
    }

    // 构造 goal
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = pose;

    // 配置回调（成员函数绑定）
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    using std::placeholders::_1;
    using std::placeholders::_2;
    opts.goal_response_callback = std::bind(&Nav2Client::on_goal_response, this, _1);
    opts.feedback_callback      = std::bind(&Nav2Client::on_feedback, this, _1, _2);
    opts.result_callback        = std::bind(&Nav2Client::on_result, this, _1);

    // 置状态为 RUNNING 并异步发送（不等待）
    last_status_.store(NavStatus::RUNNING, std::memory_order_relaxed);
    auto future_goal = client_->async_send_goal(goal_msg, opts);
    (void)future_goal;  // 不阻塞等待；结果在回调里处理
}

NavStatus Nav2Client::get_status() const
{
    return last_status_.load(std::memory_order_relaxed);
}

void Nav2Client::reset_status()
{
    last_status_.store(NavStatus::IDLE, std::memory_order_relaxed);
}

rclcpp::Node::SharedPtr Nav2Client::node() const
{
    return node_;
}

void Nav2Client::on_goal_response(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle)
{
    std::lock_guard<std::mutex> lk(goal_mutex_);
    last_goal_handle_ = goal_handle;

    if (!goal_handle)
    {
        RCLCPP_ERROR(node_->get_logger(), "Nav2 goal rejected by server");
        last_status_.store(NavStatus::ABORTED, std::memory_order_relaxed);
    }
}

void Nav2Client::on_feedback(
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const NavigateToPose::Feedback> /*feedback*/)
{
    // 可选：记录进度；不改状态
    // RCLCPP_DEBUG(node_->get_logger(), "Nav2 feedback received");
}

void Nav2Client::on_result(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result)
{
    std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>> gh;
    {
        std::lock_guard<std::mutex> lk(goal_mutex_);
        gh = last_goal_handle_;
        last_goal_handle_.reset();
    }

    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            last_status_.store(NavStatus::SUCCEEDED, std::memory_order_relaxed);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            last_status_.store(NavStatus::ABORTED, std::memory_order_relaxed);
            break;
        case rclcpp_action::ResultCode::CANCELED:
            last_status_.store(NavStatus::CANCELED, std::memory_order_relaxed);
            break;
        default:
            last_status_.store(NavStatus::ABORTED, std::memory_order_relaxed);
            break;
    }

    if (gh) {
        clear_last_goal(gh);
    }
}

void Nav2Client::request_stop()
{
    stop_requested_.store(true, std::memory_order_relaxed);
    if (exec_) {
        exec_->cancel();
    }
    std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>> gh;
    {
        std::lock_guard<std::mutex> lk(goal_mutex_);
        gh = last_goal_handle_;
        last_goal_handle_.reset();
    }
    if (gh) {
        client_->async_cancel_goal(gh);
    }
}

void Nav2Client::clear_last_goal(
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>>& gh)
{
    std::lock_guard<std::mutex> lk(goal_mutex_);
    if (last_goal_handle_ == gh) {
        last_goal_handle_.reset();
    }
}

// 单例访问器
Nav2Client& nav2_client()
{
    static Nav2Client inst;
    return inst;
}

// 解析 "x,y,yaw_deg" 为 PoseStamped（frame_id 默认 "map"）
geometry_msgs::msg::PoseStamped parse_goal(const std::string& s, const std::string& frame_id)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id.empty() ? "map" : frame_id;
    pose.header.stamp    = nav2_client().node()->get_clock()->now();

    double x = 0.0, y = 0.0, yaw_deg = 0.0;
    char c1 = 0, c2 = 0;
    std::stringstream ss(s);
    if (!((ss >> x >> c1 >> y >> c2 >> yaw_deg) && (c1 == ',' && c2 == ',')))
    {
        x = 0.0; y = 0.0; yaw_deg = 0.0;
    }

    // C++17：用常量替代 std::numbers::pi
    constexpr double kPi = 3.14159265358979323846;
    const double yaw = yaw_deg * (kPi / 180.0);

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    return pose;
}

} // namespace（单例与工具函数）

// ===================== BehaviorTree 节点声明 =====================

// ----- SendNav2Goal -----
SendNav2Goal::SendNav2Goal(const std::string& name, const BT::NodeConfig& config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList SendNav2Goal::providedPorts()
{
    return {
        BT::InputPort<std::string>("goal", "x,y,yaw_deg"),
        BT::InputPort<std::string>("frame_id", "map")
    };
}

BT::NodeStatus SendNav2Goal::tick()
{
    auto goal = getInput<std::string>("goal");
    auto frame = getInput<std::string>("frame_id");

    if (!goal) {
        LOG("SendNav2Goal", std::string("missing ") + goal.error());
        return BT::NodeStatus::FAILURE;
    }

    const std::string goal_val  = goal.value();
    const std::string frame_val = frame.value_or("map");

    if (!rclcpp::ok()) {
        LOG("SendNav2Goal", "rclcpp shutdown, skip navigation");
        return BT::NodeStatus::FAILURE;
    }

    const auto pose = parse_goal(goal_val, frame_val);
    nav2_client().publish_goal_pose(pose);
    nav2_client().send_goal_async(pose);

    LOG("SendNav2Goal", "goal=" + goal_val + ", frame=" + frame_val);
    // 发送后立即成功（不阻塞）
    return BT::NodeStatus::SUCCESS;
}

// ----- CheckNav2Success -----

CheckNav2Success::CheckNav2Success(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config) {}

BT::PortsList CheckNav2Success::providedPorts() { return {}; }

BT::NodeStatus CheckNav2Success::onStart()   { return evaluate_once(); }
BT::NodeStatus CheckNav2Success::onRunning() { return evaluate_once(); }
void CheckNav2Success::onHalted()              { /* non-blocking, optional reset */ }

BT::NodeStatus CheckNav2Success::evaluate_once()
{
  if (!rclcpp::ok()) { return BT::NodeStatus::RUNNING; }
    if (!rclcpp::ok()) {
        // 系统在关闭中，保持 RUNNING，让上层自行处理
        return BT::NodeStatus::RUNNING;
    }

    const NavStatus st = nav2_client().get_status();
    switch (st)
    {
        case NavStatus::SUCCEEDED:
	    LOG("CheckNav2Success", "Nav2 succeeded");
            return BT::NodeStatus::SUCCESS;
	case NavStatus::IDLE:
            LOG("CheckNav2Success", "Nav2 idle");
            return BT::NodeStatus::SUCCESS;

        case NavStatus::RUNNING:
	    LOG("CheckNav2Success", "Nav2 running");
            return BT::NodeStatus::RUNNING;

        case NavStatus::ABORTED:
        case NavStatus::CANCELED:
        default:
            // 按你的需求：未成功一律 RUNNING（不中断上层流程）
             LOG("CheckNav2Success", "Nav2 failure");
	    return BT::NodeStatus::FAILURE;
    }

}

// ===================== 工厂注册函数（供 XML 使用） =====================
void Register_SendNav2Goal(BT::BehaviorTreeFactory& f)
{
    f.registerNodeType<SendNav2Goal>("SendNav2Goal");
}

void Register_CheckNav2Success(BT::BehaviorTreeFactory& f)
{
    f.registerNodeType<CheckNav2Success>("CheckNav2Success");
}

