#include <iostream>
#include <fstream>
#include <thread>
#include <atomic>
#include <chrono>
#include <filesystem>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"

#include "common/config.h"
#include "common/logger.h"

// 各模块的注册入口（你工程已有）
#include "nodes/battery_ok.h"
#include "nodes/dock_and_charge.h"
#include "nodes/relocate.h"
#include "nodes/navigateTo.h"
#include "nodes/detect_fall.h"
#include "nodes/tts.h"
#include "nodes/clear_costmaps.h"
#include "nodes/wait_seconds.h"
#include "nodes/command_conditions.h"

#include <atomic>
#include <chrono>
#include <cctype>
#include <iostream>
#include <thread>
#include <string>
#include "behaviortree_cpp/blackboard.h"
#include <rclcpp/rclcpp.hpp>

// 全局停止标记
static std::atomic<bool> g_stop{false};

static void sigint_handler(int)
{
    g_stop.store(true);
    // 触发 ROS 全局关闭，令 rclcpp::ok() 变为 false
    rclcpp::shutdown();
}

// 通用：只有当 new_value 与当前值不同，才写入黑板
template <typename T>
bool set_if_changed(BT::Blackboard::Ptr bb, const std::string& key, const T& new_value)
{
    try
    {
        const T cur = bb->get<T>(key);
        if (cur == new_value)
        {
            // 不触发写入，也不触发任何潜在的监听逻辑
            return false;
        }
    }
    catch (const std::exception&)
    {
        // key 不存在：视为发生变化，直接写入
    }

    bb->set<T>(key, new_value);
    return true;
}

void input_thread_function(BT::Blackboard::Ptr bb, std::atomic<bool>& running)
{
    std::cout << "键盘面板：C 切换 commands, E 切换 expected, Q 退出\n";

    while (running.load())
    {
        // 防止无输入时忙等
        if (!std::cin.good())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        char ch;
        if (!(std::cin >> ch))
        {
            // EOF 或错误，稍后重试
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // 统一大小写
        ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));

        if (ch == 'C')
        {
            // 切换 commands: Yes <-> No
            std::string cur;
            try { cur = bb->get<std::string>("commands"); }
            catch (...) { cur = "Yes"; } // 若未初始化，默认当作 "No"

            const std::string next = (cur == "Yes") ? "No" : "Yes";

            if (set_if_changed<std::string>(bb, "commands", next))
            {
                std::cout << "[键盘口] commands => " << next << "（已更新）\n";
            }
            else
            {
                std::cout << "[键盘口] commands 维持为 " << next << "（未变化，未写入）\n";
            }
        }
        else if (ch == 'E')
        {
            // 切换 expected: Yes <-> No
            std::string cur;
            try { cur = bb->get<std::string>("expected"); }
            catch (...) { cur = "Yes"; } // 若未初始化，默认当作 "Yes"

            const std::string next = (cur == "Yes") ? "No" : "Yes";

            if (set_if_changed<std::string>(bb, "expected", next))
            {
                std::cout << "[键盘口] expected => " << next << "（已更新）\n";
            }
            else
            {
                std::cout << "[键盘口] expected 维持为 " << next << "（未变化，未写入）\n";
            }
        }
        else if (ch == 'Q')
        {
            running.store(false);
            std::cout << "[键盘口] 退出请求\n";
        }
        else
        {
            std::cout << "[键盘口] 未识别输入：" << ch << "；有效键：C/E/Q\n";
        }
    }
}


static std::string to_string_status(BT::NodeStatus s)
{
    switch (s)
    {
    case BT::NodeStatus::IDLE:    return "IDLE";
    case BT::NodeStatus::RUNNING: return "RUNNING";
    case BT::NodeStatus::SUCCESS: return "SUCCESS";
    case BT::NodeStatus::FAILURE: return "FAILURE";
    default:                      return "UNKNOWN";
    }
}


int main(int argc, char** argv)
{
    // 1) 解析命令行参数
    std::string xml_file = "tree.xml";
    std::string cfg_file = "config.ini";
    for (int i = 1; i < argc; i++){
        std::string arg = argv[i];
        if ((arg=="--xml" || arg=="-x") && i+1 < argc) xml_file = argv[++i];
        else if ((arg=="--config" || arg=="-c") && i+1 < argc) cfg_file = argv[++i];
    }

    // 2) 加载配置
    AppConfig cfg;
    if(!loadConfig(cfg_file, cfg)){
        std::cerr << "[Error] failed to load config: " << cfg_file << "\n";
        return 1;
    }

    // 3) 注册节点
    BT::BehaviorTreeFactory factory;
    Register_BatteryOK(factory);
    Register_DockAndCharge(factory);
    Register_Relocate(factory);
    Register_DetectFall(factory);
    Register_TTS(factory);
    Register_WaitSeconds(factory);
//    Register_CommandsIsYes(factory);
    
    Register_SendNav2Goal(factory);
    Register_CheckNav2Success(factory);

    Register_ReadCommand(factory);
    Register_SetCommand(factory);

    // 4) 写 models.xml（供 Groot2 Editor 导入）
    {
        const std::string models_xml = BT::writeTreeNodesModelXML(factory);
        std::ofstream("models.xml") << models_xml;
        LOG("Main", "models.xml written");
    }

    // 5) 加载行为树 XML 并创建树
    // 确保 tree.xml 中存在 ID=cfg.main_tree_id 的 BehaviorTree
    factory.registerBehaviorTreeFromFile(xml_file);
    auto tree = factory.createTree(cfg.main_tree_id);

    // 6) 将配置写入黑板（供 XML 端口使用）
    auto bb = tree.rootBlackboard();

    // 初始化黑板键的默认值（可选）
    bb->set<std::string>("commands", "Yes");   // 初始不触发
    bb->set<std::string>("expected", "Yes");  // 期望值为 Yes

    // 目标点（字符串）
    bb->set<std::string>("goal_A", cfg.goal_A);
    bb->set<std::string>("goal_B", cfg.goal_B);
    bb->set<std::string>("goal_C", cfg.goal_C);
    bb->set<std::string>("goal_D", cfg.goal_D);

    // 电量阈值（整数）
    bb->set<int>("battery_threshold", cfg.battery_threshold);
    bb->set<int>("battery_pct",      cfg.battery_pct);

LOG("BB", "goal_A=" + bb->get<std::string>("goal_A"));
LOG("BB", "goal_B=" + bb->get<std::string>("goal_B"));
LOG("BB", "goal_C=" + bb->get<std::string>("goal_C"));
LOG("BB", "goal_D=" + bb->get<std::string>("goal_D"));
LOG("BB", "battery_threshold=" + std::to_string(bb->get<double>("battery_threshold")));
LOG("BB", "battery_pct="      + std::to_string(bb->get<double>("battery_pct")));


    // 7) 启动 Groot2 监控（PRO）
    BT::Groot2Publisher publisher(tree, cfg.groot_port);
    publisher.setMaxHeartbeatDelay(std::chrono::milliseconds(8000));

    // 8) 可选：记录 btlog，Groot2 Log Replay 可用
    BT::FileLogger2 filelog(tree, "bt_trace.btlog");

    // 9) 键盘输入线程：作为“修改入口/键盘口”
    std::atomic<bool> running{true};
    std::thread input_thread(input_thread_function, bb, std::ref(running));

    // 10) 主循环
    BT::NodeStatus status = BT::NodeStatus::IDLE;

 /*      while (running.load())
    {
        status = tree.rootNode()->executeTick(); // 或：status = tree.tickRoot();
        if (status != BT::NodeStatus::RUNNING) {
        	break;
	}
	if (! rclcpp::ok()){
	break;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

*/
    // ① 进入程序就打印一次 ok 状态（还未 init）
    LOG("Bootstrap", std::string("before init: rclcpp::ok=") +
                     (rclcpp::ok() ? "true" : "false"));

    // ② 显式初始化（必须在任何 ok 检查/节点创建之前）
    rclcpp::init(argc, argv);
    LOG("Bootstrap", std::string("after init: rclcpp::ok=") +
                     (rclcpp::ok() ? "true" : "false"));


// shutdown 时，打日志并置位 running=false
rclcpp::on_shutdown([&](){
    LOG("MainLoop", "on_shutdown: set running=false");
    running.store(false, std::memory_order_relaxed);
});

std::size_t iter = 0;

    const bool running_val = running.load(std::memory_order_relaxed);
    const bool ok_val      = rclcpp::ok();

    // 循环开始前的判断值
    LOG("MainLoop",
        "pre: iter=" + std::to_string(iter) +
        ", running=" + std::string(running_val ? "true" : "false") +
        ", rclcpp::ok=" + std::string(ok_val ? "true" : "false"));

/*while (running.load(std::memory_order_relaxed) && rclcpp::ok())
{
    const bool running_val = running.load(std::memory_order_relaxed);
    const bool ok_val      = rclcpp::ok();

    // 循环开始前的判断值
    // 如果在进入 executeTick 之前条件已经不满足，直接退出
    if (!(running_val && ok_val)) {
        LOG("MainLoop", "break: condition false before tick");
        break;
    }

    // 执行一次 tick（注意：若某节点内部阻塞睡眠，post 日志会在该阻塞结束后才打印）
    auto status = tree.rootNode()->executeTick();  // 或：tree.tickRoot();

    // 循环后的状态
    ++iter;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}*/

  while (rclcpp::ok())
  {
    //rclcpp::spin_some(rosnode);
    //BT::NodeStatus s = tree.tickRoot();
    auto s = tree.rootNode()->executeTick();
    std::cout << "[TICK] root -> "
              << (s == BT::NodeStatus::SUCCESS ? "SUCCESS" :
                  s == BT::NodeStatus::FAILURE ? "FAILURE" : "RUNNING")
              << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

    tree.haltTree();

    if (input_thread.joinable()) input_thread.join();
    return 0;
}


