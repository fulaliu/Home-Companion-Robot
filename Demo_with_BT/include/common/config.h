#pragma once
#include <string>

// 你可以根据项目需求继续增加字段
struct AppConfig
{
    // 行为树主 ID（必须与 tree.xml 中 ID 一致）
    std::string main_tree_id = "MainTree";

    // 目标点（与 XML 中的黑板键一致）
    std::string goal_A = "A";
    std::string goal_B = "B";
    std::string goal_C = "C";
    std::string goal_D = "D";

    // 电量相关
    int battery_threshold = 20;
    int battery_pct       = 80;

    // Groot2 端口
    int groot_port = 1667;
    unsigned loop_ms = 100;
};

// 读取 config.ini 到 AppConfig
bool loadConfig(const std::string& path, AppConfig& cfg);

