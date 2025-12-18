#include "common/config.h"
#include "common/logger.h"   // 若没有该头，会用下面的兜底 LOG 宏

#include <unordered_map>
#include <fstream>
#include <string>
#include <string_view>
#include <algorithm>
#include <cctype>
#include <cmath>     // lround
#include <iostream>  // 兜底输出

// 兜底：如果你没有定义 LOG 宏，则使用 std::cout 打印
#ifndef LOG
  #define LOG(TAG, MSG) (std::cout << "[" << TAG << "] " << MSG << std::endl)
#endif

// ---------------- 工具函数 ----------------


static inline bool is_space(unsigned char ch)
{
    return std::isspace(ch) != 0;
}

// 复制并修剪首尾空白
static std::string trim_copy(std::string_view sv)
{
    size_t start = 0;
    while (start < sv.size() && is_space(static_cast<unsigned char>(sv[start]))) {
        ++start;
    }

    size_t end = sv.size();
    while (end > start && is_space(static_cast<unsigned char>(sv[end - 1]))) {
        --end;
    }

    return std::string(sv.substr(start, end - start));
}

/// 移除 UTF-8 BOM（常见于第一行）
static void remove_utf8_bom(std::string& s)
{
    if (s.size() >= 3 && static_cast<unsigned char>(s[0]) == 0xEF
                     && static_cast<unsigned char>(s[1]) == 0xBB
                     && static_cast<unsigned char>(s[2]) == 0xBF)
    {
        s.erase(0, 3);
    }
}

/// 去除值里的行内注释（# 或 ;，不在双引号内）
static void strip_inline_comment(std::string& s)
{
    bool in_quotes = false;
    for (size_t i = 0; i < s.size(); ++i)
    {
        char c = s[i];
        if (c == '"') in_quotes = !in_quotes;
        if (!in_quotes && (c == '#' || c == ';'))
        {
            s.erase(i); // 删除从注释起到末尾
            break;
        }
    }
    // 再修剪一次
    s = trim_copy(s);
}

/// 从 KV 获取字符串（并打印来源）
static std::string get_str_log(const std::unordered_map<std::string, std::string>& kv,
                               const char* key,
                               const std::string& def)
{
    auto it = kv.find(key);
    std::string val = (it == kv.end()) ? def : trim_copy(it->second);
    LOG("Config", std::string(key) + " = \"" + val + "\" (" + (it == kv.end() ? "default" : "ini") + ")");
    return val;
}

/// 从 KV 获取整数（容忍 "30.0" 这样的浮点字符串；并打印来源/转换）
static int get_int_flexible(const std::unordered_map<std::string, std::string>& kv,
                            const char* key, int def)
{
    auto it = kv.find(key);
    if (it == kv.end()) {
        LOG("Config", std::string(key) + " not found, use default: " + std::to_string(def));
        return def;
    }
    std::string s = trim_copy(it->second);
    if (s.empty()) {
        LOG("Config", std::string(key) + " empty, use default: " + std::to_string(def));
        return def;
    }
    try {
        size_t idx = 0;
        int v = std::stoi(s, &idx);
        if (idx == s.size()) {
            LOG("Config", std::string(key) + " = " + std::to_string(v) + " (ini/int)");
            return v;
        }
    } catch (...) {
        // fallthrough
    }
    try {
        double d = std::stod(s);
        int v = static_cast<int>(std::lround(d)); // 四舍五入
        LOG("Config", std::string(key) + " = " + std::to_string(v) +
                     " (ini/float->int, raw=\"" + s + "\")");
        return v;
    } catch (...) {
        LOG("Config", std::string(key) + " invalid(\"" + s + "\"), use default: " + std::to_string(def));
        return def;
    }
}

// ---------------- 主函数：读取 INI 并映射到 AppConfig ----------------

bool loadConfig(const std::string& path, AppConfig& cfg)
{
    std::ifstream in(path);
    if(!in.is_open()) {
        LOG("INI", "failed to open: " + path);
        return false;
    }

    LOG("INI", "loading: " + path);

    std::unordered_map<std::string, std::string> kv;

    std::string line, section;
    size_t line_no = 0;

    while (std::getline(in, line)) {
        ++line_no;

        if (line_no == 1) {
            remove_utf8_bom(line);
        }

        std::string original = line;           // 原始行（用于日志）
        line = trim_copy(line);                // ★ 记得赋值

        if (line.empty()) {
            LOG("INI", "skip blank line " + std::to_string(line_no) + ": \"" + original + "\"");
            continue;
        }
        if (line[0] == '#' || line[0] == ';') {
            LOG("INI", "skip comment line " + std::to_string(line_no) + ": \"" + original + "\"");
            continue;
        }

        // section
        if (line.front() == '[' && line.back() == ']') {
            section = line.substr(1, line.size()-2);
            LOG("INI", "section [" + section + "] (line " + std::to_string(line_no) + ")");
            continue;
        }

        // key=value
        auto pos = line.find('=');
        if (pos == std::string::npos) {
            LOG("INI", "ignore line " + std::to_string(line_no) + " (no '='): \"" + original + "\"");
            continue;
        }

        std::string key = trim_copy(std::string_view(line).substr(0, pos));
        std::string val = std::string(line.substr(pos+1));

        // 行内注释处理（值右侧的 # 或 ;）
        strip_inline_comment(val);

        if (key.empty()) {
            LOG("INI", "ignore line " + std::to_string(line_no) + " (empty key): \"" + original + "\"");
            continue;
        }

        // 扁平键：直接用 key；若需要带 section，可改为 section + "." + key
        auto it = kv.find(key);
        if (it != kv.end()) {
            LOG("INI", "overwrite key \"" + key + "\" from \"" + it->second +
                       "\" to \"" + val + "\" (line " + std::to_string(line_no) + ")");
            it->second = val;
        } else {
            kv.emplace(key, val);
            LOG("INI", "set " + (section.empty() ? "" : ("[" + section + "] ")) +
                       key + " = \"" + val + "\" (line " + std::to_string(line_no) + ")");
        }
    }

    // 解析完成后，统一 dump 一份
    LOG("INI", "---- KV dump ----");
    for (const auto& [k, v] : kv) {
        LOG("INI", k + " = \"" + v + "\"");
    }
    LOG("INI", "------------------");

    // ---------- 映射到 AppConfig（并打印每个值） ----------
    // 若某些字段你尚未在 AppConfig 定义，请到 common/config.h 添加对应成员。

    // 行为树主 ID（可选：在 ini 里配置 main_tree_id）
    cfg.main_tree_id      = get_str_log(kv, "main_tree_id",      cfg.main_tree_id);


    // Groot2 端口
    cfg.groot_port        = get_int_flexible(kv, "groot_port",    cfg.groot_port);

    // 电池相关
    cfg.battery_threshold = get_int_flexible(kv, "battery_threshold", cfg.battery_threshold);
    cfg.battery_pct       = get_int_flexible(kv, "battery_pct",       cfg.battery_pct);


    // 巡航各点（字符串，直接给节点作为端口值）
    cfg.goal_A            = get_str_log(kv, "goal_A",            cfg.goal_A);
    cfg.goal_B            = get_str_log(kv, "goal_B",            cfg.goal_B);
    cfg.goal_C            = get_str_log(kv, "goal_C",            cfg.goal_C);
    cfg.goal_D            = get_str_log(kv, "goal_D",            cfg.goal_D);

    LOG("INI", "load done: " + path);
    return true;
}

