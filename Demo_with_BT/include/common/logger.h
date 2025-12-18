#pragma once
#include <iostream>
#include <chrono>
#include <iomanip>
#include <string>

inline void LOG(const std::string& tag, const std::string& msg)
{
    using clk = std::chrono::system_clock;
    auto t = clk::to_time_t(clk::now());
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::cout << "[" << std::put_time(&tm, "%F %T") << "][" << tag << "] " << msg << std::endl;
}

