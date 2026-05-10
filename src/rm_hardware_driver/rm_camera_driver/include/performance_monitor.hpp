#ifndef PERFORMANCE_MONITOR_HPP
#define PERFORMANCE_MONITOR_HPP

#include <chrono>
#include <string>
#include <mutex>
#include <unordered_map>

namespace utils
{

class PerformanceMonitor
{
public:
    // 完全匹配代码的 Config 结构体（包含 print_interval_sec 和 logger_name）
    struct Config
    {
        bool enable_logging = false;
        double print_interval_sec = 1.0;  // 新增：代码中用到的参数
        std::string logger_name = "performance_monitor";  // 新增：代码中用到的参数
    };

    explicit PerformanceMonitor(const std::string& name = "camera")
        : name_(name), config_(Config{}) {}
    ~PerformanceMonitor() = default;  // 保留默认析构，删除重复定义

    // 匹配代码的 set_config 方法
    void set_config(const Config& config)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = config;
    }

    // 匹配代码的 register_metric 方法（空实现，满足编译）
    void register_metric(const std::string& metric_name)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // 可添加实际统计逻辑，此处仅为编译通过
        if (metric_stats_.count(metric_name) == 0)
            metric_stats_[metric_name] = 0.0;
    }

    // 匹配代码的 Timer 类（包含 set_success 方法）
    class Timer
    {
    public:
        Timer() : start_(std::chrono::high_resolution_clock::now()), success_(false) {}
        ~Timer()
        {
            // 计算耗时（避免未使用变量警告）
            auto end = std::chrono::high_resolution_clock::now();
            [[maybe_unused]] auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_).count() / 1000.0;
        }

        // 新增：匹配代码的 set_success 方法
        void set_success(bool success) { success_ = success; }

    private:
        std::chrono::high_resolution_clock::time_point start_;
        bool success_;
    };

    // 匹配代码的 create_timer 方法
    Timer create_timer(const std::string& timer_name)
    {
        return Timer();
    }

private:
    std::string name_;
    Config config_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, double> metric_stats_;
};

}  // namespace utils

#endif // PERFORMANCE_MONITOR_HPP
