#ifndef HIKCAMERA_HPP
#define HIKCAMERA_HPP

#include <memory>
#include <thread>
#include <atomic>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <camera_info_manager/camera_info_manager.hpp>

#include "MvCameraControl.h"
#include "thread_safe_queue.hpp"
#include "performance_monitor.hpp"

// 注意：不要在头文件里包含 register_node_macro.hpp 和注册宏，放在 .cpp 里

namespace camera
{

struct CameraData {
    cv::Mat img;
    rclcpp::Time stamp;  // 用 rclcpp::Time，方便直接赋给 ROS Header
};

class HikRobotNode : public rclcpp::Node
{
public:
    explicit HikRobotNode(const rclcpp::NodeOptions& options);
    HikRobotNode(const std::string& config_path, const std::string& name = "hikrobot_node");
    ~HikRobotNode();

private:
    void daemonThread();
    void captureThread();
    void processThread();
    bool startCapture();
    void stopCapture();
    void setCameraParameters();
    cv::Mat convertBayer(const cv::Mat& raw, unsigned int type);
    void resetUSB() const;
    bool loadConfig(const std::string& config_path);

private:
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;  // 新增

    // CameraInfo manager
    std::unique_ptr<camera_info_manager::CameraInfoManager> cam_info_mgr_;     // 新增

    // SDK / buffers
    void* camera_handle_ = nullptr;
    unsigned int payload_size_ = 0;
    std::unique_ptr<unsigned char[]> raw_buffer_;

    // Threads / state
    std::thread daemon_thread_, capture_thread_, process_thread_;
    std::atomic<bool> daemon_quit_{false};
    std::atomic<bool> capture_quit_{false};
    std::atomic<bool> capturing_{false};
    std::atomic<bool> node_shutdown_{false};

    // Queues / perf
    utils::ThreadSafeQueue<CameraData> queue_;
    utils::PerformanceMonitor perf_monitor_;

    // Params
    double exposure_us_, gain_, fps_;
    cv::Size target_size_;
    std::string image_topic_;
    int vid_ = 0x2bdf;
    int pid_ = 0x0299;
};

} // namespace camera

#endif // HIKCAMERA_HPP
