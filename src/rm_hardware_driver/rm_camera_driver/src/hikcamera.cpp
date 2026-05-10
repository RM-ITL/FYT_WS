#include "hikvision/hikcamera.hpp"
#include <libusb-1.0/libusb.h>
#include <fstream>
#include "rclcpp_components/register_node_macro.hpp"
#include <camera_info_manager/camera_info_manager.hpp>   // 新增
#include <sensor_msgs/msg/camera_info.hpp>               // 新增

namespace camera  
{

using namespace std::chrono_literals;

HikRobotNode::HikRobotNode(const rclcpp::NodeOptions& options)
: Node("hik_robot_node", options), queue_(3)
{
    // 从ROS参数获取配置路径
    std::string config_path;
    this->declare_parameter("config_path", "/home/hou/FYT_WS/src/rm_bringup/config/camera_config.yaml");
    this->declare_parameter("camera_name", "hik_camera");   // 新增
    this->declare_parameter("camera_info_url", "file:///home/hou/FYT_WS/src/rm_bringup/config/camera_calib.yaml"); // 新增
    
    this->get_parameter("config_path", config_path);
    std::string camera_name, camera_info_url;
    this->get_parameter("camera_name", camera_name);
    this->get_parameter("camera_info_url", camera_info_url);

    if (!loadConfig(config_path)) {
        RCLCPP_ERROR(this->get_logger(), "配置加载失败");
        throw std::runtime_error("Failed to load configuration");
    }
    
    // 初始化 CameraInfo 管理器和发布器
    cam_info_mgr_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name, camera_info_url);
    cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS());

    libusb_init(NULL);
    
    auto qos = rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::BestEffort);
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);
    
    utils::PerformanceMonitor::Config perf_config;
    perf_config.enable_logging = true;
    perf_config.print_interval_sec = 5.0;
    perf_config.logger_name = this->get_logger().get_name();
    perf_monitor_.set_config(perf_config);
    perf_monitor_.register_metric("capture");
    perf_monitor_.register_metric("process");
    
    daemon_thread_ = std::thread(&HikRobotNode::daemonThread, this);
    process_thread_ = std::thread(&HikRobotNode::processThread, this);
}

HikRobotNode::HikRobotNode(const std::string& config_path, const std::string& name)
: Node(name), queue_(3)
{
    this->declare_parameter("camera_name", "hik_camera"); 
    this->declare_parameter("camera_info_url", "file:///home/hou/FYT_WS/src/rm_bringup/config/camera_calib.yaml");
    std::string camera_name, camera_info_url;
    this->get_parameter("camera_name", camera_name);
    this->get_parameter("camera_info_url", camera_info_url);

    if (!loadConfig(config_path)) {
        RCLCPP_ERROR(this->get_logger(), "配置加载失败");
        throw std::runtime_error("Failed to load configuration");
    }

    cam_info_mgr_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name, camera_info_url);
    cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS());

    libusb_init(NULL);
    
    auto qos = rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::BestEffort);
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);
    
    utils::PerformanceMonitor::Config perf_config;
    perf_config.enable_logging = true;
    perf_config.print_interval_sec = 5.0;
    perf_config.logger_name = this->get_logger().get_name();
    perf_monitor_.set_config(perf_config);
    perf_monitor_.register_metric("capture");
    perf_monitor_.register_metric("process");
    
    daemon_thread_ = std::thread(&HikRobotNode::daemonThread, this);
    process_thread_ = std::thread(&HikRobotNode::processThread, this);
}

HikRobotNode::~HikRobotNode()
{
    node_shutdown_ = true;
    daemon_quit_ = true;
    capture_quit_ = true;
    
    if (daemon_thread_.joinable()) daemon_thread_.join();
    if (capture_thread_.joinable()) capture_thread_.join();
    if (process_thread_.joinable()) process_thread_.join();
    
    libusb_exit(NULL);
}

bool HikRobotNode::loadConfig(const std::string& config_path)
{
    try {
        std::ifstream file(config_path);
        if (!file.good()) {
            RCLCPP_ERROR(this->get_logger(), "配置文件不存在: %s", config_path.c_str());
            return false;
        }
        file.close();
        
        YAML::Node config = YAML::LoadFile(config_path);
        const YAML::Node& params = config["camera"]["parameters"];
        
        exposure_us_ = params["exposure_ms"].as<double>() * 1000.0;
        gain_ = params["gain"].as<double>();
        fps_ = params["fps"].as<double>();
        target_size_ = cv::Size(
            params["target_width"].as<int>(),
            params["target_height"].as<int>()
        );
        image_topic_ = params["image_topic"].as<std::string>();
        
        RCLCPP_INFO(this->get_logger(), "配置加载成功 - 曝光:%.1fms 增益:%.1f FPS:%.1f",
                    exposure_us_/1000.0, gain_, fps_);
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "配置加载错误: %s", e.what());
        return false;
    }
}

void HikRobotNode::daemonThread()
{
    startCapture();
    
    while (!daemon_quit_) {
        std::this_thread::sleep_for(1s);
        
        if (capturing_) continue;
        
        RCLCPP_WARN(this->get_logger(), "相机停止工作，尝试恢复...");
        stopCapture();
        resetUSB();
        std::this_thread::sleep_for(500ms);
        startCapture();
    }
    
    stopCapture();
}

bool HikRobotNode::startCapture()
{
    capturing_ = false;
    capture_quit_ = false;
    
    MV_CC_DEVICE_INFO_LIST device_list;
    if (MV_CC_EnumDevices(MV_USB_DEVICE, &device_list) != MV_OK || device_list.nDeviceNum == 0) {
        RCLCPP_ERROR(this->get_logger(), "未找到相机");
        return false;
    }
    
    if (MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]) != MV_OK ||
        MV_CC_OpenDevice(camera_handle_) != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "打开相机失败");
        if (camera_handle_) {
            MV_CC_DestroyHandle(camera_handle_);
            camera_handle_ = nullptr;
        }
        return false;
    }
    
    setCameraParameters();
    
    MVCC_INTVALUE stParam = {0};
    MV_CC_GetIntValue(camera_handle_, "PayloadSize", &stParam);
    payload_size_ = stParam.nCurValue;
    raw_buffer_.reset(new unsigned char[payload_size_]);
    
    if (MV_CC_StartGrabbing(camera_handle_) != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "开始采集失败");
        return false;
    }
    
    capture_thread_ = std::thread(&HikRobotNode::captureThread, this);
    RCLCPP_INFO(this->get_logger(), "相机开始采集");
    return true;
}

void HikRobotNode::stopCapture()
{
    capture_quit_ = true;
    if (capture_thread_.joinable()) capture_thread_.join();
    
    if (camera_handle_) {
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
    }
    
    raw_buffer_.reset();
}

void HikRobotNode::setCameraParameters()
{
    MV_CC_SetEnumValue(camera_handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
    MV_CC_SetEnumValue(camera_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_us_);
    MV_CC_SetEnumValue(camera_handle_, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain_);
    MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", fps_);
}

void HikRobotNode::captureThread()
{
    capturing_ = true;
    MV_FRAME_OUT_INFO_EX frame_info = {0};
    
    while (!capture_quit_) {
        auto timer = perf_monitor_.create_timer("capture");
        
        int ret = MV_CC_GetOneFrameTimeout(camera_handle_, raw_buffer_.get(), 
                                          payload_size_, &frame_info, 1000);
        
        if (ret == MV_OK) {

            // 用 ROS 的时间戳
            rclcpp::Time timestamp = this->now();

            cv::Mat raw_image(frame_info.nHeight, frame_info.nWidth, CV_8UC1, raw_buffer_.get());
            cv::Mat rgb_image = convertBayer(raw_image, frame_info.enPixelType);
            cv::Mat resized;
            cv::resize(rgb_image, resized, target_size_, 0, 0, cv::INTER_LINEAR);
            
              // 显式构造 CameraData
            CameraData camera_data;
            camera_data.img = resized.clone();
            camera_data.stamp = timestamp;
            queue_.push(camera_data);

            timer.set_success(true);
        } else {
            timer.set_success(false);
            capturing_ = false;
            break;
        }
        
        std::this_thread::sleep_for(1ms);
    }
    
    capturing_ = false;
}

void HikRobotNode::processThread()
{
    while (!node_shutdown_) {
        CameraData data;
        queue_.pop(data);
        
        if (!data.img.empty()) {
            auto timer = perf_monitor_.create_timer("process");

            // 发布图像
            auto img_msg = cv_bridge::CvImage();
            img_msg.header.stamp = data.stamp;      // 用采集时的 stamp，保证两者一致
            img_msg.header.frame_id = "camera_link"; // 与 TF 树一致
            img_msg.encoding = "rgb8";
            img_msg.image = data.img;
            image_pub_->publish(*img_msg.toImageMsg());

            // 发布 CameraInfo
            auto ci = cam_info_mgr_->getCameraInfo();
            ci.header = img_msg.header;
            ci.width  = data.img.cols;
            ci.height = data.img.rows;
            cam_info_pub_->publish(ci);
        }
        
        std::this_thread::yield();
    }
}

cv::Mat HikRobotNode::convertBayer(const cv::Mat& raw, unsigned int type)
{
    cv::Mat bgr;
    static const std::unordered_map<unsigned int, int> bayer_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2BGR},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2BGR},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2BGR},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2BGR}
    };
    
    auto it = bayer_map.find(type);
    if (it != bayer_map.end()) {
        cv::cvtColor(raw, bgr, it->second);
    } else if (type == PixelType_Gvsp_Mono8) {
        cv::cvtColor(raw, bgr, cv::COLOR_GRAY2BGR);
    } else {
        bgr = raw.clone();
    }
    return bgr;
}

void HikRobotNode::resetUSB() const
{
    auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
    if (handle) {
        libusb_reset_device(handle);
        libusb_close(handle);
    }
}

} // namespace camera

RCLCPP_COMPONENTS_REGISTER_NODE(camera::HikRobotNode)

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    std::string config_path = "/home/hou/FYT_WS/src/rm_bringup/config/camera_config.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }
    
    try {
        auto node = std::make_shared<camera::HikRobotNode>(config_path);
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "启动失败: " << e.what() << std::endl;
        return -1;
    }
    
    rclcpp::shutdown();
    return 0;
}
