#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rm_interfaces/msg/serial_receive_data.hpp>  // 添加这个头文件
#include "imu_driver.h"

class ImuDriverNode : public rclcpp::Node
{
public:
  ImuDriverNode()
  : Node("imu_driver_node")
  {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "/dm_imu/data_raw", rclcpp::SensorDataQoS());

    // 添加串口数据发布器
    serial_pub_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>(
      "/serial_receive", 10);

    // 从 ROS2 参数读取配置
    std::string port = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    int baud = this->declare_parameter<int>("baud", 921600);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "imu_link");
    double publish_rate_hz = this->declare_parameter<double>("publish_rate", 300.0);
    if (publish_rate_hz <= 0.0) publish_rate_hz = 300.0;

    // 初始化 IMU 驱动
    imu_driver_ = std::make_unique<io::DmImu>(port, baud);

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz));
    timer_ = this->create_wall_timer(period, std::bind(&ImuDriverNode::publish_imu_data, this));

    RCLCPP_INFO(this->get_logger(), "IMU driver node started. port=%s baud=%d rate=%.1f Hz frame=%s",
                port.c_str(), baud, publish_rate_hz, frame_id_.c_str());
  }

private:
  void publish_imu_data()
  {
    auto q    = imu_driver_->latest_quaternion();
    auto acc  = imu_driver_->latest_accel();
    auto gyro = imu_driver_->latest_gyro();

    // 发布原始 IMU 数据
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = frame_id_;

    imu_msg.orientation.w = q.w();
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();

    imu_msg.linear_acceleration.x = acc[0];
    imu_msg.linear_acceleration.y = acc[1];
    imu_msg.linear_acceleration.z = acc[2];

    imu_msg.angular_velocity.x = gyro[0];
    imu_msg.angular_velocity.y = gyro[1];
    imu_msg.angular_velocity.z = gyro[2];

    imu_msg.orientation_covariance[0]        = -1.0;
    imu_msg.angular_velocity_covariance[0]   = -1.0;
    imu_msg.linear_acceleration_covariance[0]= -1.0;

    imu_pub_->publish(imu_msg);

    // ========== 在这里添加串口数据发送 ==========
    publish_serial_data(q);
  }

  // 新增：发送符合串口格式要求的数据
  void publish_serial_data(const Eigen::Quaterniond& q)
  {
    rm_interfaces::msg::SerialReceiveData serial_msg;
    
    // 设置消息头
    serial_msg.header.stamp = this->now();
    serial_msg.header.frame_id = frame_id_;
    
    // 设置模式
    serial_msg.mode = 1;  // 可以从参数读取或根据状态设置
    
    // 设置弹速
    serial_msg.bullet_speed = 15.0;  // 可以从参数读取
    
    // 从四元数计算欧拉角（roll, pitch, yaw）
    auto euler = quaternion_to_euler(q);
    serial_msg.roll = euler[0];
    serial_msg.pitch = euler[1];
    serial_msg.yaw = euler[2];
    
    // judge_system_data 根据实际需求填充，这里留空或设默认值
    // serial_msg.judge_system_data = ...;
    
    serial_pub_->publish(serial_msg);
  }

  // 四元数转欧拉角（roll, pitch, yaw）单位：弧度
  std::array<float, 3> quaternion_to_euler(const Eigen::Quaterniond& q)
  {
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    float roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    float pitch;
    if (std::abs(sinp) >= 1)
      pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
      pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    float yaw = std::atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
  }

  std::unique_ptr<io::DmImu> imu_driver_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_pub_;  // 新增
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ImuDriverNode>());
  } catch (const std::exception &e) {
    fprintf(stderr, "[imu_driver_node] initialization failed: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
