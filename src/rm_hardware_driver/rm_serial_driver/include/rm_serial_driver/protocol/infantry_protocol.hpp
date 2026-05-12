// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SERIAL_DRIVER_INFANTRY_PROTOCOL_HPP_
#define SERIAL_DRIVER_INFANTRY_PROTOCOL_HPP_

#include "rm_serial_driver/protocol.hpp"
#include <sensor_msgs/msg/imu.hpp>

namespace fyt::serial_driver::protocol {

enum class InfantryProtocolVersion : uint8_t {
  Legacy16 = 0,   // 旧版16字节协议，仅发送 fire/pitch/yaw/distance
  Feedforward = 1 // 新版协议，支持 yaw_vel/pitch_vel/yaw_acc/pitch_acc
};

// 步兵通信协议
class ProtocolInfantry : public Protocol {
public:
  explicit ProtocolInfantry(std::string_view port_name,
                            bool enable_data_print,
                            InfantryProtocolVersion version =
  InfantryProtocolVersion::Legacy16);

  ~ProtocolInfantry() = default;

  // Send gimbal command to lower controller.
    //
    // Legacy16:
    //   - send fire flag
    //   - send pitch / yaw / distance only
    //
    // Feedforward:
    //   - send control flag
    //   - send fire flag
    //   - send pitch / yaw
    //   - send yaw_vel / pitch_vel
    //   - send yaw_acc / pitch_acc
    //   - optional distance field

  void send(const rm_interfaces::msg::GimbalCmd &data) override;

  bool receive(rm_interfaces::msg::SerialReceiveData &data) override;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> getSubscriptions(
    rclcpp::Node::SharedPtr node) override;

  std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> getClients(
    rclcpp::Node::SharedPtr node) const override;

  std::string getErrorMessage() override { return packet_tool_->getErrorMessage(); }

private:
  void sendImuData(const sensor_msgs::msg::Imu::SharedPtr msg);

  InfantryProtocolVersion version_;
  FixedPacketTool<16>::SharedPtr packet_tool_;
};
}  // namespace fyt::serial_driver::protocol

#endif  // SERIAL_DRIVER_INFANTRY_PROTOCOL_HPP_
