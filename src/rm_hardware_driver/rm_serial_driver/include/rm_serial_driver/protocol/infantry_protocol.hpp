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

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "rm_serial_driver/protocol.hpp"

namespace fyt::serial_driver::protocol {

enum class InfantryProtocolVersion : uint8_t {
  AutoAimV1 = 0
};

#pragma pack(push, 1)

struct VisionToGimbalAutoAim {
  uint8_t head[2] = {'V', 'G'};
  uint8_t mode = 0;  // 0: idle, 1: control no fire, 2: control fire

  float yaw = 0.0f;       // rad
  float yaw_vel = 0.0f;   // rad/s
  float yaw_acc = 0.0f;   // rad/s^2

  float pitch = 0.0f;     // rad
  float pitch_vel = 0.0f; // rad/s
  float pitch_acc = 0.0f; // rad/s^2

  uint8_t tail = 'V';
};

struct GimbalToVisionAutoAim {
  uint8_t head[2] = {'G', 'V'};
  uint8_t mode = 0;

  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z

  float yaw = 0.0f;       // rad
  float yaw_vel = 0.0f;   // rad/s
  float pitch = 0.0f;     // rad
  float pitch_vel = 0.0f; // rad/s

  float bullet_speed = 0.0f;
  uint16_t bullet_count = 0;

  uint8_t tail = 'G';
};

#pragma pack(pop)

static_assert(sizeof(VisionToGimbalAutoAim) == 28, "VisionToGimbalAutoAim size must be 28 bytes");
static_assert(sizeof(GimbalToVisionAutoAim) == 42, "GimbalToVisionAutoAim size must be 42 bytes");

class ProtocolInfantry : public Protocol {
public:
  explicit ProtocolInfantry(
    std::string_view port_name,
    bool enable_data_print,
    InfantryProtocolVersion version = InfantryProtocolVersion::AutoAimV1);

  ~ProtocolInfantry() override = default;

  void send(const rm_interfaces::msg::GimbalCmd &data) override;

  bool receive(rm_interfaces::msg::SerialReceiveData &data) override;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> getSubscriptions(
    rclcpp::Node::SharedPtr node) override;

  std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> getClients(
    rclcpp::Node::SharedPtr node) const override;

  std::string getErrorMessage() override { return last_error_message_; }

private:
  enum class AngleUnit : uint8_t {
    Unknown = 0,
    Radian = 1,
    Degree = 2,
  };

  void sendVisionCmdV1(const rm_interfaces::msg::GimbalCmd &data);
  bool recvGimbalStateV1(rm_interfaces::msg::SerialReceiveData &data);

  bool readExact(void *buffer, size_t len);
  bool writeExact(const void *buffer, size_t len);
  bool syncToHeaderGV();

private:
  InfantryProtocolVersion version_;
  AngleUnit angle_unit_ = AngleUnit::Unknown;
  TransporterInterface::SharedPtr transporter_;
  std::string last_error_message_ = "unknown";
};

}  // namespace fyt::serial_driver::protocol

#endif  // SERIAL_DRIVER_INFANTRY_PROTOCOL_HPP_
