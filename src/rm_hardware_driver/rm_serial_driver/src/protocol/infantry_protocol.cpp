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

#include "rm_serial_driver/protocol/infantry_protocol.hpp"

#include <cmath>
#include <cstring>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace fyt::serial_driver::protocol {

ProtocolInfantry::ProtocolInfantry(
  std::string_view port_name,
  bool enable_data_print,
  InfantryProtocolVersion version)
: version_(version) {
  (void)enable_data_print;  // 当前这版不用 16/32/64 FixedPacketTool，不打印十六进制流

  transporter_ = std::make_shared<UartTransporter>(std::string(port_name));
  if (!transporter_->open()) {
    last_error_message_ = transporter_->errorMessage();
  } else {
    last_error_message_ = "ok";
  }
}

bool ProtocolInfantry::readExact(void *buffer, size_t len) {
  if (!transporter_) {
    last_error_message_ = "transporter is null";
    return false;
  }

  if (!transporter_->isOpen()) {
    if (!transporter_->open()) {
      last_error_message_ = transporter_->errorMessage();
      return false;
    }
  }

  auto *ptr = reinterpret_cast<uint8_t *>(buffer);
  size_t total = 0;
  while (total < len) {
    int ret = transporter_->read(ptr + total, len - total);
    if (ret <= 0) {
      last_error_message_ = transporter_->errorMessage().empty()
                              ? "read failed"
                              : transporter_->errorMessage();
      return false;
    }
    total += static_cast<size_t>(ret);
  }
  return true;
}

bool ProtocolInfantry::writeExact(const void *buffer, size_t len) {
  if (!transporter_) {
    last_error_message_ = "transporter is null";
    return false;
  }

  if (!transporter_->isOpen()) {
    if (!transporter_->open()) {
      last_error_message_ = transporter_->errorMessage();
      return false;
    }
  }

  const auto *ptr = reinterpret_cast<const uint8_t *>(buffer);
  size_t total = 0;
  while (total < len) {
    int ret = transporter_->write(ptr + total, len - total);
    if (ret <= 0) {
      last_error_message_ = transporter_->errorMessage().empty()
                              ? "write failed"
                              : transporter_->errorMessage();
      return false;
    }
    total += static_cast<size_t>(ret);
  }
  return true;
}

bool ProtocolInfantry::syncToHeaderGV() {
  uint8_t first = 0;
  uint8_t second = 0;

  while (rclcpp::ok()) {
    if (!readExact(&first, 1)) {
      return false;
    }
    if (first != 'G') {
      continue;
    }

    if (!readExact(&second, 1)) {
      return false;
    }

    if (second == 'V') {
      return true;
    }
  }

  last_error_message_ = "interrupted while syncing GV header";
  return false;
}

void ProtocolInfantry::send(const rm_interfaces::msg::GimbalCmd &data) {
  switch (version_) {
    case InfantryProtocolVersion::AutoAimV1:
    default:
      sendVisionCmdV1(data);
      break;
  }
}

void ProtocolInfantry::sendVisionCmdV1(const rm_interfaces::msg::GimbalCmd &data) {
  VisionToGimbalAutoAim packet;

  // FYT_WS 内部消息层统一使用弧度制；该协议本身也使用弧度制。
  packet.mode = data.control ? (data.fire_advice ? 2 : 1) : 0;

  packet.yaw = static_cast<float>(data.yaw);
  packet.yaw_vel = static_cast<float>(data.yaw_vel);
  packet.yaw_acc = static_cast<float>(data.yaw_acc);

  packet.pitch = static_cast<float>(data.pitch);
  packet.pitch_vel = static_cast<float>(data.pitch_vel);
  packet.pitch_acc = static_cast<float>(data.pitch_acc);

  if (!writeExact(&packet, sizeof(packet))) {
    return;
  }

  last_error_message_ = "ok";
}

bool ProtocolInfantry::receive(rm_interfaces::msg::SerialReceiveData &data) {
  switch (version_) {
    case InfantryProtocolVersion::AutoAimV1:
    default:
      return recvGimbalStateV1(data);
  }
}

bool ProtocolInfantry::recvGimbalStateV1(rm_interfaces::msg::SerialReceiveData &data) {
  if (!syncToHeaderGV()) {
    return false;
  }

  GimbalToVisionAutoAim packet;
  packet.head[0] = 'G';
  packet.head[1] = 'V';

  // 头已经读过了，所以从 mode 开始继续读剩余字节
  if (!readExact(reinterpret_cast<uint8_t *>(&packet) + 2, sizeof(packet) - 2)) {
    return false;
  }

  if (packet.tail != 'G') {
    last_error_message_ = "invalid tail for GimbalToVisionAutoAim";
    return false;
  }

  data.mode = packet.mode;
  data.bullet_speed = packet.bullet_speed;
  data.bullet_count = packet.bullet_count;

  tf2::Quaternion q(packet.q[1], packet.q[2], packet.q[3], packet.q[0]);  // x y z w
  double roll_rad = 0.0;
  double pitch_rad = 0.0;
  double yaw_rad = 0.0;
  tf2::Matrix3x3(q).getRPY(roll_rad, pitch_rad, yaw_rad);

  // pitch 依旧保持“消息层正上抬、TF层取负”的兼容语义，但单位统一为弧度
  data.roll = static_cast<float>(roll_rad);
  data.pitch = static_cast<float>(-pitch_rad);
  data.yaw = static_cast<float>(yaw_rad);
  data.yaw_vel = packet.yaw_vel;
  data.pitch_vel = -packet.pitch_vel;

  last_error_message_ = "ok";
  return true;
}

std::vector<rclcpp::SubscriptionBase::SharedPtr> ProtocolInfantry::getSubscriptions(
  rclcpp::Node::SharedPtr node) {
  auto sub1 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "armor_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { this->send(*msg); });

  auto sub2 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "rune_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { this->send(*msg); });

  return {sub1, sub2};
}

std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> ProtocolInfantry::getClients(
  rclcpp::Node::SharedPtr node) const {
  auto client1 = node->create_client<rm_interfaces::srv::SetMode>(
    "armor_detector/set_mode", rmw_qos_profile_services_default);
  auto client2 = node->create_client<rm_interfaces::srv::SetMode>(
    "armor_solver/set_mode", rmw_qos_profile_services_default);
  auto client3 = node->create_client<rm_interfaces::srv::SetMode>(
    "rune_detector/set_mode", rmw_qos_profile_services_default);
  auto client4 = node->create_client<rm_interfaces::srv::SetMode>(
    "rune_solver/set_mode", rmw_qos_profile_services_default);

  return {client1, client2, client3, client4};
}

}  // namespace fyt::serial_driver::protocol
