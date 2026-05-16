// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
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

#include "armor_solver/armor_solver.hpp"
// std
#include <cmath>
#include <cstddef>
#include <stdexcept>
// project
#include "armor_solver/armor_solver_node.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace fyt::auto_aim {
Solver::Solver(std::weak_ptr<rclcpp::Node> n) : node_(n) {
  auto node = node_.lock();

  shooting_range_w_ = node->declare_parameter("solver.shooting_range_width", 0.135);
  shooting_range_h_ = node->declare_parameter("solver.shooting_range_height", 0.135);
  max_tracking_v_yaw_ = node->declare_parameter("solver.max_tracking_v_yaw", 6.0);
  prediction_delay_ = node->declare_parameter("solver.prediction_delay", 0.0);
  controller_delay_ = node->declare_parameter("solver.controller_delay", 0.0);
  side_angle_ = node->declare_parameter("solver.side_angle", 15.0);
  min_switching_v_yaw_ = node->declare_parameter("solver.min_switching_v_yaw", 1.0);

  std::string compenstator_type = node->declare_parameter("solver.compensator_type", "ideal");
  trajectory_compensator_ = CompensatorFactory::createCompensator(compenstator_type);
  trajectory_compensator_->iteration_times = node->declare_parameter("solver.iteration_times", 20);
  default_bullet_speed_ = node->declare_parameter("solver.bullet_speed", 20.0);
  trajectory_compensator_->velocity = default_bullet_speed_;

  feedforward_alpha_ = node->declare_parameter("solver.feedforward_alpha", 0.25);
  enable_acceleration_feedforward_ =
    node->declare_parameter("solver.enable_acceleration_feedforward", false);
  predictive_ff_dt_ = node->declare_parameter("solver.predictive_ff_dt", 0.01);
  max_yaw_vel_ = node->declare_parameter("solver.max_yaw_vel", 15.0);
  max_pitch_vel_ = node->declare_parameter("solver.max_pitch_vel", 15.0);

  // 这两个优先对标 Auto_aim
  max_yaw_acc_ = node->declare_parameter("solver.max_yaw_acc", 20.0);
  max_pitch_acc_ = node->declare_parameter("solver.max_pitch_acc", 50.0);

  // 突然跳变保护门限，单位 rad
  max_delta_yaw_for_feedforward_ =
    node->declare_parameter("solver.max_delta_yaw_for_feedforward", 8.0 * M_PI / 180.0);
  max_delta_pitch_for_feedforward_ =
    node->declare_parameter("solver.max_delta_pitch_for_feedforward", 8.0 * M_PI / 180.0);

  near_tolerance_yaw_ =
    node->declare_parameter("solver.near_tolerance_yaw", 1.8 * M_PI / 180.0);
  near_tolerance_pitch_ =
    node->declare_parameter("solver.near_tolerance_pitch", 1.8 * M_PI / 180.0);
  far_tolerance_yaw_ =
    node->declare_parameter("solver.far_tolerance_yaw", 1.3 * M_PI / 180.0);
  far_tolerance_pitch_ =
    node->declare_parameter("solver.far_tolerance_pitch", 1.3 * M_PI / 180.0);
  strict_distance_threshold_ =
    node->declare_parameter("solver.strict_distance_threshold", 2.0);

  trajectory_compensator_->gravity = node->declare_parameter("solver.gravity", 9.8);
  trajectory_compensator_->resistance = node->declare_parameter("solver.resistance", 0.001);

  manual_compensator_ = std::make_unique<ManualCompensator>();
  auto angle_offset = node->declare_parameter("solver.angle_offset", std::vector<std::string>{});
  if(!manual_compensator_->updateMapFlow(angle_offset)) {
    FYT_WARN("armor_solver", "Manual compensator update failed!");
  }

  state = State::TRACKING_ARMOR;
  overflow_count_ = 0;
  transfer_thresh_ = 5;

  node.reset();
}

void Solver::updateRuntimeState(const rm_interfaces::msg::GimbalState &state) {
  runtime_state_ = state;
  has_runtime_state_ = true;
}

rm_interfaces::msg::GimbalCmd Solver::solve(const rm_interfaces::msg::Target &target,
                                            const rclcpp::Time &current_time,
                                            std::shared_ptr<tf2_ros::Buffer> tf2_buffer_) {
  // Get newest parameters
  try {
    auto node = node_.lock();
    max_tracking_v_yaw_ = node->get_parameter("solver.max_tracking_v_yaw").as_double();
    prediction_delay_ = node->get_parameter("solver.prediction_delay").as_double();
    controller_delay_ = node->get_parameter("solver.controller_delay").as_double();
    side_angle_ = node->get_parameter("solver.side_angle").as_double();
    min_switching_v_yaw_ = node->get_parameter("solver.min_switching_v_yaw").as_double();
    predictive_ff_dt_ = node->get_parameter("solver.predictive_ff_dt").as_double();
    feedforward_alpha_ = node->get_parameter("solver.feedforward_alpha").as_double();
    enable_acceleration_feedforward_ =
      node->get_parameter("solver.enable_acceleration_feedforward").as_bool();
    max_yaw_vel_ = node->get_parameter("solver.max_yaw_vel").as_double();
    max_pitch_vel_ = node->get_parameter("solver.max_pitch_vel").as_double();
    max_yaw_acc_ = node->get_parameter("solver.max_yaw_acc").as_double();
    max_pitch_acc_ = node->get_parameter("solver.max_pitch_acc").as_double();
    max_delta_yaw_for_feedforward_ =
      node->get_parameter("solver.max_delta_yaw_for_feedforward").as_double();
    max_delta_pitch_for_feedforward_ =
      node->get_parameter("solver.max_delta_pitch_for_feedforward").as_double();
    near_tolerance_yaw_ = node->get_parameter("solver.near_tolerance_yaw").as_double();
    near_tolerance_pitch_ = node->get_parameter("solver.near_tolerance_pitch").as_double();
    far_tolerance_yaw_ = node->get_parameter("solver.far_tolerance_yaw").as_double();
    far_tolerance_pitch_ = node->get_parameter("solver.far_tolerance_pitch").as_double();
    strict_distance_threshold_ = node->get_parameter("solver.strict_distance_threshold").as_double();
    node.reset();
  } catch (const std::runtime_error &e) {
    FYT_ERROR("armor_solver", "{}", e.what());
  }

  // Get current roll, yaw and pitch of gimbal
  try {
    if (has_runtime_state_) {
      // 当前 runtime_state_ 中 yaw / pitch 仍然是 degree
      // Solver 内部一直按 rad 运算，所以这里先转 rad
      rpy_[0] = 0.0;
      rpy_[1] = -runtime_state_.pitch * M_PI / 180.0;
      rpy_[2] = runtime_state_.yaw * M_PI / 180.0;
    } else {
      auto gimbal_tf =
        tf2_buffer_->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);
      auto msg_q = gimbal_tf.transform.rotation;

      tf2::Quaternion tf_q;
      tf2::fromMsg(msg_q, tf_q);
      tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
      rpy_[1] = -rpy_[1];
    }
  } catch (tf2::TransformException &ex) {
    FYT_ERROR("armor_solver", "{}", ex.what());
    throw ex;
  }

  // Use flying time to approximately predict the position of target
  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);

  // 优先使用 runtime_state 的实时弹速；异常时回退到默认值
  if (has_runtime_state_ &&
      runtime_state_.bullet_speed > 10.0f &&
      runtime_state_.bullet_speed < 30.0f) {
    trajectory_compensator_->velocity = runtime_state_.bullet_speed;
  } else {
    trajectory_compensator_->velocity = default_bullet_speed_;
  }

  const double flying_time = trajectory_compensator_->getFlyingTime(target_position);
  const double base_delay =
    (current_time - rclcpp::Time(target.header.stamp)).seconds() + flying_time + prediction_delay_;

  // Initialize gimbal_cmd
  rm_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.header = target.header;

  switch (state) {
    case TRACKING_ARMOR: {
      if (std::abs(target.v_yaw) > max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) {
        state = TRACKING_CENTER;
      }

      break;
    }
    case TRACKING_CENTER: {
      if (std::abs(target.v_yaw) < max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) {
        state = TRACKING_ARMOR;
        overflow_count_ = 0;
      }
      break;
    }
  }

  const bool aim_center = (state == TRACKING_CENTER);

  auto cmd_prev = predictCommandAtTime(target, std::max(0.0, base_delay + controller_delay_ - predictive_ff_dt_), aim_center);
  auto cmd_now = predictCommandAtTime(target, std::max(0.0, base_delay + controller_delay_), aim_center);
  auto cmd_next = predictCommandAtTime(target, std::max(0.0, base_delay + controller_delay_ + predictive_ff_dt_), aim_center);

  if (!cmd_now.valid) {
    throw std::runtime_error("No valid predicted command");
  }

  const double cmd_yaw = cmd_now.yaw;
  const double cmd_pitch = cmd_now.pitch;
  gimbal_cmd.distance = cmd_now.distance;

  const bool level1 =
    isOnTarget(rpy_[2], rpy_[1], cmd_yaw, cmd_pitch, cmd_now.distance);
  const bool level2 =
    distance_based_shooter_check(rpy_[2], rpy_[1], cmd_yaw, cmd_pitch, cmd_now.distance);
  gimbal_cmd.fire_advice = level1 && level2;

  // ==================== 前馈输出开始 ====================
  double filtered_cmd_yaw = cmd_yaw;
  double filtered_cmd_pitch = cmd_pitch;
  double ff_yaw_vel = 0.0;
  double ff_pitch_vel = 0.0;
  double ff_yaw_acc = 0.0;
  double ff_pitch_acc = 0.0;

  bool feedforward_valid = false;

  const bool predictive_window_valid = cmd_prev.valid && cmd_next.valid;

  if (last_control_.valid && predictive_window_valid) {
    const double dt_ff = (current_time - last_control_.stamp).seconds();
    const double predictive_dt = std::max(1e-4, predictive_ff_dt_);
    const double raw_delta_yaw =
      angles::shortest_angular_distance(last_control_.yaw, cmd_now.yaw);
    const double raw_delta_pitch = cmd_now.pitch - last_control_.pitch;

    // 1. 时间戳异常保护
    if (dt_ff > 1e-4 && dt_ff < 0.2) {
      // 2. 突然跳变保护：切板/重捕获时不输出离谱前馈
      if (std::abs(raw_delta_yaw) < max_delta_yaw_for_feedforward_ &&
          std::abs(raw_delta_pitch) < max_delta_pitch_for_feedforward_) {
        // 3. 对最终控制角先做一级低通，再差分，降低静止目标噪声放大
        filtered_cmd_yaw = angles::normalize_angle(
          last_control_.filtered_yaw +
          feedforward_alpha_ *
            angles::shortest_angular_distance(last_control_.filtered_yaw, cmd_now.yaw));
        filtered_cmd_pitch =
          last_control_.filtered_pitch +
          feedforward_alpha_ * (cmd_now.pitch - last_control_.filtered_pitch);

        const double filtered_prev_yaw = angles::normalize_angle(
          last_control_.filtered_yaw +
          feedforward_alpha_ *
            angles::shortest_angular_distance(last_control_.filtered_yaw, cmd_prev.yaw));
        const double filtered_prev_pitch =
          last_control_.filtered_pitch +
          feedforward_alpha_ * (cmd_prev.pitch - last_control_.filtered_pitch);

        const double filtered_next_yaw = angles::normalize_angle(
          filtered_cmd_yaw +
          feedforward_alpha_ *
            angles::shortest_angular_distance(filtered_cmd_yaw, cmd_next.yaw));
        const double filtered_next_pitch =
          filtered_cmd_pitch +
          feedforward_alpha_ * (cmd_next.pitch - filtered_cmd_pitch);

        // 4. 使用预测轨迹的中心差分，替代单纯对最终命令做帧间差分
        ff_yaw_vel =
          angles::shortest_angular_distance(filtered_prev_yaw, filtered_next_yaw) / (2.0 * predictive_dt);
        ff_pitch_vel = (filtered_next_pitch - filtered_prev_pitch) / (2.0 * predictive_dt);

        // 5. 限幅速度
        ff_yaw_vel = std::clamp(ff_yaw_vel, -max_yaw_vel_, max_yaw_vel_);
        ff_pitch_vel = std::clamp(ff_pitch_vel, -max_pitch_vel_, max_pitch_vel_);

        // 6. 参考轨迹二阶差分作为加速度前馈，默认仍可关闭以抑制静止目标噪声
        if (enable_acceleration_feedforward_) {
          const double prev_to_now_yaw =
            angles::shortest_angular_distance(filtered_prev_yaw, filtered_cmd_yaw);
          const double now_to_next_yaw =
            angles::shortest_angular_distance(filtered_cmd_yaw, filtered_next_yaw);
          ff_yaw_acc = (now_to_next_yaw - prev_to_now_yaw) / (predictive_dt * predictive_dt);
          ff_pitch_acc =
            (filtered_next_pitch - 2.0 * filtered_cmd_pitch + filtered_prev_pitch) /
            (predictive_dt * predictive_dt);

          // 7. 限幅加速度
          ff_yaw_acc = std::clamp(ff_yaw_acc, -max_yaw_acc_, max_yaw_acc_);
          ff_pitch_acc = std::clamp(ff_pitch_acc, -max_pitch_acc_, max_pitch_acc_);
        }

        feedforward_valid = true;
      }
    }
  }

  // 8. 如果目标当前不可控或前馈无效，则保持清零
  if (!feedforward_valid) {
    filtered_cmd_yaw = cmd_yaw;
    filtered_cmd_pitch = cmd_pitch;
    ff_yaw_vel = 0.0;
    ff_pitch_vel = 0.0;
    ff_yaw_acc = 0.0;
    ff_pitch_acc = 0.0;
  }
  // ==================== 前馈输出结束 ====================

  // 回填控制消息（消息层当前仍使用 degree）
  gimbal_cmd.control = true;
  gimbal_cmd.yaw = cmd_yaw * 180 / M_PI;
  gimbal_cmd.pitch = cmd_pitch * 180 / M_PI;
  gimbal_cmd.yaw_vel = ff_yaw_vel * 180 / M_PI;
  gimbal_cmd.pitch_vel = ff_pitch_vel * 180 / M_PI;
  gimbal_cmd.yaw_acc = ff_yaw_acc * 180 / M_PI;
  gimbal_cmd.pitch_acc = ff_pitch_acc * 180 / M_PI;

  // 更新历史
  last_control_.valid = true;
  last_control_.stamp = current_time;
  last_control_.yaw = cmd_yaw;
  last_control_.pitch = cmd_pitch;
  last_control_.filtered_yaw = filtered_cmd_yaw;
  last_control_.filtered_pitch = filtered_cmd_pitch;
  last_control_.yaw_vel = ff_yaw_vel;
  last_control_.pitch_vel = ff_pitch_vel;

  if (gimbal_cmd.fire_advice) {
    FYT_DEBUG("armor_solver", "You Need Fire!");
  }
  return gimbal_cmd;
}

Solver::PredictedCommand Solver::predictCommandAtTime(const rm_interfaces::msg::Target &target,
                                                      double future_time_s,
                                                      bool aim_center) const noexcept {
  PredictedCommand result;

  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  target_position.x() += future_time_s * target.velocity.x;
  target_position.y() += future_time_s * target.velocity.y;
  target_position.z() += future_time_s * target.velocity.z;
  const double target_yaw = target.yaw + future_time_s * target.v_yaw;

  Eigen::Vector3d aim_point = target_position;
  if (!aim_center) {
    auto armor_positions = getArmorPositions(target_position, target_yaw, target.radius_1,
                                             target.radius_2, target.d_zc, target.d_za,
                                             static_cast<size_t>(target.armors_num));
    const int idx = selectBestArmor(armor_positions, target_position, target_yaw, target.v_yaw,
                                    static_cast<size_t>(target.armors_num));
    if (idx < 0 || idx >= static_cast<int>(armor_positions.size())) {
      return result;
    }
    aim_point = armor_positions[static_cast<size_t>(idx)];
  }

  result.distance = aim_point.norm();
  if (result.distance < 0.1) {
    return result;
  }

  double yaw = 0.0;
  double pitch = 0.0;
  calcYawAndPitch(aim_point, rpy_, yaw, pitch);

  auto angle_offset = manual_compensator_->angleHardCorrect(aim_point.head(2).norm(), aim_point.z());
  const double pitch_offset = angle_offset[0] * M_PI / 180.0;
  const double yaw_offset = angle_offset[1] * M_PI / 180.0;

  result.yaw = angles::normalize_angle(yaw + yaw_offset);
  result.pitch = pitch + pitch_offset;
  result.valid = true;
  return result;
}

bool Solver::isOnTarget(const double cur_yaw,
                        const double cur_pitch,
                        const double target_yaw,
                        const double target_pitch,
                        const double distance) const noexcept {
  // Judge whether to shoot
  double shooting_range_yaw = std::abs(atan2(shooting_range_w_ / 2, distance));
  double shooting_range_pitch = std::abs(atan2(shooting_range_h_ / 2, distance));
  // Limit the shooting area to 1 degree to avoid not shooting when distance is
  // too large
  shooting_range_yaw = std::max(shooting_range_yaw, 1.0 * M_PI / 180);
  shooting_range_pitch = std::max(shooting_range_pitch, 1.0 * M_PI / 180);
  if (std::abs(cur_yaw - target_yaw) < shooting_range_yaw &&
      std::abs(cur_pitch - target_pitch) < shooting_range_pitch) {
    return true;
  }

  return false;
}

bool Solver::distance_based_shooter_check(const double cur_yaw,
                                          const double cur_pitch,
                                          const double target_yaw,
                                          const double target_pitch,
                                          const double distance) const noexcept {
  const double tolerance_yaw =
    distance > strict_distance_threshold_ ? far_tolerance_yaw_ : near_tolerance_yaw_;
  const double tolerance_pitch =
    distance > strict_distance_threshold_ ? far_tolerance_pitch_ : near_tolerance_pitch_;

  const double yaw_err = std::abs(angles::shortest_angular_distance(cur_yaw, target_yaw));
  const double pitch_err = std::abs(cur_pitch - target_pitch);

  return yaw_err < tolerance_yaw && pitch_err < tolerance_pitch;
}

std::vector<Eigen::Vector3d> Solver::getArmorPositions(const Eigen::Vector3d &target_center,
                                                       const double target_yaw,
                                                       const double r1,
                                                       const double r2,
                                                       const double d_zc,
                                                       const double d_za,
                                                       const size_t armors_num) const noexcept {
  auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());
  // Calculate the position of each armor
  bool is_current_pair = true;
  double r = 0., target_dz = 0.;
  for (size_t i = 0; i < armors_num; i++) {
    double temp_yaw = target_yaw + i * (2 * M_PI / armors_num);
    if (armors_num == 4) {
      r = is_current_pair ? r1 : r2;
      target_dz = d_zc + (is_current_pair ? 0 : d_za);
      is_current_pair = !is_current_pair;
    } else {
      r = r1;
      target_dz = d_zc;
    }
    armor_positions[i] =
      target_center + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
  }
  return armor_positions;
}

int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                            const Eigen::Vector3d &target_center,
                            const double target_yaw,
                            const double target_v_yaw,
                            const size_t armors_num) const noexcept {
  // Angle between the car's center and the X-axis
  double alpha = std::atan2(target_center.y(), target_center.x());
  // Angle between the front of observed armor and the X-axis
  double beta = target_yaw;

  // clang-format off
  Eigen::Matrix2d R_odom2center;
  Eigen::Matrix2d R_odom2armor;
  R_odom2center << std::cos(alpha), std::sin(alpha), 
                  -std::sin(alpha), std::cos(alpha);
  R_odom2armor << std::cos(beta), std::sin(beta), 
                 -std::sin(beta), std::cos(beta);
  // clang-format on
  Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;

  // Equal to (alpha - beta) in most cases
  double decision_angle = -std::asin(R_center2armor(0, 1));

  // Angle thresh of the armor jump
  double theta = (target_v_yaw > 0 ? side_angle_ : -side_angle_) / 180.0 * M_PI;

  // Avoid the frequent switch between two armor
  if (std::abs(target_v_yaw) < min_switching_v_yaw_) {
    theta = 0;
  }

  double temp_angle = decision_angle + M_PI / armors_num - theta;

  if (temp_angle < 0) {
    temp_angle += 2 * M_PI;
  }

  int selected_id = static_cast<int>(temp_angle / (2 * M_PI / armors_num));
  return selected_id;
}

void Solver::calcYawAndPitch(const Eigen::Vector3d &p,
                             const std::array<double, 3> rpy,
                             double &yaw,
                             double &pitch) const noexcept {
  // Calculate yaw and pitch
  yaw = atan2(p.y(), p.x());
  pitch = atan2(p.z(), p.head(2).norm());

  if (double temp_pitch = pitch; trajectory_compensator_->compensate(p, temp_pitch)) {
    pitch = temp_pitch;
  }
}

std::vector<std::pair<double, double>> Solver::getTrajectory() const noexcept {
  auto trajectory = trajectory_compensator_->getTrajectory(15, rpy_[1]);
  // Rotate
  for (auto &p : trajectory) {
    double x = p.first;
    double y = p.second;
    p.first = x * cos(rpy_[1]) + y * sin(rpy_[1]);
    p.second = -x * sin(rpy_[1]) + y * cos(rpy_[1]);
  }
  return trajectory;
}

}  // namespace fyt::auto_aim
