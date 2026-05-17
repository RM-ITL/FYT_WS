// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
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

#include "armor_solver/armor_tracker.hpp"
// std
#include <algorithm>
#include <cfloat>
#include <memory>
#include <string>
// ros2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// third party
#include <angles/angles.h>
// project
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {
namespace {
Eigen::Vector3d xyzToYpd(const Eigen::Vector3d &xyz) {
  const double yaw = std::atan2(xyz.y(), xyz.x());
  const double horizontal = std::hypot(xyz.x(), xyz.y());
  const double pitch = std::atan2(xyz.z(), horizontal);
  return Eigen::Vector3d(yaw, pitch, xyz.norm());
}
}

Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
: tracker_state(LOST)
, tracked_id(std::string(""))
, measurement(Eigen::VectorXd::Zero(Z_N))
, target_state(Eigen::VectorXd::Zero(X_N))
, max_match_distance_(max_match_distance)
, max_match_yaw_diff_(max_match_yaw_diff)
, detect_count_(0)
, lost_count_(0)
, no_switch_count_(0)
, single_plate_mode_(false)
, last_yaw_(0)
, last_update_time_(std::chrono::steady_clock::now()) {}

void Tracker::init(const Armors::SharedPtr &armors_msg) noexcept {
  if (armors_msg->armors.empty()) {
    return;
  }

  tracked_armor = *std::min_element(
      armors_msg->armors.begin(), armors_msg->armors.end(),
      [](const Armor &a, const Armor &b) {
        const int rank_a = armorRank(a);
        const int rank_b = armorRank(b);
        if (rank_a != rank_b) {
          return rank_a < rank_b;
        }
        if (std::abs(a.distance_to_image_center - b.distance_to_image_center) > 1e-3) {
          return a.distance_to_image_center < b.distance_to_image_center;
        }
        return a.confidence > b.confidence;
      });

  initEKF(tracked_armor);
  FYT_INFO("armor_solver", "Init EKF!");

  tracked_id = tracked_armor.number;
  tracker_state = DETECTING;
  detect_count_ = 0;
  lost_count_ = 0;
  no_switch_count_ = 0;
  single_plate_mode_ = false;
  last_update_time_ = std::chrono::steady_clock::now();
  updateTrackedArmorType();
}

void Tracker::update(const Armors::SharedPtr &armors_msg) noexcept {
  const auto now = std::chrono::steady_clock::now();
  const double dt = std::chrono::duration<double>(now - last_update_time_).count();
  last_update_time_ = now;
  if (tracker_state != LOST && dt > timeout_sec) {
    FYT_WARN("armor_solver", "Tracker reset due to stale input: {:.3f}s", dt);
    resetTracking();
    return;
  }

  // KF predict
  Eigen::VectorXd ekf_prediction = ekf->predict();

  bool matched = false;
  // Use KF prediction as default target state if no matched armor is found
  target_state = ekf_prediction;

  if (!armors_msg->armors.empty()) {
    // Find the closest armor with the same id
    Armor same_id_armor;
    int same_id_armors_count = 0;
    auto predicted_position = getArmorPositionFromState(ekf_prediction);
    double min_position_diff = DBL_MAX;
    double yaw_diff = DBL_MAX;
    double best_cost = DBL_MAX;
    for (const auto &armor : armors_msg->armors) {
      // Only consider armors with the same id
      if (armor.number == tracked_id) {
        same_id_armor = armor;
        same_id_armors_count++;
        // Calculate the difference between the predicted position and the
        // current armor position
        auto p = armor.pose.position;
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        double position_diff = (predicted_position - position_vec).norm();
        const double measured_yaw = orientationToYaw(armor.pose.orientation);
        const double current_yaw_diff =
            std::abs(angles::shortest_angular_distance(ekf_prediction(6), measured_yaw));
        const double cost = position_diff + current_yaw_diff * 0.05 +
                            static_cast<double>(armorRank(armor)) * 0.01 +
                            static_cast<double>(armor.distance_to_image_center) * 1e-4;
        if (cost < best_cost) {
          // Find the closest armor
          best_cost = cost;
          min_position_diff = position_diff;
          yaw_diff = current_yaw_diff;
          tracked_armor = armor;
        }
      }
    }

    // Check if the distance and yaw difference of closest armor are within the
    // threshold
    if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
      // Matched armor found
      matched = true;
      // Update EKF
      const int armor_id = estimateArmorIndex(ekf_prediction, tracked_armor);
      ekf->setMeasureFunc(
          Measure{armor_id, static_cast<std::size_t>(tracked_armors_num), another_r, d_za});
      ekf->setResidualFunc([](const Eigen::Matrix<double, Z_N, 1> &z,
                              const Eigen::Matrix<double, Z_N, 1> &z_pri) {
        Eigen::Matrix<double, Z_N, 1> residual = z - z_pri;
        residual(0) = angles::shortest_angular_distance(z_pri(0), z(0));
        residual(1) = angles::shortest_angular_distance(z_pri(1), z(1));
        residual(3) = angles::shortest_angular_distance(z_pri(3), z(3));
        return residual;
      });
      measurement = buildMeasurement(tracked_armor);
      target_state = ekf->update(measurement);
      updateTrackedArmorType();
      no_switch_count_++;
      if (no_switch_count_ > single_plate_threshold &&
          std::abs(target_state(7)) < omega_threshold) {
        single_plate_mode_ = true;
      }
    } else if (!single_plate_mode_ && same_id_armors_count == 1 &&
               yaw_diff > max_match_yaw_diff_) {
      // Matched armor not found, but there is only one armor with the same id
      // and yaw has jumped, take this case as the target is spinning and armor
      // jumped
      handleArmorJump(same_id_armor);
      tracked_armor = same_id_armor;
      const int armor_id = estimateArmorIndex(target_state, tracked_armor);
      ekf->setMeasureFunc(
          Measure{armor_id, static_cast<std::size_t>(tracked_armors_num), another_r, d_za});
      measurement = buildMeasurement(tracked_armor);
      target_state = ekf->update(measurement);
      matched = true;
      no_switch_count_ = 0;
      single_plate_mode_ = false;
    } else if (same_id_armors_count == 0 && armors_msg->armors.size() == 1) {
      const auto &fallback_armor = armors_msg->armors.front();
      auto p = fallback_armor.pose.position;
      Eigen::Vector3d current_p(p.x, p.y, p.z);
      const double fallback_position_diff = (predicted_position - current_p).norm();
      const double fallback_yaw_diff = std::abs(angles::shortest_angular_distance(
          ekf_prediction(6), orientationToYaw(fallback_armor.pose.orientation)));
      if (fallback_position_diff < max_match_distance_ * 0.8 &&
          fallback_yaw_diff < max_match_yaw_diff_ * 1.2) {
        tracked_armor = fallback_armor;
        tracked_id = tracked_armor.number;
        updateTrackedArmorType();
        const int armor_id = estimateArmorIndex(target_state, tracked_armor);
        ekf->setMeasureFunc(
            Measure{armor_id, static_cast<std::size_t>(tracked_armors_num), another_r, d_za});
        measurement = buildMeasurement(tracked_armor);
        target_state = ekf->update(measurement);
        matched = true;
        no_switch_count_ = 0;
        single_plate_mode_ = false;
      }
    } else {
      // No matched armor found
      FYT_WARN("armor_solver", "No matched armor found!");
      no_switch_count_ = 0;
      single_plate_mode_ = false;
    }
  }

  // Prevent radius from spreading
  if (target_state(8) < 0.12) {
    target_state(8) = 0.12;
    ekf->setState(target_state);
  } else if (target_state(8) > 0.4) {
    target_state(8) = 0.4;
    ekf->setState(target_state);
  }

  if (isStateDiverged()) {
    FYT_WARN("armor_solver", "Tracker reset because target state diverged");
    resetTracking();
    return;
  }

  // Tracking state machine
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
        FYT_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
      FYT_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
      FYT_DEBUG("armor_solver", "Tracker state: TEMP_LOST {}", tracked_id);
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_thres) {
        lost_count_ = 0;
        tracker_state = LOST;
        FYT_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
      FYT_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
    }
  }
}

void Tracker::initEKF(const Armor &a) noexcept {
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  last_yaw_ = 0;
  double yaw = orientationToYaw(a.pose.orientation);

  // Set initial position at 0.2m behind the target
  target_state = Eigen::VectorXd::Zero(X_N);
  double r = 0.26;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  double zc = za;
  d_za = 0, d_zc = 0, another_r = r;
  target_state << xc, 0, yc, 0, zc, 0, yaw, 0, r, d_zc;

  ekf->setState(target_state);
}

void Tracker::handleArmorJump(const Armor &current_armor) noexcept {
  double last_yaw = target_state(6);
  double yaw = orientationToYaw(current_armor.pose.orientation);

  if (abs(yaw - last_yaw) > 0.4) {
    // Armor angle also jumped, take this case as target spinning
    target_state(6) = yaw;
    // Only 4 armors has 2 radius and height
    if (tracked_armors_num == ArmorsNum::NORMAL_4) {
      d_za = target_state(4) + target_state(9) - current_armor.pose.position.z;
      std::swap(target_state(8), another_r);
      d_zc = d_zc == 0 ? -d_za : 0;
      target_state(9) = d_zc;
    }
    FYT_DEBUG("armor_solver", "Armor Jump!");
  }

  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

  if ((current_p - infer_p).norm() > max_match_distance_) {
    // If the distance between the current armor and the inferred armor is too
    // large, the state is wrong, reset center position and velocity in the
    // state
    d_zc = 0;
    double r = target_state(8);
    target_state(0) = p.x + r * cos(yaw);  // xc
    target_state(1) = 0;                   // vxc
    target_state(2) = p.y + r * sin(yaw);  // yc
    target_state(3) = 0;                   // vyc
    target_state(4) = p.z;                 // zc
    target_state(5) = 0;                   // vzc
    target_state(9) = d_zc;                // d_zc
    FYT_WARN("armor_solver", "State wrong!");
  }

  ekf->setState(target_state);
}

double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion &q) noexcept {
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous (-pi~pi to -inf~inf)
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

void Tracker::updateTrackedArmorType() noexcept {
  if (tracked_armor.type == "large" &&
      (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
    tracked_armors_num = ArmorsNum::BALANCE_2;
  } else if (tracked_id == "outpost") {
    tracked_armors_num = ArmorsNum::OUTPOST_3;
  } else {
    tracked_armors_num = ArmorsNum::NORMAL_4;
  }
}

int Tracker::armorRank(const Armor &armor) noexcept {
  if (armor.rank > 0) {
    return armor.rank;
  }
  if (armor.number == "3" || armor.number == "4") {
    return 1;
  }
  if (armor.number == "sentry") {
    return 2;
  }
  if (armor.number == "1" || armor.number == "5") {
    return 3;
  }
  if (armor.number == "2") {
    return 4;
  }
  return 5;
}

bool Tracker::isStateDiverged() const noexcept {
  if (!target_state.allFinite()) {
    return true;
  }
  const Eigen::Vector3d center(target_state(0), target_state(2), target_state(4));
  if (center.norm() > max_position_norm) {
    return true;
  }
  if (std::abs(target_state(7)) > max_abs_v_yaw) {
    return true;
  }
  return false;
}

void Tracker::resetTracking() noexcept {
  tracker_state = LOST;
  tracked_id.clear();
  detect_count_ = 0;
  lost_count_ = 0;
  no_switch_count_ = 0;
  single_plate_mode_ = false;
}

int Tracker::estimateArmorIndex(const Eigen::VectorXd &state, const Armor &armor) const noexcept {
  const std::size_t armors_num = static_cast<std::size_t>(tracked_armors_num);
  std::vector<Eigen::Vector3d> candidates;
  candidates.reserve(armors_num);

  bool is_current_pair = true;
  for (std::size_t i = 0; i < armors_num; ++i) {
    double temp_yaw = state(6) + static_cast<double>(i) * (2.0 * M_PI / armors_num);
    double r = state(8);
    double dz = state(9);
    if (armors_num == 4) {
      r = is_current_pair ? state(8) : another_r;
      dz = state(9) + (is_current_pair ? 0.0 : d_za);
      is_current_pair = !is_current_pair;
    }
    candidates.emplace_back(
        state(0) - r * std::cos(temp_yaw), state(2) - r * std::sin(temp_yaw), state(4) + dz);
  }

  const Eigen::Vector3d observed(
      armor.pose.position.x, armor.pose.position.y, armor.pose.position.z);
  double min_error = DBL_MAX;
  int best_id = 0;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const double error = (candidates[i] - observed).norm();
    if (error < min_error) {
      min_error = error;
      best_id = static_cast<int>(i);
    }
  }
  return best_id;
}

Eigen::Matrix<double, Z_N, 1> Tracker::buildMeasurement(const Armor &armor) noexcept {
  const Eigen::Vector3d xyz(
      armor.pose.position.x, armor.pose.position.y, armor.pose.position.z);
  const Eigen::Vector3d ypd = xyzToYpd(xyz);
  Eigen::Matrix<double, Z_N, 1> z;
  z << ypd(0), ypd(1), ypd(2), orientationToYaw(armor.pose.orientation);
  return z;
}

Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd &x) noexcept {
  // Calculate predicted position of the current armor
  double xc = x(0), yc = x(2), za = x(4) + x(9);
  double yaw = x(6), r = x(8);
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, za);
}

}  // namespace fyt::auto_aim
