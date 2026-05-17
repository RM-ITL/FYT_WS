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

#ifndef ARMOR_SOLVER_SOLVER_HPP_
#define ARMOR_SOLVER_SOLVER_HPP_

// std
#include <memory>
// ros2
#include <tf2_ros/buffer.h>
#include <angles/angles.h>

#include <rclcpp/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// 3rd party
#include <Eigen/Dense>
// project
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/gimbal_state.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "armor_solver/planner.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"
#include "rm_utils/math/manual_compensator.hpp"

namespace fyt::auto_aim {
// Solver class used to solve the gimbal command from tracked target
class Solver {
public:
  struct PredictedCommand {
    double yaw = 0.0;
    double pitch = 0.0;
    double distance = -1.0;
    bool valid = false;
  };

  void updateRuntimeState(const rm_interfaces::msg::GimbalState &state);
  explicit Solver(std::weak_ptr<rclcpp::Node> node);
  // explicit Solver(std::string trajectory_compensator_type, float max_tracking_v_yaw);
  ~Solver() = default;

  // Solve the gimbal command from tracked target
  // Throw: tf2::TransformException if the transform from "odom" to "gimbal_link" is not available
  rm_interfaces::msg::GimbalCmd solve(const rm_interfaces::msg::Target &target_msg,
                                      const rclcpp::Time &current_time,
                                      std::shared_ptr<tf2_ros::Buffer> tf2_buffer_);

  enum State { TRACKING_ARMOR = 0, TRACKING_CENTER = 1 } state;

  std::vector<std::pair<double, double>> getTrajectory() const noexcept; 

private:
    rm_interfaces::msg::GimbalState runtime_state_;
  // Get the armor positions from the target robot
  std::vector<Eigen::Vector3d> getArmorPositions(const Eigen::Vector3d &target_center,
                                                 const double yaw,
                                                 const double r1,
                                                 const double r2,
                                                 const double d_zc,
                                                 const double d_za,
                                                 const size_t armors_num) const noexcept;

  // Select the best armor to shoot
  // Return: selected idx in {0, 1, ..., armors_num - 1}
  int selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                      const Eigen::Vector3d &target_center,
                      const double target_yaw,
                      const double target_v_yaw,
                      const size_t armors_num) const noexcept;

  void calcYawAndPitch(const Eigen::Vector3d &p,
                       const std::array<double, 3> rpy,
                       double &yaw,
                       double &pitch) const noexcept;

  PredictedCommand predictCommandAtTime(const rm_interfaces::msg::Target &target,
                                        double future_time_s,
                                        bool aim_center) const noexcept;

  bool has_runtime_state_ = false;
  
  bool isOnTarget(const double cur_yaw,
                  const double cur_pitch,
                  const double target_yaw,
                  const double target_pitch,
                  const double distance) const noexcept;

  bool distance_based_shooter_check(const double cur_yaw,
                                    const double cur_pitch,
                                    const double target_yaw,
                                    const double target_pitch,
                                    const double distance) const noexcept;

  std::unique_ptr<TrajectoryCompensator> trajectory_compensator_;
  std::unique_ptr<ManualCompensator> manual_compensator_;
  std::unique_ptr<TinyMpcPlanner> planner_;

  std::array<double, 3> rpy_;

  double default_bullet_speed_;

  struct ControlHistory {
    bool valid = false;
    rclcpp::Time stamp;
    double yaw = 0.0;            // raw cmd yaw, rad
    double pitch = 0.0;          // raw cmd pitch, rad
    double filtered_yaw = 0.0;   // low-pass filtered yaw, rad
    double filtered_pitch = 0.0; // low-pass filtered pitch, rad
    double yaw_vel = 0.0;        // rad/s
    double pitch_vel = 0.0;      // rad/s
  };

  ControlHistory last_control_;

  double feedforward_alpha_;
  bool enable_acceleration_feedforward_;
  double predictive_ff_dt_;
  double max_yaw_vel_;
  double max_pitch_vel_;
  double max_yaw_acc_;
  double max_pitch_acc_;
  double max_delta_yaw_for_feedforward_;
  double max_delta_pitch_for_feedforward_;

  double near_tolerance_yaw_;
  double near_tolerance_pitch_;
  double far_tolerance_yaw_;
  double far_tolerance_pitch_;
  double strict_distance_threshold_;

  double prediction_delay_;
  double controller_delay_;

  double shooting_range_w_;
  double shooting_range_h_;

  double max_tracking_v_yaw_;
  int overflow_count_;
  int transfer_thresh_;

  double side_angle_;
  double min_switching_v_yaw_;

  std::weak_ptr<rclcpp::Node> node_;
};
}  // namespace fyt::auto_aim
#endif  // ARMOR_SOLVER_SOLVER_HPP_
