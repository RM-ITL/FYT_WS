#ifndef ARMOR_SOLVER_PLANNER_HPP_
#define ARMOR_SOLVER_PLANNER_HPP_

#include <functional>
#include <memory>

#include <Eigen/Dense>
#include <angles/angles.h>
#include <rclcpp/rclcpp.hpp>

#include "armor_solver/tinympc/tiny_api.hpp"
#include "rm_interfaces/msg/target.hpp"

namespace fyt::auto_aim {

class TinyMpcPlanner {
public:
  struct ReferenceCommand {
    double yaw = 0.0;
    double pitch = 0.0;
    double distance = -1.0;
    bool valid = false;
  };

  struct Plan {
    bool valid = false;
    bool fire = false;
    double target_yaw = 0.0;
    double target_pitch = 0.0;
    double yaw = 0.0;
    double yaw_vel = 0.0;
    double yaw_acc = 0.0;
    double pitch = 0.0;
    double pitch_vel = 0.0;
    double pitch_acc = 0.0;
    double distance = -1.0;
  };

  using PredictFunc = std::function<ReferenceCommand(double)>;

  explicit TinyMpcPlanner(std::weak_ptr<rclcpp::Node> node);
  ~TinyMpcPlanner() = default;

  Plan plan(const rm_interfaces::msg::Target &target,
            double bullet_speed,
            const PredictFunc &predict_fn) noexcept;

private:
  static constexpr double kDt = 0.01;
  static constexpr int kHalfHorizon = 50;
  static constexpr int kHorizon = kHalfHorizon * 2;

  using Trajectory = Eigen::Matrix<double, 4, kHorizon>;
  struct TrajectoryReference {
    Trajectory traj = Trajectory::Zero();
    double yaw0 = 0.0;
    bool valid = false;
  };

  void setupYawSolver() noexcept;
  void setupPitchSolver() noexcept;
  TrajectoryReference buildTrajectory(double delay_time, const PredictFunc &predict_fn) const noexcept;

  std::weak_ptr<rclcpp::Node> node_;

  double fire_thresh_ = 0.02;
  double decision_speed_ = 3.0;
  double high_speed_delay_time_ = 0.05;
  double low_speed_delay_time_ = 0.02;
  double max_yaw_acc_ = 20.0;
  double max_pitch_acc_ = 50.0;
  Eigen::Vector2d q_yaw_{50.0, 1.0};
  Eigen::Vector2d q_pitch_{50.0, 1.0};
  double r_yaw_ = 1.0;
  double r_pitch_ = 1.0;

  TinySolver *yaw_solver_ = nullptr;
  TinySolver *pitch_solver_ = nullptr;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_PLANNER_HPP_
