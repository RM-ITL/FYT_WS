#include "armor_solver/planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace fyt::auto_aim {

TinyMpcPlanner::TinyMpcPlanner(std::weak_ptr<rclcpp::Node> node) : node_(node) {
  if (auto n = node_.lock()) {
    fire_thresh_ = n->declare_parameter("planner.fire_thresh", 0.02);
    decision_speed_ = n->declare_parameter("planner.decision_speed", 3.0);
    high_speed_delay_time_ = n->declare_parameter("planner.high_speed_delay_time", 0.05);
    low_speed_delay_time_ = n->declare_parameter("planner.low_speed_delay_time", 0.02);
    max_yaw_acc_ = n->declare_parameter("planner.max_yaw_acc", 20.0);
    max_pitch_acc_ = n->declare_parameter("planner.max_pitch_acc", 50.0);

    const auto q_yaw = n->declare_parameter("planner.q_yaw", std::vector<double>{50.0, 1.0});
    const auto q_pitch =
        n->declare_parameter("planner.q_pitch", std::vector<double>{50.0, 1.0});
    const auto r_yaw = n->declare_parameter("planner.r_yaw", std::vector<double>{1.0});
    const auto r_pitch = n->declare_parameter("planner.r_pitch", std::vector<double>{1.0});

    if (q_yaw.size() >= 2) {
      q_yaw_ << q_yaw[0], q_yaw[1];
    }
    if (q_pitch.size() >= 2) {
      q_pitch_ << q_pitch[0], q_pitch[1];
    }
    if (!r_yaw.empty()) {
      r_yaw_ = r_yaw.front();
    }
    if (!r_pitch.empty()) {
      r_pitch_ = r_pitch.front();
    }
  }

  setupYawSolver();
  setupPitchSolver();
}

void TinyMpcPlanner::setupYawSolver() noexcept {
  Eigen::MatrixXd A{{1.0, kDt}, {0.0, 1.0}};
  Eigen::MatrixXd B{{0.0}, {kDt}};
  Eigen::VectorXd f{{0.0, 0.0}};
  Eigen::Vector2d Q = q_yaw_;
  Eigen::Matrix<double, 1, 1> R;
  R << r_yaw_;

  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, kHorizon, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, kHorizon, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, kHorizon, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, kHorizon - 1, -max_yaw_acc_);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, kHorizon - 1, max_yaw_acc_);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);
  yaw_solver_->settings->max_iter = 10;
}

void TinyMpcPlanner::setupPitchSolver() noexcept {
  Eigen::MatrixXd A{{1.0, kDt}, {0.0, 1.0}};
  Eigen::MatrixXd B{{0.0}, {kDt}};
  Eigen::VectorXd f{{0.0, 0.0}};
  Eigen::Vector2d Q = q_pitch_;
  Eigen::Matrix<double, 1, 1> R;
  R << r_pitch_;

  tiny_setup(
      &pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, kHorizon, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, kHorizon, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, kHorizon, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, kHorizon - 1, -max_pitch_acc_);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, kHorizon - 1, max_pitch_acc_);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);
  pitch_solver_->settings->max_iter = 10;
}

TinyMpcPlanner::TrajectoryReference TinyMpcPlanner::buildTrajectory(
    double delay_time, const PredictFunc &predict_fn) const noexcept {
  TrajectoryReference result;

  const auto sample = [&predict_fn](double t) {
    return predict_fn(t);
  };

  auto yaw_pitch_last = sample(delay_time - (kHalfHorizon + 1) * kDt);
  auto yaw_pitch = sample(delay_time - kHalfHorizon * kDt);
  if (!yaw_pitch.valid) {
    return result;
  }

  result.yaw0 = yaw_pitch.yaw;

  for (int i = 0; i < kHorizon; ++i) {
    auto yaw_pitch_next = sample(delay_time + (i - kHalfHorizon + 1) * kDt);
    if (!yaw_pitch_next.valid || !yaw_pitch.valid || !yaw_pitch_last.valid) {
      return TrajectoryReference{};
    }

    const double yaw_vel =
        angles::shortest_angular_distance(yaw_pitch_last.yaw, yaw_pitch_next.yaw) / (2.0 * kDt);
    const double pitch_vel = (yaw_pitch_next.pitch - yaw_pitch_last.pitch) / (2.0 * kDt);

    result.traj.col(i) << angles::shortest_angular_distance(result.yaw0, yaw_pitch.yaw), yaw_vel,
        yaw_pitch.pitch, pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  result.valid = true;
  return result;
}

TinyMpcPlanner::Plan TinyMpcPlanner::plan(const rm_interfaces::msg::Target &target,
                                          double bullet_speed,
                                          const PredictFunc &predict_fn) noexcept {
  (void)bullet_speed;
  Plan plan;

  if (!yaw_solver_ || !pitch_solver_) {
    return plan;
  }

  const double delay_time =
      std::abs(target.v_yaw) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  const auto center_cmd = predict_fn(delay_time);
  if (!center_cmd.valid) {
    return plan;
  }

  const auto trajectory_ref = buildTrajectory(delay_time, predict_fn);
  if (!trajectory_ref.valid) {
    return plan;
  }
  const Trajectory &traj = trajectory_ref.traj;
  const double yaw0 = trajectory_ref.yaw0;

  Eigen::VectorXd yaw_x0(2);
  yaw_x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, yaw_x0);
  yaw_solver_->work->Xref = traj.block(0, 0, 2, kHorizon);
  tiny_solve(yaw_solver_);

  Eigen::VectorXd pitch_x0(2);
  pitch_x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, pitch_x0);
  pitch_solver_->work->Xref = traj.block(2, 0, 2, kHorizon);
  tiny_solve(pitch_solver_);

  plan.valid = true;
  plan.distance = center_cmd.distance;
  plan.target_yaw = angles::normalize_angle(traj(0, kHalfHorizon) + yaw0);
  plan.target_pitch = traj(2, kHalfHorizon);
  plan.yaw = angles::normalize_angle(yaw_solver_->work->x(0, kHalfHorizon) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, kHalfHorizon);
  plan.yaw_acc = yaw_solver_->work->u(0, kHalfHorizon);
  plan.pitch = pitch_solver_->work->x(0, kHalfHorizon);
  plan.pitch_vel = pitch_solver_->work->x(1, kHalfHorizon);
  plan.pitch_acc = pitch_solver_->work->u(0, kHalfHorizon);

  const int shoot_offset = 2;
  const int index = std::min(kHalfHorizon + shoot_offset, kHorizon - 1);
  plan.fire = std::hypot(
                  traj(0, index) - yaw_solver_->work->x(0, index),
                  traj(2, index) - pitch_solver_->work->x(0, index)) < fire_thresh_;
  return plan;
}

}  // namespace fyt::auto_aim
