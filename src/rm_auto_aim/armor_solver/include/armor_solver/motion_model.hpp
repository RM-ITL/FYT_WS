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

#ifndef ARMOR_SOLVER_MOTION_MODEL_HPP_
#define ARMOR_SOLVER_MOTION_MODEL_HPP_

// ceres
#include <ceres/ceres.h>
#include <cmath>
// project
#include "rm_utils/math/extended_kalman_filter.hpp"

namespace fyt::auto_aim {

enum class MotionModel {
  CONSTANT_VELOCITY = 0,  // Constant velocity
  CONSTANT_ROTATION = 1,  // Constant rotation velocity
  CONSTANT_VEL_ROT = 2    // Constant velocity and rotation velocity
};

// X_N: state dimension, Z_N: measurement dimension
constexpr int X_N = 10, Z_N = 4;

struct Predict {
  explicit Predict(double dt, MotionModel model = MotionModel::CONSTANT_VEL_ROT)
  : dt(dt), model(model) {}

  template <typename T>
  void operator()(const T x0[X_N], T x1[X_N]) {
    for (int i = 0; i < X_N; i++) {
      x1[i] = x0[i];
    }

    // v_xyz
    if (model == MotionModel::CONSTANT_VEL_ROT || model == MotionModel::CONSTANT_VELOCITY) {
      // linear velocity
      x1[0] += x0[1] * dt;
      x1[2] += x0[3] * dt;
      x1[4] += x0[5] * dt;
    } else {
      // no velocity
      x1[1] *= 0.;
      x1[3] *= 0.;
      x1[5] *= 0.;
    }

    // v_yaw
    if (model == MotionModel::CONSTANT_VEL_ROT || model == MotionModel::CONSTANT_ROTATION) {
      // angular velocity
      x1[6] += x0[7] * dt;
    } else {
      // no rotation
      x1[7] *= 0.;
    }
  }

  double dt;
  MotionModel model;
};

struct Measure {
  int armor_id = 0;
  std::size_t armors_num = 4;
  double another_r = 0.26;
  double d_za = 0.0;

  template <typename T>
  void operator()(const T x[X_N], T z[Z_N]) {
    const T armors_num_t = T(static_cast<double>(armors_num));
    const T temp_yaw = x[6] + T(static_cast<double>(armor_id)) * T(2.0 * M_PI) / armors_num_t;
    const bool use_secondary = armors_num == 4 && (armor_id % 2 == 1);
    const T radius = use_secondary ? T(another_r) : x[8];
    const T z_offset = x[9] + (use_secondary ? T(d_za) : T(0.0));

    const T armor_x = x[0] - ceres::cos(temp_yaw) * radius;
    const T armor_y = x[2] - ceres::sin(temp_yaw) * radius;
    const T armor_z = x[4] + z_offset;
    const T horizontal_dist = ceres::sqrt(armor_x * armor_x + armor_y * armor_y);

    z[0] = ceres::atan2(armor_y, armor_x);
    z[1] = ceres::atan2(armor_z, horizontal_dist);
    z[2] = ceres::sqrt(horizontal_dist * horizontal_dist + armor_z * armor_z);
    z[3] = temp_yaw;
  }
};

using RobotStateEKF = ExtendedKalmanFilter<X_N, Z_N, Predict, Measure>;

}  // namespace fyt::auto_aim
#endif
