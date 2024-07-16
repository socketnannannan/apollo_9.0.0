/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "modules/planning/planning_base/common/trajectory_stitcher.h"

#include <algorithm>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/angle.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_model/vehicle_model.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::VehicleModel;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;

TrajectoryPoint TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
    const double planning_cycle_time, const VehicleState& vehicle_state) {
  TrajectoryPoint point;
  point.mutable_path_point()->set_s(0.0);
  point.mutable_path_point()->set_x(vehicle_state.x());
  point.mutable_path_point()->set_y(vehicle_state.y());
  point.mutable_path_point()->set_z(vehicle_state.z());
  point.mutable_path_point()->set_theta(vehicle_state.heading());
  point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  point.set_v(vehicle_state.linear_velocity());
  point.set_a(vehicle_state.linear_acceleration());
  point.set_relative_time(planning_cycle_time);
  return point;
}
// 在特定情况下重新初始化拼接轨迹。当需要重新规划轨迹时，该函数会生成一个新的轨迹点。
std::vector<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const double planning_cycle_time, const VehicleState& vehicle_state) {
  TrajectoryPoint reinit_point;
  static constexpr double kEpsilon_v = 0.1;
  static constexpr double kEpsilon_a = 0.4;
  // TODO(Jinyun/Yu): adjust kEpsilon if corrected IMU acceleration provided
  // 检查车辆的线速度和加速度是否接近零：
  // 如果速度和加速度都接近零，则根据当前车辆状态计算轨迹点。
  // 否则，预测车辆状态并计算轨迹点。
  if (std::abs(vehicle_state.linear_velocity()) < kEpsilon_v &&
      std::abs(vehicle_state.linear_acceleration()) < kEpsilon_a) {
    reinit_point = ComputeTrajectoryPointFromVehicleState(planning_cycle_time,
                                                          vehicle_state);
  } else {
    VehicleState predicted_vehicle_state;
    predicted_vehicle_state =
        VehicleModel::Predict(planning_cycle_time, vehicle_state);
    reinit_point = ComputeTrajectoryPointFromVehicleState(
        planning_cycle_time, predicted_vehicle_state);
  }

  return std::vector<TrajectoryPoint>(1, reinit_point);
}

// only used in navigation mode
void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double x_diff, const double y_diff, const double theta_diff,
    PublishableTrajectory* prev_trajectory) {
  if (!prev_trajectory) {
    return;
  }

  // R^-1
  double cos_theta = std::cos(theta_diff);
  double sin_theta = -std::sin(theta_diff);

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

  std::for_each(prev_trajectory->begin(), prev_trajectory->end(),
                [&cos_theta, &sin_theta, &tx, &ty,
                 &theta_diff](common::TrajectoryPoint& p) {
                  auto x = p.path_point().x();
                  auto y = p.path_point().y();
                  auto theta = p.path_point().theta();

                  auto x_new = cos_theta * x - sin_theta * y + tx;
                  auto y_new = sin_theta * x + cos_theta * y + ty;
                  auto theta_new =
                      common::math::NormalizeAngle(theta - theta_diff);

                  p.mutable_path_point()->set_x(x_new);
                  p.mutable_path_point()->set_y(y_new);
                  p.mutable_path_point()->set_theta(theta_new);
                });
}

/* Planning from current vehicle state if:
   1. the auto-driving mode is off
   (or) 2. we don't have the trajectory from last planning cycle
   (or) 3. the position deviation from actual and target is too high
*/
/* 
方案1：
根据ADC（Autonomous Driving Car）实际位置状态信息或根据ADC实际位置状态信息，通过车辆运动学推导0.1s后的位置状态信息，
作为规划起始点状态，在Apollo中，这一方案叫做重规划（Replan），但是它属于轨迹拼接的一种，只是拼接点集只有一个点而已。
方案2：
根据当前帧时间戳以及ADC实际位置信息，在上一帧轨迹中寻找匹配点，将上一帧轨迹中匹配点往后0.1s所对应的轨迹点作为当前帧的规划起始状态点，
待当前帧规划生成轨迹后，再和上一帧轨迹中所选择的规划起始点前一段距离的轨迹点进行拼接，形成一条完整的车辆运动轨迹，发送给下游控制模块。
严格来说，以上两种方案都属于轨迹拼接，区别是方案1拼接轨迹只有一个点，而方案2的拼接轨迹是上一帧的部分有序点集，但是在Apollo代码中，
称第一种方案叫做重新初始化拼接点（也叫重规划）。
https://blog.csdn.net/qq_38422317/article/details/129349174?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522172111539416800226514232%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=172111539416800226514232&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-129349174-null-null.142^v100^pc_search_result_base1&utm_term=%E8%BD%A8%E8%BF%B9%E6%8B%BC%E6%8E%A5&spm=1018.2226.3001.4187
在以下情况下，根据当前车辆状态进行规划：
   1.自动驾驶模式关闭
   （或）2. 我们没有上一个计划周期的轨迹
   （或）3.位置与实际值和目标值的偏差过大
*/
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const canbus::Chassis& vehicle_chassis, const VehicleState& vehicle_state,
    const double current_timestamp, const double planning_cycle_time,
    const size_t preserved_points_num, const bool replan_by_offset,
    const PublishableTrajectory* prev_trajectory, std::string* replan_reason) {
  if (!FLAGS_enable_trajectory_stitcher) {
    *replan_reason = "stitch is disabled by gflag.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  if (!prev_trajectory) {
    *replan_reason = "replan for no previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    *replan_reason = "replan for manual mode.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    *replan_reason = "replan for empty previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  // 检查当前时间是否在上一个轨迹的时间范围内：如果当前时间在上一个轨迹的时间范围外，则返回重新初始化的拼接轨迹。
  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();

  size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);
  // 检查时间匹配点是否存在路径点：如果时间匹配点没有路径点，则返回重新初始化的拼接轨迹。
  if (time_matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    AWARN << "current time smaller than the previous trajectory's first time";
    *replan_reason =
        "replan for current time smaller than the previous trajectory's first "
        "time.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  if (time_matched_index + 1 >= prev_trajectory_size) {
    AWARN << "current time beyond the previous trajectory's last time";
    *replan_reason =
        "replan for current time beyond the previous trajectory's last time";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  auto time_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(time_matched_index));

  if (!time_matched_point.has_path_point()) {
    *replan_reason = "replan for previous trajectory missed path point";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  // 查询位置匹配点和检查偏差 如果根据偏差重新规划，检查纵向、横向和时间偏差是否超过阈值。如果超过阈值，则返回重新初始化的拼接轨迹。
  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
      {vehicle_state.x(), vehicle_state.y()}, 1.0e-6);

  auto frenet_sd = ComputePositionProjection(
      vehicle_state.x(), vehicle_state.y(),
      prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(position_matched_index)));

  if (replan_by_offset) {
    if (vehicle_chassis.has_parking_brake()) {
      static bool parking_brake = true;
      if (parking_brake && !vehicle_chassis.parking_brake()) {
        parking_brake = vehicle_chassis.parking_brake();
        const std::string msg =
            "parking brake off, ego move, replan to avoid large station error";
        AERROR << msg;
        *replan_reason = msg;
        return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                vehicle_state);
      }
      parking_brake = vehicle_chassis.parking_brake();
    }

    auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first;
    auto lat_diff = frenet_sd.second;
    double time_diff =
        time_matched_point.relative_time() -
        prev_trajectory
            ->TrajectoryPointAt(static_cast<uint32_t>(position_matched_index))
            .relative_time();

    ADEBUG << "Control lateral diff: " << lat_diff
           << ", longitudinal diff: " << lon_diff
           << ", time diff: " << time_diff;

    if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold) {
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lat_diff = ",
          lat_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }

    if (std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lon_diff = ",
          lon_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }

    if (std::fabs(time_diff) > FLAGS_replan_time_threshold) {
      const std::string msg = absl::StrCat(
          "the difference between time matched point relative time and "
          "actual position corresponding relative time is too "
          "large. Replan is triggered. time_diff = ",
          time_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
  } else {
    ADEBUG << "replan according to certain amount of "
           << "lat、lon and time offset is disabled";
  }
  // 当前时刻的绝对时间-上一时刻轨迹的初始时间+planning_cycle_time
  // 计算前向时间匹配索引，获取时间和位置匹配索引的最小值。
  double forward_rel_time = veh_rel_time + planning_cycle_time;

  size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  ADEBUG << "Position matched index:\t" << position_matched_index;
  ADEBUG << "Time matched index:\t" << time_matched_index;
  // 选取时间匹配索引和位置匹配索引中的较小索引作为匹配点索引值matched_index
  auto matched_index = std::min(time_matched_index, position_matched_index);

  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->begin() +
          std::max(0, static_cast<int>(matched_index - preserved_points_num)),
      prev_trajectory->begin() + forward_time_index + 1);
  ADEBUG << "stitching_trajectory size: " << stitching_trajectory.size();
  // 生成拼接轨迹：
  const double zero_s = stitching_trajectory.back().path_point().s();
  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      *replan_reason = "replan for previous trajectory missed path point";
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }
  return stitching_trajectory;
}

std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
    const double x, const double y, const TrajectoryPoint& p) {
  Vec2d v(x - p.path_point().x(), y - p.path_point().y());
  Vec2d n(std::cos(p.path_point().theta()), std::sin(p.path_point().theta()));

  std::pair<double, double> frenet_sd;
  frenet_sd.first = v.InnerProd(n) + p.path_point().s();
  frenet_sd.second = v.CrossProd(n);
  return frenet_sd;
}

}  // namespace planning
}  // namespace apollo
