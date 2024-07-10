/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/scenarios/bare_intersection_unprotected/stage_approach.h"

#include <vector>

#include "cyber/common/log.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::hdmap::PathOverlap;

StageResult BareIntersectionUnprotectedStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {  // 规划初始点planning_init_point
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(
      GetContextAs<BareIntersectionUnprotectedContext>()->scenario_config);

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);  //按照task列表依次规划
  if (result.HasError()) {
    AERROR << "BareIntersectionUnprotectedStageApproach planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  const std::string pnc_junction_overlap_id =
      GetContextAs<BareIntersectionUnprotectedContext>()
          ->current_pnc_junction_overlap_id;
  if (pnc_junction_overlap_id.empty()) {  // 检查路口是否存在
    return FinishScenario();
  }

  // get overlap along reference line
  PathOverlap* current_pnc_junction =  // 检查路口是否存在
      reference_line_info.GetOverlapOnReferenceLine(
          pnc_junction_overlap_id, ReferenceLineInfo::PNC_JUNCTION);
  if (!current_pnc_junction) {
    return FinishScenario();
  }

  static constexpr double kPassStopLineBuffer = 0.3;  // unit: m
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_to_pnc_junction =  // 主车到路口距离
      current_pnc_junction->start_s - adc_front_edge_s;
  ADEBUG << "pnc_junction_overlap_id[" << pnc_junction_overlap_id
         << "] start_s[" << current_pnc_junction->start_s
         << "] distance_adc_to_pnc_junction[" << distance_adc_to_pnc_junction
         << "]";
  if (distance_adc_to_pnc_junction < -kPassStopLineBuffer) {
    // passed stop line
    return FinishStage(frame);
  }

  // set cruise_speed to slow down  设置主车限速，实现主车减速接近路口。
  frame->mutable_reference_line_info()->front().LimitCruiseSpeed(
      scenario_config_.approach_cruise_speed());

  // set right_of_way_status  设置路口路权，主车处于非保护状态。
  reference_line_info.SetJunctionRightOfWay(current_pnc_junction->start_s,
                                            false);
  // 再次执行Task 规划
  result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "BareIntersectionUnprotectedStageApproach planning error";
  }
  // 检查当前是否由障碍物阻塞   统计主车需要等待的障碍物序列wait_for_obstacle_ids，为其构建停止墙
  std::vector<std::string> wait_for_obstacle_ids;
  bool clear = CheckClear(reference_line_info, &wait_for_obstacle_ids);
  // 参数配置策略：enable_explicit_stop。如果True，则在主车需要等到，设置stop = true，
  // 为wait_for_obstacle_ids障碍物建立STOP虚拟墙。在连续5帧不需要等待的情况下，stop = false。
  if (scenario_config_.enable_explicit_stop()) {
    bool stop = false;
    static constexpr double kCheckClearDistance = 5.0;  // meter
    static constexpr double kStartWatchDistance = 2.0;  // meter
    // 到路口距离 < 5m  or  到路口距离 > 2m   or  存在堵塞 停车
    if (distance_adc_to_pnc_junction <= kCheckClearDistance &&
        distance_adc_to_pnc_junction >= kStartWatchDistance && !clear) {
      stop = true;
    } else if (distance_adc_to_pnc_junction < kStartWatchDistance) { // 到路口距离 < 2m 等5帧重置
      // creeping area
      counter_ = clear ? counter_ + 1 : 0;  

      if (counter_ >= 5) {
        counter_ = 0;  // reset
      } else {
        stop = true;
      }
    }

    if (stop) {
      // build stop decision
      // ADEBUG << "BuildStopDecision: bare pnc_junction["
      //        << pnc_junction_overlap_id << "] start_s["
      //        << current_pnc_junction->start_s << "]";
      const std::string virtual_obstacle_id =
          "PNC_JUNCTION_" + current_pnc_junction->object_id;
      planning::util::BuildStopDecision(
          virtual_obstacle_id, current_pnc_junction->start_s,
          scenario_config_.stop_distance(),
          StopReasonCode::STOP_REASON_STOP_SIGN, wait_for_obstacle_ids,
          "bare intersection", frame,
          &(frame->mutable_reference_line_info()->front()));
    }
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

bool BareIntersectionUnprotectedStageApproach::CheckClear( // 检查当前是否由障碍物阻塞
    const ReferenceLineInfo& reference_line_info,
    std::vector<std::string>* wait_for_obstacle_ids) {
  // TODO(all): move to conf
  static constexpr double kConf_min_boundary_t = 6.0;        // second
  static constexpr double kConf_ignore_max_st_min_t = 0.1;   // second
  static constexpr double kConf_ignore_min_st_min_s = 15.0;  // meter

  bool all_far_away = true;
  for (auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (obstacle->IsVirtual() || obstacle->IsStatic()) {  // 剔除虚拟且静态障碍物（比如虚拟墙）
      continue;
    }
    //该障碍物与adc mint < 6s（可能需要建立停止墙）  障碍物的视距基本最大就是6s？？？？？
    if (obstacle->reference_line_st_boundary().min_t() < kConf_min_boundary_t) {  
      const double kepsilon = 1e-6;
      double obstacle_traveled_s =  // 移动距离
          obstacle->reference_line_st_boundary().bottom_left_point().s() -
          obstacle->reference_line_st_boundary().bottom_right_point().s();
      // ADEBUG << "obstacle[" << obstacle->Id() << "] obstacle_st_min_t["
      //        << obstacle->reference_line_st_boundary().min_t()
      //        << "] obstacle_st_min_s["
      //        << obstacle->reference_line_st_boundary().min_s()
      //        << "] obstacle_traveled_s[" << obstacle_traveled_s << "]";

      // ignore the obstacle which is already on reference line and moving
      // along the direction of ADC   忽略已经在参考线上并沿着ADC的方向移动的障碍物
      // 障碍物静止  or  紧贴adc   or   和本车相差15m以上（忽略不建立停止墙）
      if (obstacle_traveled_s < kepsilon &&
          obstacle->reference_line_st_boundary().min_t() <
              kConf_ignore_max_st_min_t &&
          obstacle->reference_line_st_boundary().min_s() >  // 离本车很远不碰撞
              kConf_ignore_min_st_min_s) {
        continue;
      }

      wait_for_obstacle_ids->push_back(obstacle->Id());
      all_far_away = false;
    }
  }
  return all_far_away;
}

StageResult BareIntersectionUnprotectedStageApproach::FinishStage(
    Frame* frame) {
  next_stage_ = "BARE_INTERSECTION_UNPROTECTED_INTERSECTION_CRUISE";

  // reset cruise_speed
  auto& reference_line_info = frame->mutable_reference_line_info()->front();
  reference_line_info.LimitCruiseSpeed(FLAGS_default_cruise_speed);

  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
