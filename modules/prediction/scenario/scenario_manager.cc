/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/scenario/scenario_manager.h"

#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

void ScenarioManager::Run(ContainerManager* container_manager) {
    // 提取环境特征
  auto environment_features =
      FeatureExtractor::ExtractEnvironmentFeatures(container_manager);

  // 分析提取的特征，确定场景特征
  auto ptr_scenario_features = ScenarioAnalyzer::Analyze(environment_features);

  // 设置当前场景
  current_scenario_ = ptr_scenario_features->scenario();

  // TODO: 包括车道和路口过滤器的其他功能
  // TODO(all) other functionalities including lane, junction filters
}

const Scenario ScenarioManager::scenario() const { return current_scenario_; }

}  // namespace prediction
}  // namespace apollo
