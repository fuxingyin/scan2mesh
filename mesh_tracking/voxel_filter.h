/*
 * Copyright 2016 The Cartographer Authors
 * Copyright 2026 The Eternelle Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef VOXEL_FILTER_H_
#define VOXEL_FILTER_H_

#include <cstddef>

#include "utils/point_cloud.h"

struct AdaptiveVoxelFilterOptions {
  float max_length = 0.5f;
  std::size_t min_num_points = 1000;
  float max_range = 50.0f;
};

// Keep one original point per voxel.
// The original point coordinates are NOT averaged or modified.
scan2mesh::ColorCloud VoxelFilter(const scan2mesh::ColorCloud& point_cloud,
                                  float resolution);

scan2mesh::NormalCloud VoxelFilter(const scan2mesh::NormalCloud& point_cloud,
                                   float resolution);

scan2mesh::ColorCloud AdaptiveVoxelFilterWithRangeLimit(
    const scan2mesh::ColorCloud& point_cloud,
    const AdaptiveVoxelFilterOptions& options);

scan2mesh::ColorCloud AdaptiveVoxelFilter(
    const scan2mesh::ColorCloud& point_cloud,
    const AdaptiveVoxelFilterOptions& options);

#endif  // VOXEL_FILTER_H_