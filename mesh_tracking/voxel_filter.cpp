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

#include <cmath>
#include <cstdint>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include "voxel_filter.h"

namespace {

struct VoxelKey {
  int x;
  int y;
  int z;
  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const {
    std::size_t seed = std::hash<int>()(key.x);
    seed ^= std::hash<int>()(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

template <typename PointT>
VoxelKey GetVoxelKey(const PointT& point, float resolution) {
  return VoxelKey{static_cast<int>(std::floor(point.x / resolution)),
                  static_cast<int>(std::floor(point.y / resolution)),
                  static_cast<int>(std::floor(point.z / resolution))};
}

template <typename PointT>
std::vector<bool> RandomizedVoxelFilterIndices(
    const scan2mesh::PointCloud<PointT>& point_cloud, float resolution) {
  if (point_cloud.empty() || resolution <= 0.0f) {
    return std::vector<bool>(point_cloud.size(), true);
  }

  std::minstd_rand0 generator;
  struct VoxelData {
    std::size_t point_count = 0;
    std::size_t selected_index = 0;
  };

  std::unordered_map<VoxelKey, VoxelData, VoxelKeyHash> voxels;

  voxels.reserve(point_cloud.size());
  for (std::size_t i = 0; i < point_cloud.size(); ++i) {
    const auto& point = point_cloud.points[i];
    const VoxelKey voxel_key = GetVoxelKey(point, resolution);
    VoxelData& voxel = voxels[voxel_key];
    ++voxel.point_count;
    if (voxel.point_count == 1) {
      voxel.selected_index = i;
      continue;
    }
    std::uniform_int_distribution<std::size_t> distribution(1,
                                                            voxel.point_count);

    if (distribution(generator) == voxel.point_count) {
      voxel.selected_index = i;
    }
  }

  std::vector<bool> points_used(point_cloud.size(), false);
  for (const auto& voxel : voxels) {
    points_used[voxel.second.selected_index] = true;
  }

  return points_used;
}

template <typename PointT>
scan2mesh::PointCloud<PointT> RandomizedVoxelFilter(
    const scan2mesh::PointCloud<PointT>& point_cloud, float resolution) {
  const std::vector<bool> points_used =
      RandomizedVoxelFilterIndices(point_cloud, resolution);

  scan2mesh::PointCloud<PointT> filtered_cloud;

  filtered_cloud.reserve(point_cloud.size());

  for (std::size_t i = 0; i < point_cloud.size(); ++i) {
    if (points_used[i]) {
      filtered_cloud.points.push_back(point_cloud.points[i]);
    }
  }

  return filtered_cloud;
}

template <typename PointT>
scan2mesh::PointCloud<PointT> FilterByMaxRange(
    const scan2mesh::PointCloud<PointT>& point_cloud, float max_range) {
  if (max_range <= 0.0f) {
    return point_cloud;
  }

  const float max_range_squared = max_range * max_range;
  scan2mesh::PointCloud<PointT> filtered_cloud;
  filtered_cloud.reserve(point_cloud.size());
  for (const auto& point : point_cloud.points) {
    const float range_squared =
        point.x * point.x + point.y * point.y + point.z * point.z;

    if (range_squared <= max_range_squared) {
      filtered_cloud.points.push_back(point);
    }
  }

  return filtered_cloud;
}

template <typename PointT>
scan2mesh::PointCloud<PointT> AdaptivelyVoxelFiltered(
    const scan2mesh::PointCloud<PointT>& point_cloud,
    const AdaptiveVoxelFilterOptions& options) {
  if (point_cloud.size() <= options.min_num_points) {
    return point_cloud;
  }

  scan2mesh::PointCloud<PointT> result =
      RandomizedVoxelFilter(point_cloud, options.max_length);

  if (result.size() >= options.min_num_points) {
    return result;
  }

  // Search for a smaller voxel size that produces enough points.
  for (float high_length = options.max_length;
       high_length > 1e-2f * options.max_length; high_length /= 2.0f) {
    float low_length = high_length / 2.0f;

    result = RandomizedVoxelFilter(point_cloud, low_length);

    if (result.size() >= options.min_num_points) {
      while ((high_length - low_length) / low_length > 0.1f) {
        const float mid_length = (low_length + high_length) * 0.5f;

        scan2mesh::PointCloud<PointT> candidate =
            RandomizedVoxelFilter(point_cloud, mid_length);

        if (candidate.size() >= options.min_num_points) {
          low_length = mid_length;
          result = std::move(candidate);
        } else {
          high_length = mid_length;
        }
      }

      return result;
    }
  }

  return result;
}

}  // namespace

scan2mesh::ColorCloud VoxelFilter(const scan2mesh::ColorCloud& point_cloud,
                                  float resolution) {
  return RandomizedVoxelFilter(point_cloud, resolution);
}

scan2mesh::NormalCloud VoxelFilter(const scan2mesh::NormalCloud& point_cloud,
                                   float resolution) {
  return RandomizedVoxelFilter(point_cloud, resolution);
}

scan2mesh::ColorCloud AdaptiveVoxelFilterWithRangeLimit(
    const scan2mesh::ColorCloud& point_cloud,
    const AdaptiveVoxelFilterOptions& options) {
  const auto range_filtered_cloud =
      FilterByMaxRange(point_cloud, options.max_range);

  return AdaptivelyVoxelFiltered(range_filtered_cloud, options);
}

scan2mesh::ColorCloud AdaptiveVoxelFilter(
    const scan2mesh::ColorCloud& point_cloud,
    const AdaptiveVoxelFilterOptions& options) {
  return AdaptivelyVoxelFiltered(point_cloud, options);
}
