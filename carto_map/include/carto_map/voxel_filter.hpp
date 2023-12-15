#include "cartographer/sensor/point_cloud.h"
#include "cartographer/common/math.h"
#include "absl/container/flat_hash_map.h"
#include <random>
#include <bitset>
#include <cmath>

uint64_t GetVoxelCellIndex(const Eigen::Vector3f& point,
                               const float resolution) {
  const Eigen::Array3f index = point.array() / resolution;
  const uint64_t x = cartographer::common::RoundToInt(index.x());
  const uint64_t y = cartographer::common::RoundToInt(index.y());
  const uint64_t z = cartographer::common::RoundToInt(index.z());
  return (x << 42) + (y << 21) + z;
}

template <class T, class PointFunction>
std::vector<bool> RandomizedVoxelFilterIndices(
    const std::vector<T>& point_cloud, const float resolution,
    PointFunction&& point_function) {
  std::minstd_rand0 generator;
  absl::flat_hash_map<uint64_t, std::pair<int, int>>
      voxel_count_and_point_index;
  for (size_t i = 0; i < point_cloud.size(); i++) {
    auto& voxel = voxel_count_and_point_index[GetVoxelCellIndex(
        point_function(point_cloud[i]), resolution)];
    voxel.first++;
    if (voxel.first == 1) {
      voxel.second = i;
    } else {
      std::uniform_int_distribution<> distribution(1, voxel.first);
      if (distribution(generator) == voxel.first) {
        voxel.second = i;
      }
    }
  }
  std::vector<bool> points_used(point_cloud.size(), false);
  for (const auto& voxel_and_index : voxel_count_and_point_index) {
    points_used[voxel_and_index.second.second] = true;
  }
  return points_used;
}

cartographer::sensor::PointCloud VoxelFilter(
    const cartographer::sensor::PointCloud& point_cloud,
    const float resolution) {
  const std::vector<bool> points_used = RandomizedVoxelFilterIndices(
      point_cloud.points(), resolution,
      [](const cartographer::sensor::RangefinderPoint& point) { return point.position; });

  cartographer::sensor::PointCloud res;
  for (size_t i = 0; i < point_cloud.size(); i++) {
    if (points_used[i]) {
      res.push_back(point_cloud[i]);
    }
  }
  LOG(INFO) << "VoxelFilter input size: " << point_cloud.size()
            << "; output size: " << res.size();
  return res;
}