#ifndef SCAN2MESH_POINT_CLOUD_H_
#define SCAN2MESH_POINT_CLOUD_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <vector>

#include "point_types.h"

namespace scan2mesh {

template <typename PointT>
struct PointCloud {
  using Ptr = std::shared_ptr<PointCloud>;
  using ConstPtr = std::shared_ptr<const PointCloud>;
  std::vector<PointT> points;
  bool empty() const { return points.empty(); }
  std::size_t size() const { return points.size(); }
  void clear() { points.clear(); }
  void reserve(std::size_t n) { points.reserve(n); }
  void push_back(const PointT& point) { points.push_back(point); }
  PointCloud& operator+=(const PointCloud& other) {
    points.insert(points.end(), other.points.begin(), other.points.end());
    return *this;
  }
};

using ColorCloud = PointCloud<PointXYZRGB>;
using NormalCloud = PointCloud<PointXYZRGBNormal>;

template <typename PointT, typename Derived>
void TransformPointCloud(const PointCloud<PointT>& input,
                         PointCloud<PointT>& output,
                         const Eigen::MatrixBase<Derived>& transform) {
  if (&input != &output) output = input;
  for (auto& point : output.points) {
    const Eigen::Vector3f p = transform.template topLeftCorner<3, 3>() *
                                  Eigen::Vector3f(point.x, point.y, point.z) +
                              transform.template topRightCorner<3, 1>();
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
  }
}
}  // namespace scan2mesh
#endif
