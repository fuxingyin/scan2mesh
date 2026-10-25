#ifndef SCAN2MESH_EIGEN_UTILS_H_
#define SCAN2MESH_EIGEN_UTILS_H_

#include <Eigen/Core>
#include <cmath>
#include <cstddef>

namespace scan2mesh {

// Use scalar checks so validation also works with Eigen versions that do not
// provide MatrixBase::allFinite(). Index by row/column to support expressions.
template <typename Derived>
bool AllFinite(const Eigen::MatrixBase<Derived>& matrix) {
  for (std::ptrdiff_t row = 0; row < matrix.rows(); ++row) {
    for (std::ptrdiff_t col = 0; col < matrix.cols(); ++col) {
      if (!std::isfinite(matrix(row, col))) return false;
    }
  }
  return true;
}

}  // namespace scan2mesh
#endif
