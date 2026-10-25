#ifndef SCAN2MESH_POINT_TYPES_H_
#define SCAN2MESH_POINT_TYPES_H_
#include <cstdint>
#include <type_traits>
namespace scan2mesh {
// Shared by host code and CUDA; do not duplicate these layouts in device code.
struct PointXYZRGB {
  float x = 0, y = 0, z = 0;
  std::uint8_t r = 0, g = 0, b = 0, a = 255;
};
struct PointXYZRGBNormal {
  float x = 0, y = 0, z = 0;
  float normal_x = 0, normal_y = 0, normal_z = 0;
  std::uint8_t r = 0, g = 0, b = 0, a = 255;
};

static_assert(std::is_standard_layout<PointXYZRGB>::value &&
                  std::is_trivially_copyable<PointXYZRGB>::value,
              "Points must be POD-compatible");
static_assert(std::is_standard_layout<PointXYZRGBNormal>::value &&
                  std::is_trivially_copyable<PointXYZRGBNormal>::value,
              "Points must be POD-compatible");
}  // namespace scan2mesh
#endif
