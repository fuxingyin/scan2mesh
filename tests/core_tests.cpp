#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "mesh_tracking/scan2mesh_icp.h"
#include "mesh_tracking/voxel_filter.h"
#include "utils/eigen_utils.h"
#include "utils/ply_reader.h"

namespace {
using Cloud = scan2mesh::ColorCloud;
using Pose = Scan2MeshICP::Pose;
void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void TestFiniteChecks() {
  Pose matrix = Pose::Identity();
  Require(scan2mesh::AllFinite(matrix), "Finite matrix rejected");
  matrix(2, 3) = std::numeric_limits<float>::infinity();
  Require(!scan2mesh::AllFinite(matrix), "Infinite matrix accepted");
  Require(!scan2mesh::AllFinite(matrix.block<3, 1>(0, 3)),
          "Infinite block expression accepted");
  matrix(2, 3) = -std::numeric_limits<float>::infinity();
  Require(!scan2mesh::AllFinite(matrix.transpose()),
          "Negative infinity in transpose accepted");
  Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
  Require(scan2mesh::AllFinite(rotation.coeffs()), "Finite quaternion rejected");
  rotation.x() = std::numeric_limits<float>::quiet_NaN();
  Require(!scan2mesh::AllFinite(rotation.coeffs()), "NaN quaternion accepted");
  Require(scan2mesh::AllFinite(Eigen::Vector3f(1, 2, 3)),
          "Finite temporary vector rejected");
}
struct Fixtures {
  std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      ("scan2mesh-tests-" +
       std::to_string(
           std::chrono::steady_clock::now().time_since_epoch().count()));
  Fixtures() { std::filesystem::create_directory(path); }
  ~Fixtures() { std::filesystem::remove_all(path); }
  std::string File(const char* name) const { return (path / name).string(); }
};

template <typename T>
void Binary(std::ofstream& stream, T value, bool big_endian) {
  unsigned char bytes[sizeof(T)];
  std::memcpy(bytes, &value, sizeof(T));
  const uint16_t one = 1;
  const bool host_little = *reinterpret_cast<const uint8_t*>(&one) == 1;
  if (host_little == big_endian) std::reverse(bytes, bytes + sizeof(T));
  stream.write(reinterpret_cast<char*>(bytes), sizeof(T));
}

void TestPLY(const Fixtures& fixtures) {
  const auto ascii = fixtures.File("mixed.ply");
  {
    std::ofstream out(ascii);
    out << "ply\nformat ascii 1.0\nelement vertex 3\nproperty uchar green\n"
        << "property double z\nproperty float x\nproperty uchar blue\nproperty "
           "double y\n"
        << "property uchar red\nproperty float confidence\nelement face 1\n"
        << "property uchar red\nproperty list uchar int "
           "vertex_indices\nend_header\n"
        << "20 3 1 30 2 10 0.5\n21 6 4 31 5 11 0.6\n22 9 7 32 8 12 0.7\n"
        << "255 3 0 1 2\n";
  }
  Cloud cloud;
  Require(LoadPLYCloud(ascii, cloud), "Mixed ASCII cloud failed");
  Require(cloud.size() == 3 && cloud.points[0].x == 1 &&
              cloud.points[0].y == 2 && cloud.points[0].z == 3 &&
              cloud.points[0].r == 10 && cloud.points[0].g == 20 &&
              cloud.points[0].b == 30,
          "Property ordering/type/color conversion failed");
  unsigned int nv = 99, nf = 99;
  std::vector<float> vertices;
  std::vector<unsigned int> faces;
  Require(LoadPLYMesh(ascii, nv, nf, vertices, faces),
          "Mixed ASCII mesh failed");
  Require(nv == 3 && nf == 1 && vertices[8] == 9 && faces[2] == 2,
          "Mesh values failed");
  const auto saved = fixtures.File("roundtrip.ply");
  Require(SavePLYCloud(saved, cloud), "Binary save failed");
  Cloud roundtrip;
  Require(LoadPLYCloud(saved, roundtrip), "Binary reload failed");
  Require(roundtrip.size() == 3 && roundtrip.points[2].b == 32 &&
              roundtrip.points[2].z == 9,
          "Binary roundtrip values failed");
  for (bool big : {false, true}) {
    const auto path = fixtures.File(big ? "big.ply" : "little.ply");
    {
      std::ofstream out(path, std::ios::binary);
      out << "ply\nformat binary_" << (big ? "big" : "little")
          << "_endian 1.0\n"
          << "element vertex 3\nproperty double x\nproperty float y\nproperty "
             "float z\n"
          << "element face 1\nproperty list uchar ushort "
             "vertex_indices\nend_header\n";
      for (int i = 0; i < 3; ++i) {
        Binary(out, double(i), big);
        Binary(out, float(i + 1), big);
        Binary(out, float(i + 2), big);
      }
      Binary(out, uint8_t(3), big);
      for (int i = 0; i < 3; ++i) Binary(out, uint16_t(i), big);
    }
    Require(LoadPLYMesh(path, nv, nf, vertices, faces),
            "Binary endian mesh failed");
    Require(vertices[6] == 2 && faces[2] == 2, "Binary endian values failed");
    Require(LoadPLYCloud(path, roundtrip), "Colorless cloud failed");
    Require(roundtrip.points[0].r == 0, "Missing colors not defaulted");
  }
  for (const char* payload : {"3 0 1 9", "4 0 1 2 0", "3 0 1", "3 -1 1 2"}) {
    const auto path = fixtures.File("invalid.ply");
    {
      std::ofstream out(path);
      out << "ply\nformat ascii 1.0\nelement vertex 3\nproperty float "
             "x\nproperty float y\n"
          << "property float z\nelement face 1\nproperty list uchar int "
             "vertex_indices\n"
          << "end_header\n0 0 0\n1 0 0\n0 1 0\n"
          << payload;
    }
    Require(!LoadPLYMesh(path, nv, nf, vertices, faces),
            "Invalid mesh accepted");
    Require(nv == 0 && nf == 0 && vertices.empty() && faces.empty(),
            "Failure left stale mesh data");
  }
  const auto truncated = fixtures.File("truncated.ply");
  std::filesystem::copy_file(saved, truncated);
  std::filesystem::resize_file(truncated,
                               std::filesystem::file_size(truncated) - 1);
  Require(!LoadPLYCloud(truncated, roundtrip) && roundtrip.empty(),
          "Truncated cloud accepted");
}

void TestICP() {
  std::vector<float> vertices = {-3, -3, -3, 3, -3, -3, 3, 3, -3, -3, 3, -3,
                                 -3, -3, 3,  3, -3, 3,  3, 3, 3,  -3, 3, 3};
  std::vector<unsigned int> faces = {0, 1, 2, 0, 2, 3, 4, 6, 5, 4, 7, 6,
                                     0, 4, 5, 0, 5, 1, 3, 2, 6, 3, 6, 7,
                                     0, 3, 7, 0, 7, 4, 1, 5, 6, 1, 6, 2};
  Pose extrinsic = Pose::Identity();
  extrinsic.topLeftCorner<3, 3>() =
      Eigen::AngleAxisf(0.13f, Eigen::Vector3f::UnitX()).toRotationMatrix();
  extrinsic.topRightCorner<3, 1>() = Eigen::Vector3f(0.2f, -0.1f, 0.15f);
  Pose truth = Pose::Identity();
  truth.topLeftCorner<3, 3>() =
      (Eigen::AngleAxisf(0.25f, Eigen::Vector3f::UnitZ()) *
       Eigen::AngleAxisf(0.15f, Eigen::Vector3f::UnitY()))
          .toRotationMatrix();
  truth.topRightCorner<3, 1>() = Eigen::Vector3f(0.3f, -0.2f, 0.1f);
  const Pose sensor_inverse = (truth * extrinsic).inverse();
  auto cloud = std::make_shared<Cloud>();
  for (int axis = 0; axis < 3; ++axis) {
    for (int sign : {-1, 1})
      for (int a = -2; a <= 2; ++a)
        for (int b = -2; b <= 2; ++b) {
          Eigen::Vector3f world;
          world[axis] = sign * 3.0f;
          world[(axis + 1) % 3] = a * 0.8f;
          world[(axis + 2) % 3] = b * 0.8f;
          const Eigen::Vector3f sensor =
              sensor_inverse.topLeftCorner<3, 3>() * world +
              sensor_inverse.topRightCorner<3, 1>();
          scan2mesh::PointXYZRGB p;
          p.x = sensor.x();
          p.y = sensor.y();
          p.z = sensor.z();
          cloud->push_back(p);
        }
  }
  Scan2MeshICP tracker(Pose::Identity(), extrinsic, 8, 12, &vertices, &faces);
  Pose perturbation = Pose::Identity();
  perturbation.topLeftCorner<3, 3>() =
      Eigen::AngleAxisf(0.025f, Eigen::Vector3f(1, 2, 3).normalized())
          .toRotationMatrix();
  perturbation.topRightCorner<3, 1>() = Eigen::Vector3f(0.04f, -0.03f, 0.02f);
  Pose estimate = truth * perturbation;
  Require(tracker.EstimatePose(estimate, cloud, 0),
          "Synthetic room alignment failed");
  Require((estimate - truth).norm() < 1e-4f,
          "ICP pose/extrinsic composition is incorrect");
  Cloud transformed;
  scan2mesh::TransformPointCloud(*cloud, transformed, extrinsic);
  scan2mesh::TransformPointCloud(transformed, transformed, extrinsic.inverse());
  Require(std::abs(transformed.points[0].x - cloud->points[0].x) < 1e-5,
          "In-place transform failed");
  auto invalid = std::make_shared<Cloud>();
  invalid->points.resize(20);
  const Pose unchanged = estimate;
  Require(!tracker.EstimatePose(estimate, invalid, 0) && estimate == unchanged,
          "No-hit ICP altered pose");
  for (auto& p : invalid->points) p.x = std::numeric_limits<float>::quiet_NaN();
  Require(!tracker.EstimatePose(estimate, invalid, 0) && estimate == unchanged,
          "Nonfinite ICP altered pose");
  auto plane = std::make_shared<Cloud>();
  for (int i = 0; i < 20; ++i) {
    scan2mesh::PointXYZRGB p;
    p.x = 3;
    p.y = (i % 5 - 2) * .3f;
    p.z = (i / 5 - 2) * .3f;
    plane->push_back(p);
  }
  Scan2MeshICP planar_tracker(Pose::Identity(), Pose::Identity(), 8, 12,
                              &vertices, &faces);
  estimate = Pose::Identity();
  Require(!planar_tracker.EstimatePose(estimate, plane, 0) &&
              estimate == Pose::Identity(),
          "Degenerate plane accepted");
  const auto filtered = VoxelFilter(*cloud, 0.5f);
  Require(!filtered.empty() && filtered.size() <= cloud->size(),
          "Voxel filtering failed");
}
}  // namespace

int main() {
  try {
#ifdef SCAN2MESH_USE_CUDA
    cudaSafeCall(cudaSetDevice(0));
#endif
    Fixtures fixtures;
    TestFiniteChecks();
    TestPLY(fixtures);
    TestICP();
    std::cout << "All PLY, raycasting, transform, voxel and ICP regressions "
                 "passed.\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Test failed: " << e.what() << '\n';
    return 1;
  }
}
