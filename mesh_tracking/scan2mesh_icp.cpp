/*
 * Copyright 2026 The Eternelle Authors
 *
 * Non-Commercial Source-Available License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to use,
 * copy, modify, and distribute the Software for non-commercial purposes, subject
 * to the following conditions:
 *
 * 1. Non-commercial use only.
 *    The Software may be used for personal, academic, educational, research,
 *    evaluation, and other non-commercial purposes.
 *
 * 2. Commercial use.
 *    Commercial use of the Software, including use in a commercial product,
 *    service, system, or business operation, requires prior written permission
 *    from the copyright holder.
 *
 * 3. Redistribution.
 *    Redistributions of the Software, with or without modification, must retain
 *    this copyright notice and this license.
 *
 * 4. No trademark rights.
 *    This license does not grant permission to use the names "Eternelle",
 *    "Eternelle3D", "Scan2Mesh", or related trademarks to imply endorsement.
 *
 * 5. No warranty.
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *
 * 6. Commercial license.
 *    The copyright holder may grant separate commercial licenses under
 *    individually negotiated terms.
 */

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <stdexcept>

#include "scan2mesh_icp.h"

Scan2MeshICP::Scan2MeshICP(const Pose&, const Pose& sensor_to_imu,
                           unsigned int vertex_num, unsigned int face_num,
                           std::vector<float>* vertices,
                           std::vector<unsigned int>* faces, float dist_thresh)
    : T_imu_hor(sensor_to_imu),
      distance_threshold(dist_thresh),
      mesh_raycaster(new MeshRaycaster(vertex_num, face_num, vertices, faces)) {
  if (!std::isfinite(dist_thresh) || dist_thresh <= 0)
    throw std::invalid_argument("ICP distance threshold must be positive");
#ifdef SCAN2MESH_USE_CUDA
  block_normal_equations.create(64);
  reduced_normal_equations.create(1);
#endif
}
Scan2MeshICP::~Scan2MeshICP() = default;

bool Scan2MeshICP::EstimatePose(Pose& pose,
                                const scan2mesh::ColorCloud::ConstPtr& cloud,
                                int64_t /*timestamp*/) {
  if (!cloud || cloud->size() < 6 || !pose.allFinite()) return false;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  const Pose initial = pose;
  Pose estimate = pose;
  Pose increment = Pose::Identity();
  const Eigen::Matrix3f initial_inverse_rotation =
      initial.topLeftCorner<3, 3>().transpose();
  const Eigen::Vector3f initial_translation = initial.topRightCorner<3, 1>();
  auto targets = std::make_shared<scan2mesh::NormalCloud>();
#ifdef SCAN2MESH_USE_CUDA
  DeviceArray<scan2mesh::PointXYZRGB> source_device;
  source_device.upload(cloud->points);
#endif
  for (int iteration = 0; iteration < 5; ++iteration) {
    const Pose world_sensor = estimate * T_imu_hor;
    if (!mesh_raycaster->RaycastCloud(cloud, world_sensor, targets))
      return false;
    Matrix6d A = Matrix6d::Zero();
    Vector6d b = Vector6d::Zero();
    std::size_t correspondence_count = 0;
#ifdef SCAN2MESH_USE_CUDA
    DeviceArray<scan2mesh::PointXYZRGBNormal> target_device;
    target_device.upload(targets->points);
    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> current_rotation =
        estimate.topLeftCorner<3, 3>();
    Eigen::Vector3f current_translation = estimate.topRightCorner<3, 1>();
    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> sensor_rotation =
        T_imu_hor.topLeftCorner<3, 3>();
    Eigen::Vector3f sensor_translation = T_imu_hor.topRightCorner<3, 1>();
    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> previous_inverse =
        initial_inverse_rotation;
    Eigen::Matrix<float, 6, 6, Eigen::RowMajor> gpu_A;
    Eigen::Matrix<float, 6, 1> gpu_b;
    float residual[2] = {};
    ComputeICPNormalEquations(
        device_cast<Matrix3f>(current_rotation),
        device_cast<float3>(current_translation),
        device_cast<Matrix3f>(sensor_rotation),
        device_cast<float3>(sensor_translation),
        device_cast<Matrix3f>(previous_inverse),
        device_cast<float3>(initial_translation), source_device, target_device,
        distance_threshold, 0.0f, block_normal_equations,
        reduced_normal_equations, gpu_A.data(), gpu_b.data(), residual, 128, 64,
        static_cast<int>(cloud->size()));
    A = gpu_A.cast<double>();
    b = gpu_b.cast<double>();
    if (!std::isfinite(residual[1])) return false;
    correspondence_count = static_cast<std::size_t>(residual[1]);
#else
    for (std::size_t i = 0; i < cloud->size(); ++i) {
      const auto& source = cloud->points[i];
      const auto& target = targets->points[i];
      const Eigen::Vector3f p_world =
          world_sensor.topLeftCorner<3, 3>() *
              Eigen::Vector3f(source.x, source.y, source.z) +
          world_sensor.topRightCorner<3, 1>();
      const Eigen::Vector3f q_world(target.x, target.y, target.z);
      const Eigen::Vector3f n_world(target.normal_x, target.normal_y,
                                    target.normal_z);
      if (!p_world.allFinite() || !q_world.allFinite() ||
          !n_world.allFinite() || n_world.norm() <= 0.1f ||
          (q_world - p_world).norm() > distance_threshold)
        continue;
      // Linearize in the initial IMU frame, matching the CUDA equations.
      const Eigen::Vector3d p =
          (initial_inverse_rotation * (p_world - initial_translation))
              .cast<double>();
      const Eigen::Vector3d q =
          (initial_inverse_rotation * (q_world - initial_translation))
              .cast<double>();
      const Eigen::Vector3d n =
          (initial_inverse_rotation * n_world).cast<double>();
      Vector6d J;
      J.head<3>() = n;
      J.tail<3>() = p.cross(n);
      const double residual = n.dot(p - q);
      const double weight =
          std::abs(residual) > 0.1 ? 0.1 / std::abs(residual) : 1.0;
      A.noalias() += weight * J * J.transpose();
      b.noalias() += weight * J * residual;
      ++correspondence_count;
    }
#endif
    if (correspondence_count < 6 || !A.allFinite() || !b.allFinite())
      return false;
    Eigen::SelfAdjointEigenSolver<Matrix6d> spectrum(A, Eigen::EigenvaluesOnly);
    if (spectrum.info() != Eigen::Success ||
        spectrum.eigenvalues()(0) <= spectrum.eigenvalues()(5) * 1e-9)
      return false;
    Eigen::LDLT<Matrix6d> solver(A);
    const Vector6d step = solver.solve(-b);
    if (solver.info() != Eigen::Success || !step.allFinite()) return false;
    Pose delta = Pose::Identity();
    const Eigen::Vector3d rotation = step.tail<3>();
    const double angle = rotation.norm();
    if (angle > 1e-12)
      delta.topLeftCorner<3, 3>() = Eigen::AngleAxisd(angle, rotation / angle)
                                        .toRotationMatrix()
                                        .cast<float>();
    delta.topRightCorner<3, 1>() = step.head<3>().cast<float>();
    increment = delta * increment;
    estimate = initial * increment;
    if (!estimate.allFinite()) return false;
    if (step.norm() < 1e-6) break;
  }
  pose = estimate;
  return true;
}
