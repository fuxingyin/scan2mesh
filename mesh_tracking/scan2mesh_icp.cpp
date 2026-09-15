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

#include "scan2mesh_icp.h"

#include <chrono>

Scan2MeshICP::Scan2MeshICP(
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& T_imu_vti,
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& T_imu_hor,
    const unsigned int vertex_num, const unsigned int face_num,
    vector<float>* vertices, vector<unsigned int>* faces, float dist_thresh)
    : T_imu_vti(T_imu_vti),
      T_imu_hor(T_imu_hor),
      mesh_vertex_num(vertex_num),
      mesh_face_num(face_num),
      mesh_vertices(vertices),
      mesh_faces(faces),
      distance_threshold(dist_thresh) {
  block_normal_equations.create(MAX_THREADS);
  reduced_normal_equations.create(1);

  R_imu_vti = T_imu_vti.topLeftCorner(3, 3);
  t_imu_vti = T_imu_vti.topRightCorner(3, 1);
  R_vti_imu = R_imu_vti.inverse();
  t_vti_imu = -R_vti_imu * t_imu_vti;

  R_imu_hor = T_imu_hor.topLeftCorner(3, 3);
  t_imu_hor = T_imu_hor.topRightCorner(3, 1);
  R_hor_imu = R_imu_hor.inverse();
  t_hor_imu = -R_hor_imu * t_imu_hor;

  mesh_raycaster.reset(new MeshRaycaster(mesh_vertex_num, mesh_face_num,
                                         mesh_vertices, mesh_faces));

  device_R_imu_vti = device_cast<Matrix3f>(R_imu_vti);
  device_t_imu_vti = device_cast<float3>(t_imu_vti);
  device_R_vti_imu = device_cast<Matrix3f>(R_vti_imu);
  device_t_vti_imu = device_cast<float3>(t_vti_imu);

  device_R_imu_hor = device_cast<Matrix3f>(R_imu_hor);
  device_t_imu_hor = device_cast<float3>(t_imu_hor);
  device_R_hor_imu = device_cast<Matrix3f>(R_hor_imu);
  device_t_hor_imu = device_cast<float3>(t_hor_imu);
}

Scan2MeshICP::~Scan2MeshICP() {}

Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Scan2MeshICP::Rodrigues(
    const Eigen::Vector3d& src) {
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> dst =
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Identity();
  double rx, ry, rz, theta;
  rx = src(0);
  ry = src(1);
  rz = src(2);
  theta = src.norm();
  if (theta >= std::numeric_limits<double>::epsilon()) {
    const double I[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double c = cos(theta);
    double s = sin(theta);
    double c1 = 1. - c;
    double itheta = theta ? 1. / theta : 0.;
    rx *= itheta;
    ry *= itheta;
    rz *= itheta;
    double rrt[] = {rx * rx, rx * ry, rx * rz, rx * ry, ry * ry,
                    ry * rz, rx * rz, ry * rz, rz * rz};
    double _r_x_[] = {0, -rz, ry, rz, 0, -rx, -ry, rx, 0};
    double R[9];
    for (int k = 0; k < 9; k++) {
      R[k] = c * I[k] + c1 * rrt[k] + s * _r_x_[k];
    }
    memcpy(dst.data(), &R[0],
           sizeof(Eigen::Matrix<double, 3, 3, Eigen::RowMajor>));
  }
  return dst;
}

bool Scan2MeshICP::EstimatePose(
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& T_world_imu,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_in_hor,
    const int64_t timestamp) {
  if (!cloud_in_hor || cloud_in_hor->empty()) {
    return false;
  }

  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Rprev =
      T_world_imu.topLeftCorner<3, 3>();
  Eigen::Vector3f tprev = T_world_imu.topRightCorner<3, 1>();
  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Rprev_inv = Rprev.inverse();

  const auto device_Rprev_inv = device_cast<Matrix3f>(Rprev_inv);
  const auto device_tprev = device_cast<float3>(tprev);

  const Eigen::Matrix<float, 4, 4, Eigen::RowMajor> T_wi_init = T_world_imu;
  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> T_wi_est = T_world_imu;
  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> T_inc_est =
      Eigen::Matrix<float, 4, 4, Eigen::RowMajor>::Identity();

  DeviceArray<pcl::PointXYZRGB> cloud_in_sta_dev;
  cloud_in_sta_dev.upload(cloud_in_hor->points);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cast_hor(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  constexpr int kMaxIcpIterations = 5;
  for (int i = 0; i < kMaxIcpIterations; ++i) {
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> T_world_hor =
        T_wi_est * T_imu_hor;

    mesh_raycaster->RaycastCloud(cloud_in_hor, T_world_hor, cloud_cast_hor);
    if (cloud_cast_hor->empty()) {
      break;
    }

    DeviceArray<pcl::PointXYZRGBNormal> cloud_cast_hor_dev;
    cloud_cast_hor_dev.upload(cloud_cast_hor->points);

    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Rcurr =
        T_wi_est.topLeftCorner<3, 3>();
    Eigen::Vector3f tcurr = T_wi_est.topRightCorner<3, 1>();

    const auto device_Rcurr = device_cast<Matrix3f>(Rcurr);
    const auto device_tcurr = device_cast<float3>(tcurr);

    Eigen::Matrix<float, 6, 6, Eigen::RowMajor> A_icp;
    Eigen::Matrix<float, 6, 1> b_icp;
    float residual[2] = {0.0f, 0.0f};

    ComputeICPNormalEquations(
        device_Rcurr, device_tcurr, device_R_imu_hor, device_t_imu_hor,
        device_Rprev_inv, device_tprev, cloud_in_sta_dev, cloud_cast_hor_dev,
        distance_threshold, angle_threshold, block_normal_equations,
        reduced_normal_equations, A_icp.data(), b_icp.data(), residual, 128, 64,
        cloud_in_hor->points.size());

    // Cast to double for numerical stability during solving
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> dA_icp = A_icp.cast<double>();
    Eigen::Matrix<double, 6, 1> db_icp = b_icp.cast<double>();

    // Solve Ax = -b
    Eigen::Matrix<double, 6, 1> result = dA_icp.ldlt().solve(-db_icp);

    // Update transformation
    Eigen::Vector3f inc_t(result(0), result(1), result(2));
    Eigen::Vector3d inc_ksi(result(3), result(4), result(5));

    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> rt_inc_step =
        Eigen::Matrix<float, 4, 4, Eigen::RowMajor>::Identity();
    rt_inc_step.topLeftCorner<3, 3>() = Rodrigues(inc_ksi).cast<float>();
    rt_inc_step.topRightCorner<3, 1>() = inc_t;

    T_inc_est = rt_inc_step * T_inc_est;
    T_wi_est = T_wi_init * T_inc_est;
  }

  T_world_imu = T_wi_est;
  return true;
}