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

#ifndef SCAN2MESH_ICP_H_
#define SCAN2MESH_ICP_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <iostream>

#include "cuda/internal.h"
#include "mesh_raycaster.h"

class Scan2MeshICP {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Scan2MeshICP(Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& T_imu_vti,
               Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& T_imu_hor,
               const unsigned int vertex_num, const unsigned int face_num,
               vector<float>* vertices, vector<unsigned int>* faces,
               float dist_thresh = 2.0f);

  virtual ~Scan2MeshICP();

  bool EstimatePose(
      Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& T_world_imu,
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_in_hor,
      const int64_t timestamp);

 private:
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Rodrigues(
      const Eigen::Vector3d& src);

  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> T_imu_vti;
  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> T_imu_hor;

  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> R_imu_vti;
  Eigen::Vector3f t_imu_vti;
  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> R_vti_imu;
  Eigen::Vector3f t_vti_imu;

  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> R_imu_hor;
  Eigen::Vector3f t_imu_hor;
  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> R_hor_imu;
  Eigen::Vector3f t_hor_imu;

  unsigned int mesh_vertex_num;
  unsigned int mesh_face_num;
  vector<float>* mesh_vertices;
  vector<unsigned int>* mesh_faces;

  float distance_threshold;
  float angle_threshold;

  DeviceArray<Se3NormalEquation> block_normal_equations;
  DeviceArray<Se3NormalEquation> reduced_normal_equations;

  std::unique_ptr<MeshRaycaster> mesh_raycaster;

  Matrix3f device_R_imu_vti;
  float3 device_t_imu_vti;
  Matrix3f device_R_vti_imu;
  float3 device_t_vti_imu;
  Matrix3f device_R_imu_hor;
  float3 device_t_imu_hor;
  Matrix3f device_R_hor_imu;
  float3 device_t_hor_imu;
};

#endif
