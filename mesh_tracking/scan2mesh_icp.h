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

#include <cstdint>
#include <memory>

#include "mesh_raycaster.h"
#ifdef SCAN2MESH_USE_CUDA
#include "cuda/internal.h"
#endif

class Scan2MeshICP {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Pose = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;
  Scan2MeshICP(const Pose& T_imu_vti, const Pose& T_imu_hor,
               unsigned int vertex_num, unsigned int face_num,
               std::vector<float>* vertices, std::vector<unsigned int>* faces,
               float dist_thresh = 2.0f);
  ~Scan2MeshICP();

  // Input points stay in the horizontal LiDAR frame. On failure pose is
  // unchanged.
  bool EstimatePose(Pose& T_world_imu,
                    const scan2mesh::ColorCloud::ConstPtr& cloud,
                    int64_t timestamp);

 private:
  Pose T_imu_hor;
  float distance_threshold;
  std::unique_ptr<MeshRaycaster> mesh_raycaster;
#ifdef SCAN2MESH_USE_CUDA
  DeviceArray<Se3NormalEquation> block_normal_equations;
  DeviceArray<Se3NormalEquation> reduced_normal_equations;
#endif
};
#endif
