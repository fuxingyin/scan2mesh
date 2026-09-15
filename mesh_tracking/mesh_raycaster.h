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

#ifndef MESH_RAYCASTER_H_
#define MESH_RAYCASTER_H_

#include <embree4/rtcore.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "string.h"

using namespace std;

class MeshRaycaster {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MeshRaycaster(const int vertex_num, const int face_num,
                vector<float>* vertices, vector<unsigned int>* faces);

  virtual ~MeshRaycaster();

  bool RaycastCloud(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input_cloud,
      const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& T_world_sensor,
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr raycast_cloud,
      const std::string& output_name = "");

  void reset();

 private:
  RTCDevice InitializeDevice();

  RTCScene InitializeRaycastingScene(RTCDevice device, int vertex_num,
                                     int face_num, vector<float>* mesh_vertices,
                                     vector<unsigned int>* mesh_faces);

  int vertex_num;
  int face_num;
  vector<float>* vertices;
  vector<unsigned int>* faces;

  RTCDevice device;
  RTCScene scene;
};

#endif
