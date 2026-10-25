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

#include <iostream>

#include "mesh_tracking/mesh_raycaster.h"
#include "utils/ply_reader.h"

int main(int argc, char* argv[]) {
  if (argc < 3 || argc > 4) {
    std::cerr << "Usage: " << argv[0]
              << " <mesh.ply> <input_cloud.ply> [output.ply]\n";
    return 1;
  }
  try {
    unsigned int vertex_count = 0, face_count = 0;
    std::vector<float> vertices;
    std::vector<unsigned int> faces;
    if (!LoadPLYMesh(argv[1], vertex_count, face_count, vertices, faces))
      return 1;
    auto input = std::make_shared<scan2mesh::ColorCloud>();
    if (!LoadPLYCloud(argv[2], *input)) return 1;
    MeshRaycaster raycaster(vertex_count, face_count, &vertices, &faces);
    auto hits = std::make_shared<scan2mesh::NormalCloud>();
    const Eigen::Matrix<float, 4, 4, Eigen::RowMajor> pose =
        Eigen::Matrix4f::Identity();
    if (!raycaster.RaycastCloud(input, pose, hits)) return 1;
    scan2mesh::ColorCloud output;
    for (const auto& hit : hits->points) {
      if (hit.normal_x == 0 && hit.normal_y == 0 && hit.normal_z == 0) continue;
      scan2mesh::PointXYZRGB point;
      point.x = hit.x;
      point.y = hit.y;
      point.z = hit.z;
      point.r = hit.r;
      point.g = hit.g;
      point.b = hit.b;
      output.push_back(point);
    }
    const std::string path = argc == 4 ? argv[3] : "casted_points.ply";
    if (!SavePLYCloud(path, output)) return 1;
    std::cout << "Saved " << output.size() << " raycast hits to " << path
              << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
