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

#ifndef UTILS_PANGO_CLOUD_H_
#define UTILS_PANGO_CLOUD_H_

#include <pangolin/gl/gl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>

class PointCloudRenderer {
 public:
  PointCloudRenderer(const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
      : point_count(cloud.size()),
        color_offset(offsetof(pcl::PointXYZRGB, rgb)),
        point_stride(sizeof(pcl::PointXYZRGB)),
        vertex_buffer(0) {
    CreateBuffer(cloud);
  }

  PointCloudRenderer(const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud)
      : point_count(cloud.size()),
        color_offset(offsetof(pcl::PointXYZRGBNormal, rgb)),
        point_stride(sizeof(pcl::PointXYZRGBNormal)),
        vertex_buffer(0) {
    CreateBuffer(cloud);
  }

  ~PointCloudRenderer() { ReleaseBuffer(); }

  PointCloudRenderer(const PointCloudRenderer&) = delete;
  PointCloudRenderer& operator=(const PointCloudRenderer&) = delete;

  void DrawPoints(float point_size = 6.0f) const {
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);

    glVertexPointer(3, GL_FLOAT, static_cast<GLsizei>(point_stride), nullptr);

    glColorPointer(3, GL_UNSIGNED_BYTE, static_cast<GLsizei>(point_stride),
                   reinterpret_cast<const void*>(color_offset));

    glPointSize(point_size);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(point_count));

    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
  }

  std::size_t PointCount() const { return point_count; }

 private:
  template <typename PointType>
  void CreateBuffer(const pcl::PointCloud<PointType>& cloud) {
    glGenBuffers(1, &vertex_buffer);

    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);

    glBufferData(GL_ARRAY_BUFFER, cloud.points.size() * sizeof(PointType),
                 cloud.points.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
  }

  void ReleaseBuffer() {
    if (vertex_buffer != 0) {
      glDeleteBuffers(1, &vertex_buffer);
      vertex_buffer = 0;
    }
  }

 private:
  std::size_t point_count;

  std::size_t color_offset;
  std::size_t point_stride;

  GLuint vertex_buffer;
};

#endif  // UTILS_PANGO_CLOUD_H_