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

#ifndef CloudRenderData_H_
#define CloudRenderData_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

class CloudRenderData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Eigen::Matrix4f Transform;

  typedef pcl::PointCloud<pcl::PointXYZRGB> RGBPointCloud;
  typedef pcl::PointCloud<pcl::PointXYZRGBNormal> RGBNormalPointCloud;

  CloudRenderData(const Transform& transform, const RGBPointCloud::Ptr& cloud)
      : transform(transform), cloud(cloud) {}

  CloudRenderData(const Transform& transform,
                  const RGBNormalPointCloud::Ptr& cloud)
      : transform(transform), cloud_with_normal(cloud) {}

  ~CloudRenderData() = default;

  Transform transform;

  RGBPointCloud::Ptr cloud;
  RGBNormalPointCloud::Ptr cloud_with_normal;
};

#endif
