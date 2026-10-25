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

#include "utils/eigen_utils.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "mesh_raycaster.h"

namespace {
void CheckEmbree(RTCDevice device) {
  const RTCError error = rtcGetDeviceError(device);
  if (error != RTC_ERROR_NONE)
    throw std::runtime_error("Embree error " +
                             std::to_string(static_cast<int>(error)));
}
}  // namespace

MeshRaycaster::MeshRaycaster(unsigned int vertex_num, unsigned int face_num,
                             const std::vector<float>* vertices,
                             const std::vector<unsigned int>* faces) {
  if (!vertices || !faces || !vertex_num || !face_num ||
      vertices->size() != static_cast<std::size_t>(vertex_num) * 3 ||
      faces->size() != static_cast<std::size_t>(face_num) * 3)
    throw std::invalid_argument("Invalid raycasting mesh buffers");
  for (const float value : *vertices)
    if (!std::isfinite(value))
      throw std::invalid_argument("Non-finite mesh vertex");
  for (const unsigned int index : *faces)
    if (index >= vertex_num)
      throw std::invalid_argument("Mesh index out of range");
  device = rtcNewDevice(nullptr);
  if (!device) throw std::runtime_error("Cannot create Embree device");
  RTCGeometry geometry = nullptr;
  try {
    scene = rtcNewScene(device);
    CheckEmbree(device);
    geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
    CheckEmbree(device);
    auto* vertex_buffer = rtcSetNewGeometryBuffer(
        geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
        3 * sizeof(float), vertex_num);
    auto* index_buffer = rtcSetNewGeometryBuffer(
        geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
        3 * sizeof(unsigned int), face_num);
    CheckEmbree(device);
    if (!vertex_buffer || !index_buffer)
      throw std::runtime_error("Cannot allocate Embree buffers");
    std::memcpy(vertex_buffer, vertices->data(),
                vertices->size() * sizeof(float));
    std::memcpy(index_buffer, faces->data(),
                faces->size() * sizeof(unsigned int));
    rtcCommitGeometry(geometry);
    rtcAttachGeometry(scene, geometry);
    CheckEmbree(device);
    rtcReleaseGeometry(geometry);
    geometry = nullptr;
    rtcCommitScene(scene);
    CheckEmbree(device);
  } catch (...) {
    if (geometry) rtcReleaseGeometry(geometry);
    if (scene) rtcReleaseScene(scene);
    rtcReleaseDevice(device);
    throw;
  }
}

MeshRaycaster::~MeshRaycaster() {
  rtcReleaseScene(scene);
  rtcReleaseDevice(device);
}

bool MeshRaycaster::RaycastCloud(
    const scan2mesh::ColorCloud::ConstPtr& input_cloud,
    const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& T_world_sensor,
    scan2mesh::NormalCloud::Ptr raycast_cloud,
    const std::string& /*output_name*/) {
  if (!input_cloud || !raycast_cloud) {
    std::cerr << "Invalid input or output point cloud." << std::endl;
    return false;
  }

  raycast_cloud->clear();
  if (!scan2mesh::AllFinite(T_world_sensor)) return false;
  const Eigen::Matrix3f R_world_sensor = T_world_sensor.topLeftCorner<3, 3>();
  const Eigen::Vector3f t_world_sensor = T_world_sensor.topRightCorner<3, 1>();

  raycast_cloud->clear();
  raycast_cloud->reserve(input_cloud->size());

  for (const auto& input_point : input_cloud->points) {
    const Eigen::Vector3f point_sensor(input_point.x, input_point.y,
                                       input_point.z);

    const float range = point_sensor.norm();

    scan2mesh::PointXYZRGBNormal output_point{};
    if (!scan2mesh::AllFinite(point_sensor) || !std::isfinite(range) ||
        range <= std::numeric_limits<float>::epsilon()) {
      raycast_cloud->push_back(output_point);
      continue;
    }

    const Eigen::Vector3f ray_direction_sensor = point_sensor / range;

    const Eigen::Vector3f ray_direction_world =
        R_world_sensor * ray_direction_sensor;

    RTCRayHit ray_hit{};

    ray_hit.ray.org_x = t_world_sensor.x();
    ray_hit.ray.org_y = t_world_sensor.y();
    ray_hit.ray.org_z = t_world_sensor.z();

    ray_hit.ray.dir_x = ray_direction_world.x();
    ray_hit.ray.dir_y = ray_direction_world.y();
    ray_hit.ray.dir_z = ray_direction_world.z();

    ray_hit.ray.tnear = 0.0f;
    ray_hit.ray.tfar = std::numeric_limits<float>::infinity();
    ray_hit.ray.mask = RTC_INVALID_GEOMETRY_ID;
    ray_hit.ray.flags = 0;

    ray_hit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    ray_hit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

    rtcIntersect1(scene, &ray_hit, nullptr);

    if (ray_hit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
      const Eigen::Vector3f intersection_point_world =
          t_world_sensor + ray_direction_world * ray_hit.ray.tfar;

      Eigen::Vector3f surface_normal_world(ray_hit.hit.Ng_x, ray_hit.hit.Ng_y,
                                           ray_hit.hit.Ng_z);

      const float normal_length = surface_normal_world.norm();
      if (normal_length > std::numeric_limits<float>::epsilon()) {
        surface_normal_world /= normal_length;
      }

      output_point.x = intersection_point_world.x();
      output_point.y = intersection_point_world.y();
      output_point.z = intersection_point_world.z();

      output_point.r = 0;
      output_point.g = 0;
      output_point.b = 255;

      output_point.normal_x = surface_normal_world.x();
      output_point.normal_y = surface_normal_world.y();
      output_point.normal_z = surface_normal_world.z();
    }

    raycast_cloud->push_back(output_point);
  }

  return true;
}
