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

#include "mesh_raycaster.h"

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

MeshRaycaster::MeshRaycaster(const int vertex_num, const int face_num,
                             vector<float>* vertices,
                             vector<unsigned int>* faces)
    : vertex_num(vertex_num),
      face_num(face_num),
      vertices(vertices),
      faces(faces) {
  device = InitializeDevice();
  scene =
      InitializeRaycastingScene(device, vertex_num, face_num, vertices, faces);
}

MeshRaycaster::~MeshRaycaster() {
  /* Though not strictly necessary in this example, you should
   * always make sure to release resources allocated through Embree. */
  rtcReleaseScene(scene);
  rtcReleaseDevice(device);
}

void errorFunction(void* userPtr, enum RTCError error, const char* str) {
  printf("error %d: %s\n", error, str);
}

/*
 * Embree has a notion of devices, which are entities that can run
 * raytracing kernels.
 * We initialize our device here, and then register the error handler so that
 * we don't miss any errors.
 *
 * rtcNewDevice() takes a configuration string as an argument. See the API docs
 * for more information.
 *
 * Note that RTCDevice is reference-counted.
 */
RTCDevice MeshRaycaster::InitializeDevice() {
  RTCDevice device = rtcNewDevice(NULL);

  if (!device)
    printf("error %d: cannot create device\n", rtcGetDeviceError(NULL));

  rtcSetDeviceErrorFunction(device, errorFunction, NULL);
  return device;
}

/*
 * Create a scene, which is a collection of geometry objects. Scenes are
 * what the intersect / occluded functions work on. You can think of a
 * scene as an acceleration structure, e.g. a bounding-volume hierarchy.
 *
 * Scenes, like devices, are reference-counted.
 */
RTCScene MeshRaycaster::InitializeRaycastingScene(
    RTCDevice device, int vertex_num, int face_num,
    vector<float>* mesh_vertices, vector<unsigned int>* mesh_faces) {
  RTCScene scene = rtcNewScene(device);

  /*
   * Create a triangle mesh geometry, and initialize a single triangle.
   * You can look up geometry types in the API documentation to
   * find out which type expects which buffers.
   *
   * We create buffers directly on the device, but you can also use
   * shared buffers. For shared buffers, special care must be taken
   * to ensure proper alignment and padding. This is described in
   * more detail in the API documentation.
   */
  RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
  float* vertices = (float*)rtcSetNewGeometryBuffer(
      geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float),
      vertex_num);

  unsigned* indices = (unsigned*)rtcSetNewGeometryBuffer(
      geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned),
      face_num);

  if (vertices && indices) {
    memcpy(vertices, mesh_vertices->data(), 3 * vertex_num * sizeof(float));
    memcpy(indices, mesh_faces->data(), 3 * face_num * sizeof(unsigned int));
  }

  /*
   * You must commit geometry objects when you are done setting them up,
   * or you will not get any intersections.
   */
  rtcCommitGeometry(geom);

  /*
   * In rtcAttachGeometry(...), the scene takes ownership of the geom
   * by increasing its reference count. This means that we don't have
   * to hold on to the geom handle, and may release it. The geom object
   * will be released automatically when the scene is destroyed.
   *
   * rtcAttachGeometry() returns a geometry ID. We could use this to
   * identify intersected objects later on.
   */
  rtcAttachGeometry(scene, geom);
  rtcReleaseGeometry(geom);

  /*
   * Like geometry objects, scenes must be committed. This lets
   * Embree know that it may start building an acceleration structure.
   */
  rtcCommitScene(scene);

  return scene;
}

void waitForKeyPressedUnderWindows() {
#if defined(_WIN32)
  HANDLE hStdOutput = GetStdHandle(STD_OUTPUT_HANDLE);

  CONSOLE_SCREEN_BUFFER_INFO csbi;
  if (!GetConsoleScreenBufferInfo(hStdOutput, &csbi)) {
    printf("GetConsoleScreenBufferInfo failed: %lu\n", GetLastError());
    return;
  }

  /* do not pause when running on a shell */
  if (csbi.dwCursorPosition.X != 0 || csbi.dwCursorPosition.Y != 0) return;

  /* only pause if running in separate console window. */
  printf("\n\tPress any key to exit...\n");
  _getch();
#endif
}

bool MeshRaycaster::RaycastCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input_cloud,
    const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& T_world_sensor,
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr raycast_cloud,
    const std::string& output_name) {
  if (!input_cloud || !raycast_cloud) {
    std::cerr << "Invalid input or output point cloud." << std::endl;
    return false;
  }

  const Eigen::Matrix3f R_world_sensor = T_world_sensor.topLeftCorner<3, 3>();
  const Eigen::Vector3f t_world_sensor = T_world_sensor.topRightCorner<3, 1>();

  raycast_cloud->clear();
  raycast_cloud->reserve(input_cloud->size());

  for (const auto& input_point : input_cloud->points) {
    const Eigen::Vector3f point_sensor(input_point.x, input_point.y,
                                       input_point.z);

    const float range = point_sensor.norm();

    pcl::PointXYZRGBNormal output_point{};
    if (range <= std::numeric_limits<float>::epsilon()) {
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

    rtcIntersect1(scene, &ray_hit);

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
