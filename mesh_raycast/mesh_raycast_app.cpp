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
#include "utils/ply_reader.h"

using namespace std;

/*
 * We will register this error handler with the device in InitializeDevice(),
 * so that we are automatically informed on errors.
 * This is extremely helpful for finding bugs in your code, prevents you
 * from having to add explicit error checking to each Embree API call.
 */
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
RTCDevice InitializeDevice() {
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
RTCScene InitializeScene(RTCDevice device) {
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
      geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float), 3);

  unsigned* indices = (unsigned*)rtcSetNewGeometryBuffer(
      geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned),
      1);

  if (vertices && indices) {
    vertices[0] = 0.f;
    vertices[1] = 0.f;
    vertices[2] = 0.f;
    vertices[3] = 1.f;
    vertices[4] = 0.f;
    vertices[5] = 0.f;
    vertices[6] = 0.f;
    vertices[7] = 1.f;
    vertices[8] = 0.f;

    indices[0] = 0;
    indices[1] = 1;
    indices[2] = 2;
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

/*
 * Create a scene, which is a collection of geometry objects. Scenes are
 * what the intersect / occluded functions work on. You can think of a
 * scene as an acceleration structure, e.g. a bounding-volume hierarchy.
 *
 * Scenes, like devices, are reference-counted.
 */
RTCScene InitializeRaycastingScene(RTCDevice device, int vertex_num,
                                   int face_num, vector<float>& mesh_vertices,
                                   vector<unsigned int>& mesh_faces) {
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
    // vertices[0] = 0.f; vertices[1] = 0.f; vertices[2] = 0.f;
    // vertices[3] = 1.f; vertices[4] = 0.f; vertices[5] = 0.f;
    // vertices[6] = 0.f; vertices[7] = 1.f; vertices[8] = 0.f;
    // indices[0] = 0; indices[1] = 1; indices[2] = 2;
    memcpy(vertices, mesh_vertices.data(), 3 * vertex_num * sizeof(float));
    memcpy(indices, mesh_faces.data(), 3 * face_num * sizeof(unsigned int));
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

/*
 * Cast a single ray with origin (ox, oy, oz) and direction
 * (dx, dy, dz).
 */
void castRay(RTCScene scene, float ox, float oy, float oz, float dx, float dy,
             float dz) {
  /*
   * The ray hit structure holds both the ray and the hit.
   * The user must initialize it properly -- see API documentation
   * for rtcIntersect1() for details.
   */
  struct RTCRayHit rayhit;
  rayhit.ray.org_x = ox;
  rayhit.ray.org_y = oy;
  rayhit.ray.org_z = oz;
  rayhit.ray.dir_x = dx;
  rayhit.ray.dir_y = dy;
  rayhit.ray.dir_z = dz;
  rayhit.ray.tnear = 0;
  rayhit.ray.tfar = std::numeric_limits<float>::infinity();
  rayhit.ray.mask = -1;
  rayhit.ray.flags = 0;
  rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
  rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

  /*
   * There are multiple variants of rtcIntersect. This one
   * intersects a single ray with the scene.
   */
  rtcIntersect1(scene, &rayhit);

  printf("%f, %f, %f: ", ox, oy, oz);
  if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
    /* Note how geomID and primID identify the geometry we just hit.
     * We could use them here to interpolate geometry information,
     * compute shading, etc.
     * Since there is only a single triangle in this scene, we will
     * get geomID=0 / primID=0 for all hits.
     * There is also instID, used for instancing. See
     * the instancing tutorials for more information */
    printf("Found intersection on geometry %d, primitive %d at tfar=%f\n",
           rayhit.hit.geomID, rayhit.hit.primID, rayhit.ray.tfar);
  } else
    printf("Did not find any intersection.\n");
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

int main(int argc, char* argv[]) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <mesh.ply> <input_cloud.ply>"
              << std::endl;
    return -1;
  }

  const std::string mesh_path = argv[1];
  const std::string cloud_path = argv[2];
  unsigned int vertex_num = 0;
  unsigned int face_num = 0;
  std::vector<float> vertices;
  std::vector<unsigned int> faces;

  LoadPLYMesh(mesh_path, vertex_num, face_num, vertices, faces);

  std::cout << "Input mesh: " << mesh_path << std::endl;
  std::cout << "Vertices: " << vertex_num << ", Faces: " << face_num
            << std::endl;

  RTCDevice device = InitializeDevice();
  RTCScene scene =
      InitializeRaycastingScene(device, vertex_num, face_num, vertices, faces);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_input(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  if (pcl::io::loadPLYFile(cloud_path, *cloud_input) != 0) {
    std::cerr << "Failed to load point cloud: " << cloud_path << std::endl;

    rtcReleaseScene(scene);
    rtcReleaseDevice(device);
    return -1;
  }

  std::cout << "Input cloud: " << cloud_path << std::endl;
  std::cout << "Input points: " << cloud_input->size() << std::endl;

  Eigen::Isometry3f T_world_imu = Eigen::Isometry3f::Identity();
  const Eigen::Matrix3f R_world_imu = T_world_imu.rotation();
  const Eigen::Vector3f t_world_imu = T_world_imu.translation();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cast_results(
      new pcl::PointCloud<pcl::PointXYZ>);
  cast_results->reserve(cloud_input->size());
  const auto begin = std::chrono::steady_clock::now();
  for (const auto& cloud_point : cloud_input->points) {
    const Eigen::Vector3f point(cloud_point.x, cloud_point.y, cloud_point.z);
    const float range = point.norm();
    if (range < 1e-6f) {
      continue;
    }

    const Eigen::Vector3f dir_imu = point / range;
    const Eigen::Vector3f dir_world = R_world_imu * dir_imu;

    RTCRayHit rayhit{};

    rayhit.ray.org_x = t_world_imu.x();
    rayhit.ray.org_y = t_world_imu.y();
    rayhit.ray.org_z = t_world_imu.z();

    rayhit.ray.dir_x = dir_world.x();
    rayhit.ray.dir_y = dir_world.y();
    rayhit.ray.dir_z = dir_world.z();

    rayhit.ray.tnear = 0.0f;
    rayhit.ray.tfar = std::numeric_limits<float>::infinity();
    rayhit.ray.mask = -1;
    rayhit.ray.flags = 0;

    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

    rtcIntersect1(scene, &rayhit);

    if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
      continue;
    }

    const Eigen::Vector3f hit_point_imu = dir_imu * rayhit.ray.tfar;

    pcl::PointXYZ hit_point;
    hit_point.x = hit_point_imu.x();
    hit_point.y = hit_point_imu.y();
    hit_point.z = hit_point_imu.z();

    cast_results->push_back(hit_point);
  }

  const auto end = std::chrono::steady_clock::now();
  const double elapsed_ms =
      std::chrono::duration<double, std::milli>(end - begin).count();

  std::cout << "Raycasting time: " << elapsed_ms << " ms" << std::endl;
  std::cout << "Casted points: " << cast_results->size() << std::endl;

  pcl::io::savePLYFileBinary("/home/fxy/casted_points.ply", *cast_results);

  rtcReleaseScene(scene);
  rtcReleaseDevice(device);

  return 0;
}