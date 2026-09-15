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

#include "tracker_interface.h"

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "cuda/internal.h"
#include "utils/config_args.h"
#include "utils/ply_reader.h"
#include "utils/urdf_loader.h"
#include "voxel_filter.h"

constexpr float kVoxelSize = 0.2f;
constexpr float kTranslationThreshold = 0.3f;
constexpr float kRotationThresholdDeg = 15.0f;
constexpr size_t kTrajectoryCheckFrameLimit = 3000;

TrackerInterface::TrackerInterface(const std::string& data_path,
                                   const std::string& mesh_path,
                                   const std::string& urdf_path)
    : data_path(data_path),
      mesh_path(mesh_path),
      urdf_path(urdf_path),
      shared_state(SharedState::get()) {
  InitializeMesh();
  InitializeSensorTransforms();
  InitializeTrackingComponents();
  LoadTrajectory();
}

void TrackerInterface::InitializeMesh() {
  if (!LoadPLYMesh(mesh_path, vertex_num, face_num, vertices, faces)) {
    throw std::runtime_error("Failed to load mesh: " + mesh_path);
  }

  std::cout << "Loaded mesh: " << vertex_num << " vertices, " << face_num
            << " faces." << std::endl;

  raycast_mesh_data.reset(new RaycastMesh());

  raycast_mesh_data->vertex_num = vertex_num;
  raycast_mesh_data->face_num = face_num;
  raycast_mesh_data->vertices = &vertices;
  raycast_mesh_data->faces = &faces;
}

void TrackerInterface::InitializeSensorTransforms() {
  Eigen::Matrix4f T_imu_horizontal = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f T_imu_vertical = Eigen::Matrix4f::Identity();
  if (!URDFLoader::LoadLinkTransform(
          urdf_path, "imu_link", "horizontal_vlp16_link", &T_imu_horizontal)) {
    throw std::runtime_error(
        "Failed to load IMU -> horizontal LiDAR transform from URDF.");
  }
  if (!URDFLoader::LoadLinkTransform(urdf_path, "imu_link",
                                     "vertical_vlp16_link", &T_imu_vertical)) {
    throw std::runtime_error(
        "Failed to load IMU -> vertical LiDAR transform from URDF.");
  }

  T_imu_vti = T_imu_vertical;
  T_imu_hor = T_imu_horizontal;

  T_vti_imu = T_imu_vti.inverse();
  T_hor_imu = T_imu_hor.inverse();

  T_world_imu.setIdentity();
  T_world_vti = T_world_imu * T_imu_vti;
  T_world_hor = T_world_imu * T_imu_hor;
}

void TrackerInterface::InitializeTrackingComponents() {
  raycaster.reset(new MeshRaycaster(
      raycast_mesh_data->vertex_num, raycast_mesh_data->face_num,
      raycast_mesh_data->vertices, raycast_mesh_data->faces));

  scan2mesh_tracker.reset(
      new Scan2MeshICP(T_imu_vti, T_imu_hor, raycast_mesh_data->vertex_num,
                       raycast_mesh_data->face_num, raycast_mesh_data->vertices,
                       raycast_mesh_data->faces));
}

void TrackerInterface::LoadTrajectory() {
  const std::string trajectory_file = data_path + "/trajectory.poses";
  std::ifstream file(trajectory_file);

  if (!file.is_open()) {
    std::cerr << "Failed to open trajectory file: " << trajectory_file
              << std::endl;
    return;
  }

  tracking_trajectory.clear();
  tracking_timestamps.clear();

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    std::stringstream stream(line);
    int64_t timestamp = 0;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;
    if (!(stream >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw)) {
      std::cerr << "Failed to parse trajectory line: " << line << std::endl;
      continue;
    }

    const Eigen::Quaternionf rotation(qw, qx, qy, qz);
    const Eigen::Vector3f translation(x, y, z);

    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.translate(translation);
    pose.rotate(rotation);

    const uint64_t timestamp_us = static_cast<uint64_t>(timestamp);
    tracking_trajectory[timestamp_us] = pose;
    tracking_timestamps.push_back(timestamp_us);
  }

  std::cout << "Loaded " << tracking_timestamps.size() << " trajectory poses."
            << std::endl;
}

bool TrackerInterface::RunTracking() {
  while (!end_requested.Get()) {
    if (shared_state.pause_tracking.Get()) {
      usleep(1000);
      continue;
    }

    if (first_run) {
      cudaSafeCall(cudaSetDevice(ConfigArgs::get().gpu_id));
      first_run = false;
    }

    if (current_frame >= tracking_timestamps.size()) {
      return true;
    }

    const uint64_t lidar_time = tracking_timestamps[current_frame];
    const std::string cloud_file =
        data_path + "/ply_folder/" + std::to_string(lidar_time) + ".ply";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile(cloud_file, *input_cloud) < 0) {
      std::cerr << "Failed to load point cloud: " << cloud_file << std::endl;
      return false;
    }

    if (input_cloud->empty()) {
      std::cerr << "Point cloud is empty: " << cloud_file << std::endl;
      return false;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr horizontal_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    vertical_cloud->reserve(input_cloud->size());
    horizontal_cloud->reserve(input_cloud->size());

    SplitLidarCloud(input_cloud, vertical_cloud, horizontal_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr horizontal_cloud_original(
        new pcl::PointCloud<pcl::PointXYZRGB>(*horizontal_cloud));

    FilterHorizontalCloud(horizontal_cloud);

    pcl::transformPointCloud(*vertical_cloud, *vertical_cloud, T_vti_imu);
    pcl::transformPointCloud(*horizontal_cloud, *horizontal_cloud, T_hor_imu);

    EstimatePose(horizontal_cloud, lidar_time);

    ValidatePose(lidar_time);

    // pcl::transformPointCloud(*horizontal_cloud_original,
    //                          *horizontal_cloud_original, T_hor_imu);
    GenerateRaycastClouds(horizontal_cloud);

    last_tracked_pose = T_world_imu;

    last_input_cloud_vertical = vertical_cloud;
    last_input_cloud_horizontal = horizontal_cloud;

    UpdateVisualization();

    shared_state.frame_id.SetAndNotifyAll(current_frame);

    ++current_frame;
  }

  return true;
}

void TrackerInterface::SplitLidarCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& vertical_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& horizontal_cloud) {
  for (const auto& point : input_cloud->points) {
    pcl::PointXYZRGB colored_point = point;
    if (point.g == 0) {
      colored_point.r = 0;
      colored_point.g = 210;
      colored_point.b = 255;
      vertical_cloud->push_back(colored_point);
    } else {
      colored_point.r = 255;
      colored_point.g = 100;
      colored_point.b = 30;
      horizontal_cloud->push_back(colored_point);
    }
  }
}

void TrackerInterface::FilterHorizontalCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
  AdaptiveVoxelFilterOptions options;
  options.max_length = 0.5f;
  options.min_num_points = 2000;
  *cloud = AdaptiveVoxelFilter(*cloud, options);
}

void TrackerInterface::EstimatePose(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, uint64_t lidar_time) {
  const auto start = std::chrono::high_resolution_clock::now();

  scan2mesh_tracker->EstimatePose(T_world_imu, cloud, lidar_time);

  const auto end = std::chrono::high_resolution_clock::now();
  const double elapsed_ms =
      std::chrono::duration<double, std::milli>(end - start).count();
  std::cout << "ICP pose estimation time: " << elapsed_ms << " ms" << std::endl;
}

void TrackerInterface::ValidatePose(uint64_t lidar_time) {
  const auto trajectory_it = tracking_trajectory.find(lidar_time);
  if (trajectory_it == tracking_trajectory.end()) {
    std::cerr << "Cannot find timestamp " << lidar_time
              << " in reference trajectory." << std::endl;

    return;
  }

  if (current_frame >= kTrajectoryCheckFrameLimit) {
    return;
  }

  const Eigen::Matrix4f T_reference = trajectory_it->second.matrix();
  const Eigen::Matrix4f T_difference = T_reference.inverse() * T_world_imu;
  const float translation_error = T_difference.block<3, 1>(0, 3).norm();

  const Eigen::Matrix3f rotation_difference = T_difference.block<3, 3>(0, 0);
  const Eigen::AngleAxisf angle_axis(rotation_difference);
  const float rotation_error_deg =
      std::abs(angle_axis.angle()) * 180.0f / static_cast<float>(M_PI);

  if (translation_error <= kTranslationThreshold &&
      rotation_error_deg <= kRotationThresholdDeg) {
    return;
  }

  std::cerr << "\n"
            << "========================================\n"
            << "WARNING: Pose difference is too large!\n"
            << "Timestamp: " << lidar_time << '\n'
            << "Translation error: " << translation_error << " m\n"
            << "Rotation error: " << rotation_error_deg << " deg\n"
            << "========================================\n";

  T_world_imu = T_reference;
}

void TrackerInterface::GenerateRaycastClouds(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& horizontal_cloud) {
  T_world_hor = T_world_imu * T_imu_hor;

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr horizontal_raycast(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  raycaster->RaycastCloud(horizontal_cloud, T_world_hor, horizontal_raycast);

  // last_raycasted_cloud_vertical = vertical_raycast;
  last_raycasted_cloud_horizontal = horizontal_raycast;
}

void TrackerInterface::UpdateVisualization() {
  UpdateLiveCloudData();
  UpdateRaycastCloudData();
}

void TrackerInterface::UpdateLiveCloudData() {
  boost::mutex::scoped_lock lock(live_mutex);

  live_available = true;

  live_cloud_vertical.reset(
      new CloudRenderData(T_world_vti, last_input_cloud_vertical));
  live_cloud_horizontal.reset(
      new CloudRenderData(T_world_hor, last_input_cloud_horizontal));
}

void TrackerInterface::UpdateRaycastCloudData() {
  boost::mutex::scoped_lock lock(raycast_mutex);

  raycast_available = true;

  const Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();

  raycasted_cloud_vertical.reset(
      new CloudRenderData(identity, last_raycasted_cloud_vertical));
  raycasted_cloud_horizontal.reset(
      new CloudRenderData(identity, last_raycasted_cloud_horizontal));
}
