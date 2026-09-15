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

#ifndef TRACKER_INTERFACE_H_
#define TRACKER_INTERFACE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/thread/mutex.hpp>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "mesh_raycaster.h"
#include "scan2mesh_icp.h"
#include "shared_states.h"
#include "utils/cloud_render_data.h"
#include "utils/thread_safe_value.h"

class TrackerInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct RaycastMesh {
    unsigned int vertex_num = 0;
    unsigned int face_num = 0;

    std::vector<float>* vertices = nullptr;
    std::vector<unsigned int>* faces = nullptr;
  };

  explicit TrackerInterface(const std::string& data_path,
                            const std::string& mesh_path,
                            const std::string& urdf_path);

  ~TrackerInterface() = default;

  TrackerInterface(const TrackerInterface&) = delete;
  TrackerInterface& operator=(const TrackerInterface&) = delete;

  bool RunTracking();

  RaycastMesh* GetRaycastTargetMesh() { return raycast_mesh_data.get(); }

  const RaycastMesh* GetRaycastTargetMesh() const {
    return raycast_mesh_data.get();
  }

  CloudRenderData* GetLiveVerticalCloud() { return live_cloud_vertical.get(); }
  CloudRenderData* GetLiveHorizontalCloud() {
    return live_cloud_horizontal.get();
  }
  CloudRenderData* GetRaycastVerticalCloud() {
    return raycasted_cloud_vertical.get();
  }
  CloudRenderData* GetRaycastHorizontalCloud() {
    return raycasted_cloud_horizontal.get();
  }

  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> GetLastTrackedPose() const {
    return last_tracked_pose;
  }

  ThreadSafeValue<bool> end_requested{false};

  bool live_available = false;
  boost::mutex live_mutex;

  bool raycast_available = false;
  boost::mutex raycast_mutex;

 private:
  using Matrix4fRowMajor = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;

  using TrajectoryMap = std::map<
      uint64_t, Eigen::Isometry3f, std::less<uint64_t>,
      Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Isometry3f>>>;

  void InitializeMesh();

  void InitializeSensorTransforms();

  void InitializeTrackingComponents();

  void LoadTrajectory();

  void SplitLidarCloud(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& vertical_cloud,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& horizontal_cloud);

  void FilterHorizontalCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

  void EstimatePose(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                    uint64_t lidar_time);

  void ValidatePose(uint64_t lidar_time);

  void GenerateRaycastClouds(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& horizontal_cloud);

  void UpdateVisualization();

  void UpdateLiveCloudData();

  void UpdateRaycastCloudData();

 private:
  size_t current_frame = 0;
  bool first_run = true;
  std::string data_path;
  std::string mesh_path;
  std::string urdf_path;
  SharedState& shared_state;

  // Sensor extrinsic transforms.
  Matrix4fRowMajor T_imu_vti;
  Matrix4fRowMajor T_imu_hor;
  Matrix4fRowMajor T_vti_imu;
  Matrix4fRowMajor T_hor_imu;

  // Tracking trajectory.
  std::vector<uint64_t> tracking_timestamps;
  TrajectoryMap tracking_trajectory;

  // Registration mesh.
  unsigned int vertex_num = 0;
  unsigned int face_num = 0;
  std::vector<float> vertices;
  std::vector<unsigned int> faces;

  std::unique_ptr<RaycastMesh> raycast_mesh_data;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_input_cloud_vertical;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_input_cloud_horizontal;

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr last_raycasted_cloud_vertical;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr last_raycasted_cloud_horizontal;

  // Visualization data.
  std::unique_ptr<CloudRenderData> live_cloud_vertical;
  std::unique_ptr<CloudRenderData> live_cloud_horizontal;
  std::unique_ptr<CloudRenderData> raycasted_cloud_vertical;
  std::unique_ptr<CloudRenderData> raycasted_cloud_horizontal;

  // Current poses.
  Matrix4fRowMajor T_world_imu;
  Matrix4fRowMajor T_world_vti;
  Matrix4fRowMajor T_world_hor;

  Matrix4fRowMajor last_tracked_pose;

  // Tracking components.
  std::unique_ptr<MeshRaycaster> raycaster;
  std::unique_ptr<Scan2MeshICP> scan2mesh_tracker;
};

#endif  // TRACKER_INTERFACE_H_