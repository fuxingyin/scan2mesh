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

#ifndef MESH_TRACKING_VISUALIZER_H_
#define MESH_TRACKING_VISUALIZER_H_

#include <pangolin/pangolin.h>

#include "shared_states.h"
#include "tracker_interface.h"
#include "utils/mesh_renderer.h"
#include "utils/point_cloud_renderer.h"

using namespace std;

class MeshTrackingVisualizer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MeshTrackingVisualizer();

  virtual ~MeshTrackingVisualizer();

  bool Run();

  void reset();

  void SetTracker(TrackerInterface* interface);

 private:
  void SetupLighting();

  void DrawLidarPose(const Eigen::Matrix4f& lidar_pose, float axis_length);
  void DrawLidarBox(const Eigen::Matrix4f& lidar_pose, float axis_length);

  void Render();

  void UpdateFollowCamera();

  void HandleInput();

  void UpdateLiveCloudDisplay();

  void UpdateCastCloudDisplay();

  pangolin::OpenGlRenderState s_cam;

  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> trackered_pose;

  std::unique_ptr<PointCloudRenderer> live_cloud;
  std::unique_ptr<PointCloudRenderer> raycast_cloud;
  std::unique_ptr<MeshRenderer> cast_surface;

  pangolin::Var<bool> pause;

  pangolin::Var<bool> follow_pose;
  pangolin::Var<bool> draw_vertical_cloud;
  pangolin::Var<bool> draw_horizontal_cloud;
  // pangolin::Var<bool> draw_raycast_vertical_cloud;
  pangolin::Var<bool> draw_raycast_horizontal_cloud;
  pangolin::Var<bool> draw_surface;
  pangolin::Var<bool> draw_wireframe;
  pangolin::Var<float> display_height;

  pangolin::Var<std::string> frame;
  pangolin::Var<std::string> status;

  TrackerInterface* trackerInterface;
  SharedState& threadPack;

  Eigen::Matrix4f vertical_pose;
  Eigen::Matrix4f horizontal_pose;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr live_vertical_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr live_horizontal_cloud;

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr raycasted_cloud_vertical;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr raycasted_cloud_horizontal;
};

#endif
