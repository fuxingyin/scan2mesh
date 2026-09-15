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

#include "mesh_tracking_visualizer.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

MeshTrackingVisualizer::MeshTrackingVisualizer()
    : live_cloud(nullptr),
      raycast_cloud(nullptr),
      cast_surface(nullptr),
      pause("ui.Pause", false, true),
      follow_pose("ui.Follow Pose", false, true),
      draw_vertical_cloud("ui.Draw Vertical", false, true),
      draw_horizontal_cloud("ui.Draw Horizontal", true, true),
      // draw_raycast_vertical_cloud("ui.Draw Cast Vertical", false, true),
      draw_raycast_horizontal_cloud("ui.Draw Casted Horizontal", false, true),
      draw_surface("ui.Draw Surface", true, true),
      draw_wireframe("ui.Draw WireFrame", true, true),
      display_height("ui.Display Height", 2.f, -10.f, 10.f),
      frame("ui.Frame:", "0"),
      status("ui.Status:", ""),
      threadPack(SharedState::get()) {
  live_vertical_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  live_horizontal_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  vertical_pose.setIdentity();
  horizontal_pose.setIdentity();

  raycasted_cloud_vertical.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  raycasted_cloud_horizontal.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  constexpr int kWindowWidth = 1280;
  constexpr int kWindowHeight = 960;
  constexpr int kPanelWidth = 180;
  pangolin::CreateWindowAndBind("Scan2Mesh", kWindowWidth + kPanelWidth,
                                kWindowHeight);
  SetupLighting();
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glEnable(GL_DEPTH_TEST);
  s_cam = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
      pangolin::ModelViewLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0));
  pangolin::Display("cam")
      .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
      .SetHandler(new pangolin::Handler3D(s_cam));
  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                        pangolin::Attach::Pix(kPanelWidth));

  reset();
}

void MeshTrackingVisualizer::SetupLighting() {
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  const GLfloat light_position[] = {5.0f, 5.0f, 10.0f, 1.0f};
  const GLfloat light_ambient[] = {0.25f, 0.25f, 0.25f, 1.0f};
  const GLfloat light_diffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
  const GLfloat light_specular[] = {0.3f, 0.3f, 0.3f, 1.0f};

  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);

  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

  glEnable(GL_NORMALIZE);

  glDisable(GL_LIGHTING);
}

MeshTrackingVisualizer::~MeshTrackingVisualizer() { reset(); }

void MeshTrackingVisualizer::reset() {}

void MeshTrackingVisualizer::SetTracker(TrackerInterface* tracker_interface) {
  trackerInterface = tracker_interface;

  const auto mesh_info = tracker_interface->GetRaycastTargetMesh();

  cast_surface.reset(new MeshRenderer(*mesh_info->vertices, *mesh_info->faces));
}

bool MeshTrackingVisualizer::Run() {
  while (!pangolin::ShouldQuit()) {
    glClearColor(0.18f, 0.20f, 0.21f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    pangolin::Display("cam").Activate(s_cam);

    UpdateLiveCloudDisplay();
    UpdateCastCloudDisplay();

    Render();
    HandleInput();

    pangolin::FinishFrame();
  }

  trackerInterface->end_requested.Set(true);
  return true;
}

void MeshTrackingVisualizer::UpdateLiveCloudDisplay() {
  {
    boost::mutex::scoped_lock lock(trackerInterface->live_mutex,
                                   boost::try_to_lock);

    if (lock && trackerInterface->live_available) {
      trackered_pose = trackerInterface->GetLastTrackedPose();
      trackerInterface->live_available = false;

      const auto vertical = trackerInterface->GetLiveVerticalCloud();
      const auto horizontal = trackerInterface->GetLiveHorizontalCloud();

      *live_vertical_cloud = *vertical->cloud;
      *live_horizontal_cloud = *horizontal->cloud;

      vertical_pose = vertical->transform;
      horizontal_pose = horizontal->transform;
    }
  }

  // Rebuild display cloud according to the current buttons.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  if (draw_vertical_cloud && !live_vertical_cloud->empty()) {
    pcl::PointCloud<pcl::PointXYZRGB> cloud = *live_vertical_cloud;
    pcl::transformPointCloud(cloud, cloud, vertical_pose);
    *display_cloud += cloud;
  }

  if (draw_horizontal_cloud && !live_horizontal_cloud->empty()) {
    pcl::PointCloud<pcl::PointXYZRGB> cloud = *live_horizontal_cloud;
    pcl::transformPointCloud(cloud, cloud, horizontal_pose);
    *display_cloud += cloud;
  }

  // Always update the renderer, including when both buttons are disabled.
  if (!display_cloud->empty()) {
    live_cloud.reset(new PointCloudRenderer(*display_cloud));
  } else {
    live_cloud.reset();
  }
}

void MeshTrackingVisualizer::UpdateCastCloudDisplay() {
  {
    boost::mutex::scoped_lock lock(trackerInterface->raycast_mutex,
                                   boost::try_to_lock);

    if (lock && trackerInterface->raycast_available) {
      trackerInterface->raycast_available = false;

      // const auto vertical = trackerInterface->GetRaycastVerticalCloud();
      const auto horizontal = trackerInterface->GetRaycastHorizontalCloud();

      // *raycasted_cloud_vertical = *vertical->cloud_with_normal;
      *raycasted_cloud_horizontal = *horizontal->cloud_with_normal;
    }
  }

  // Rebuild display cloud according to the current buttons.
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr display_cloud(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  // if (draw_raycast_vertical_cloud && !raycasted_cloud_vertical->empty()) {
  //   *display_cloud += *raycasted_cloud_vertical;
  // }

  if (draw_raycast_horizontal_cloud && !raycasted_cloud_horizontal->empty()) {
    *display_cloud += *raycasted_cloud_horizontal;
  }

  if (!display_cloud->empty()) {
    raycast_cloud.reset(new PointCloudRenderer(*display_cloud));
  } else {
    raycast_cloud.reset();
  }
}

void MeshTrackingVisualizer::DrawLidarPose(const Eigen::Matrix4f& lidar_pose,
                                           float axis_length) {
  glPushMatrix();
  glMultMatrixf(lidar_pose.data());
  glLineWidth(4.0f);
  pangolin::glDrawAxis(axis_length);
  glLineWidth(1.0f);
  glPopMatrix();
}

void MeshTrackingVisualizer::DrawLidarBox(const Eigen::Matrix4f& lidar_pose,
                                          float axis_length) {
  glPushMatrix();
  glMultMatrixf(lidar_pose.data());

  // Draw coordinate axes.
  glLineWidth(3.0f);

  glBegin(GL_LINES);
  // X axis.
  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(axis_length, 0.0f, 0.0f);
  // Y axis.
  glColor3f(0.0f, 1.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, axis_length);
  // Z axis.
  glColor3f(0.0f, 0.5f, 1.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, axis_length);
  glEnd();

  glLineWidth(1.0f);
  // Draw LiDAR body.
  glColor3f(1.0f, 0.7f, 0.1f);

  const float size = 0.12f;
  glPushMatrix();
  glScalef(size, size, size);

  pangolin::glDrawColouredCube();

  glPopMatrix();
  glPopMatrix();
}

void MeshTrackingVisualizer::Render() {
  if (draw_surface) {
    cast_surface->DrawFilledTriangles(display_height);
  }

  if (draw_wireframe) {
    cast_surface->DrawWireframe(display_height);
  }

  if (live_cloud) {
    live_cloud->DrawPoints();
  }

  if (raycast_cloud) {
    raycast_cloud->DrawPoints();
  }

  DrawLidarPose(horizontal_pose, 0.5f);
  // DrawLidarBox(horizontal_pose, 0.5f);

  glDisable(GL_DEPTH_TEST);

  glEnable(GL_DEPTH_TEST);
}

void MeshTrackingVisualizer::UpdateFollowCamera() {
  pangolin::OpenGlMatrix model_view;
  const Eigen::Matrix3f rotation = trackered_pose.topLeftCorner<3, 3>();
  const Eigen::Vector3f position = trackered_pose.topRightCorner<3, 1>();

  const Eigen::Vector3f local_forward(0.0f, 0.0f, -1.0f);
  const Eigen::Vector3f local_up(0.0f, -1.0f, 0.0f);

  const Eigen::Vector3f forward = (rotation * local_forward).normalized();
  const Eigen::Vector3f up = (rotation * local_up).normalized();

  constexpr float kCameraDistance = 20.0f;

  const Eigen::Vector3f eye = position - forward * kCameraDistance;
  const Eigen::Vector3f at = position;

  const Eigen::Vector3f z_axis = (eye - at).normalized();
  const Eigen::Vector3f x_axis = up.cross(z_axis).normalized();
  const Eigen::Vector3f y_axis = z_axis.cross(x_axis);

  Eigen::Matrix4d view_matrix;
  view_matrix << x_axis.x(), x_axis.y(), x_axis.z(), -x_axis.dot(eye),
      y_axis.x(), y_axis.y(), y_axis.z(), -y_axis.dot(eye), z_axis.x(),
      z_axis.y(), z_axis.z(), -z_axis.dot(eye), 0.0, 0.0, 0.0, 1.0;

  std::memcpy(model_view.m, view_matrix.data(), sizeof(view_matrix));

  s_cam.SetModelViewMatrix(model_view);
}

void MeshTrackingVisualizer::HandleInput() {
  if (pause.GuiChanged()) {
    threadPack.pause_tracking.Set(pause);
  }

  status = pause ? "Paused" : "Running";

  if (follow_pose) {
    UpdateFollowCamera();
  }

  frame = std::to_string(threadPack.frame_id.Get());
}