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

#include "urdf_loader.h"

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

std::string ReadFile(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    return "";
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

bool LoadURDFModel(const std::string& urdf_file,
                   urdf::ModelInterfaceSharedPtr* model) {
  if (model == nullptr) {
    return false;
  }

  const std::string urdf_xml = ReadFile(urdf_file);
  if (urdf_xml.empty()) {
    std::cerr << "Failed to read URDF file: " << urdf_file << std::endl;
    return false;
  }

  *model = urdf::parseURDF(urdf_xml);
  if (!*model) {
    std::cerr << "Failed to parse URDF: " << urdf_file << std::endl;
    return false;
  }

  return true;
}

}  // namespace

Eigen::Matrix4f URDFLoader::PoseToMatrix(const Eigen::Vector3d& position,
                                         const Eigen::Quaterniond& rotation) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) =
      rotation.normalized().toRotationMatrix().cast<float>();
  transform.block<3, 1>(0, 3) = position.cast<float>();
  return transform;
}

bool URDFLoader::LoadFixedJointTransform(const std::string& urdf_file,
                                         const std::string& parent_link,
                                         const std::string& child_link,
                                         Eigen::Matrix4f* transform) {
  if (transform == nullptr) {
    return false;
  }

  urdf::ModelInterfaceSharedPtr model;
  if (!LoadURDFModel(urdf_file, &model)) {
    return false;
  }

  const urdf::LinkConstSharedPtr child = model->getLink(child_link);
  if (!child) {
    std::cerr << "Cannot find child link: " << child_link << std::endl;
    return false;
  }

  const urdf::JointConstSharedPtr joint = child->parent_joint;
  if (!joint) {
    std::cerr << "Link has no parent joint: " << child_link << std::endl;
    return false;
  }

  if (joint->parent_link_name != parent_link) {
    std::cerr << "Unexpected parent link for " << child_link << ": "
              << joint->parent_link_name << ", expected: " << parent_link
              << std::endl;

    return false;
  }

  const urdf::Pose& pose = joint->parent_to_joint_origin_transform;
  const Eigen::Vector3d position(pose.position.x, pose.position.y,
                                 pose.position.z);
  const Eigen::Quaterniond rotation(pose.rotation.w, pose.rotation.x,
                                    pose.rotation.y, pose.rotation.z);
  *transform = PoseToMatrix(position, rotation);

  return true;
}

bool URDFLoader::GetRootToLinkTransform(
    const urdf::ModelInterfaceSharedPtr& model, const std::string& link_name,
    Eigen::Matrix4f* transform) {
  if (transform == nullptr) {
    return false;
  }

  if (!model) {
    std::cerr << "URDF model is null." << std::endl;
    return false;
  }

  urdf::LinkConstSharedPtr link = model->getLink(link_name);
  if (!link) {
    std::cerr << "Cannot find link: " << link_name << std::endl;
    return false;
  }

  Eigen::Matrix4f root_to_link = Eigen::Matrix4f::Identity();

  // Walk from the target link upward to the root link.
  while (link && link->parent_joint) {
    const urdf::JointConstSharedPtr joint = link->parent_joint;
    const urdf::Pose& pose = joint->parent_to_joint_origin_transform;
    const Eigen::Vector3d position(pose.position.x, pose.position.y,
                                   pose.position.z);
    const Eigen::Quaterniond rotation(pose.rotation.w, pose.rotation.x,
                                      pose.rotation.y, pose.rotation.z);
    const Eigen::Matrix4f parent_to_child = PoseToMatrix(position, rotation);

    /*
     * URDF joint origin represents the pose of the child frame
     * relative to the parent frame:
     *
     *   T_root_child =
     *       T_root_parent * T_parent_child
     *
     * Since we are traversing upward:
     *
     *   child -> parent -> root
     *
     * prepend each transform:
     */

    root_to_link = parent_to_child * root_to_link;
    link = link->getParent();
  }
  *transform = root_to_link;
  return true;
}

bool URDFLoader::LoadLinkTransform(const std::string& urdf_file,
                                   const std::string& source_link,
                                   const std::string& target_link,
                                   Eigen::Matrix4f* transform) {
  if (transform == nullptr) {
    return false;
  }

  urdf::ModelInterfaceSharedPtr model;
  if (!LoadURDFModel(urdf_file, &model)) {
    return false;
  }

  Eigen::Matrix4f root_to_source;
  Eigen::Matrix4f root_to_target;
  if (!GetRootToLinkTransform(model, source_link, &root_to_source)) {
    return false;
  }
  if (!GetRootToLinkTransform(model, target_link, &root_to_target)) {
    return false;
  }

  /*
   * We have:
   *
   *   T_root_target =
   *       T_root_source * T_source_target
   *
   * Therefore:
   *
   *   T_source_target =
   *       inverse(T_root_source) * T_root_target
   */

  *transform = root_to_source.inverse() * root_to_target;
  return true;
}