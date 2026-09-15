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

#ifndef ETERNELLE_URDF_LOADER_H_
#define ETERNELLE_URDF_LOADER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <string>

namespace urdf {
class ModelInterface;
using ModelInterfaceSharedPtr = std::shared_ptr<ModelInterface>;
}  // namespace urdf

class URDFLoader {
 public:
  /**
   * @brief Loads the transform between two directly connected links.
   *
   * The child link must be directly connected to the parent link by a joint.
   *
   * @param urdf_file Path to the URDF file.
   * @param parent_link Name of the parent link.
   * @param child_link Name of the child link.
   * @param transform Output transform from parent_link to child_link.
   *
   * @return True if the transform is loaded successfully.
   */
  static bool LoadFixedJointTransform(const std::string& urdf_file,
                                      const std::string& parent_link,
                                      const std::string& child_link,
                                      Eigen::Matrix4f* transform);

  /**
   * @brief Loads the transform between any two links in the URDF tree.
   *
   * The returned transform converts coordinates from source_link to
   * target_link:
   *
   *   P_target = T_source_target * P_source
   *
   * @param urdf_file Path to the URDF file.
   * @param source_link Source coordinate frame.
   * @param target_link Target coordinate frame.
   * @param transform Output transform from source_link to target_link.
   *
   * @return True if the transform is loaded successfully.
   */
  static bool LoadLinkTransform(const std::string& urdf_file,
                                const std::string& source_link,
                                const std::string& target_link,
                                Eigen::Matrix4f* transform);

 private:
  /**
   * @brief Converts a position and quaternion to a homogeneous transform.
   */
  static Eigen::Matrix4f PoseToMatrix(const Eigen::Vector3d& position,
                                      const Eigen::Quaterniond& rotation);

  /**
   * @brief Computes the transform from the URDF root link to a target link.
   *
   * @param model Parsed URDF model.
   * @param link_name Target link name.
   * @param transform Output transform from the root link to link_name.
   *
   * @return True if the transform is computed successfully.
   */
  static bool GetRootToLinkTransform(const urdf::ModelInterfaceSharedPtr& model,
                                     const std::string& link_name,
                                     Eigen::Matrix4f* transform);
};

#endif  // ETERNELLE_URDF_LOADER_H_