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

#ifndef SCAN2MESH_CONTROLLER_H_
#define SCAN2MESH_CONTROLLER_H_

#include <GL/glew.h>

#include <fstream>
#include <iostream>
#include <memory>

#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/image_viewer.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>

#include "utils/config_args.h"
#include "mesh_tracking_visualizer.h"
#include "tracker_interface.h"

class Scan2MeshController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Scan2MeshController(int argc, char* argv[]);

  virtual ~Scan2MeshController();

  int start();

  static Scan2MeshController* controller;

 private:
  bool setup();

  int MainLoop();

  std::unique_ptr<TrackerInterface> scan2mesh_tracker;
  std::unique_ptr<MeshTrackingVisualizer> visualizer;

  boost::thread_group threads;
};

#endif
