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

#include "scan2mesh_controller.h"

#include <boost/algorithm/algorithm.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include "shared_states.h"

Scan2MeshController* Scan2MeshController::controller = 0;

Scan2MeshController::Scan2MeshController(int argc, char* argv[])
    : scan2mesh_tracker(nullptr), visualizer(nullptr) {
  ConfigArgs::get(argc, argv);

  assert(!Scan2MeshController::controller);

  Scan2MeshController::controller = this;
}

Scan2MeshController::~Scan2MeshController() {}

int Scan2MeshController::start() {
  if (setup()) {
    return MainLoop();
  } else {
    return -1;
  }
}

bool Scan2MeshController::setup() {
  SharedState::get();

  scan2mesh_tracker = std::unique_ptr<TrackerInterface>(new TrackerInterface(
      ConfigArgs::get().data_path, ConfigArgs::get().mesh_path,
      ConfigArgs::get().urdf_path));

  if (ConfigArgs::get().enable_visualization) {
    visualizer =
        std::unique_ptr<MeshTrackingVisualizer>(new MeshTrackingVisualizer());

    visualizer->SetTracker(scan2mesh_tracker.get());
  }

  return true;
}

int Scan2MeshController::MainLoop() {
  threads.add_thread(new boost::thread(
      boost::bind(&TrackerInterface::RunTracking, scan2mesh_tracker.get())));

  if (visualizer) {
    visualizer->Run();
  }

  threads.join_all();

  return 0;
}
