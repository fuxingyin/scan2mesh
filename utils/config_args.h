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

#ifndef CONFIGARGS_H_
#define CONFIGARGS_H_

#include <pcl/console/parse.h>

#include <cassert>
#include <cstdlib>
#include <iostream>
#include <string>

class ConfigArgs {
 public:
  static const ConfigArgs& get(int argc = 0, char** argv = nullptr) {
    static const ConfigArgs instance(argc, argv);
    return instance;
  }

  static void usage(const std::string& argv0) {
    std::cout << "Usage: " << argv0 << " [options]\n"
              << "\n"
              << "Options:\n"
              << "  -d <path>       Path to the registration folder\n"
              << "  -m <mesh>       Path to the registration mesh\n"
              << "  -u <path>       Path to urdf configure file\n"
              << "  -gpu <id>       CUDA GPU device ID (default: 0)\n"
              << "  -vs             Enable processing visualization\n"
              << "  -h, --help      Display this help message and exit\n"
              << std::endl;
  }

  std::string data_path;
  std::string mesh_path;
  std::string urdf_path;
  int gpu_id;
  bool enable_visualization;

 private:
  ConfigArgs(int argc, char** argv) : gpu_id(0), enable_visualization(false) {
    assert(argc > 0 && argv != nullptr);

    const bool help = pcl::console::find_switch(argc, argv, "-h") ||
                      pcl::console::find_switch(argc, argv, "--help");

    if (help) {
      usage(argv[0]);
      std::exit(EXIT_SUCCESS);
    }

    pcl::console::parse_argument(argc, argv, "-d", data_path);
    pcl::console::parse_argument(argc, argv, "-m", mesh_path);
    pcl::console::parse_argument(argc, argv, "-u", urdf_path);
    pcl::console::parse_argument(argc, argv, "-gpu", gpu_id);

    enable_visualization = pcl::console::find_switch(argc, argv, "-vs");

    if (data_path.empty()) {
      std::cerr << "Error: -d <logfile> is required.\n\n";
      usage(argv[0]);
      std::exit(EXIT_FAILURE);
    }

    if (mesh_path.empty()) {
      std::cerr << "Error: -m <logfile> is required.\n\n";
      usage(argv[0]);
      std::exit(EXIT_FAILURE);
    }

    if (urdf_path.empty()) {
      std::cerr << "Error: -u <logfile> is required.\n\n";
      usage(argv[0]);
      std::exit(EXIT_FAILURE);
    }
  }
};

#endif  // CONFIGARGS_H_