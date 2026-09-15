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

#include "ply_reader.h"

#include <cstdint>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

bool LoadPLYMesh(const std::string &filename, unsigned int &vertex_num,
                 unsigned int &face_num, std::vector<float> &vertices,
                 std::vector<unsigned int> &faces) {
  FILE *file = std::fopen(filename.c_str(), "rb");
  if (file == nullptr) {
    std::cerr << "ERROR: Unable to open PLY file: " << filename << std::endl;
    return false;
  }

  auto close_file = [&file]() {
    if (file != nullptr) {
      std::fclose(file);
      file = nullptr;
    }
  };

  enum class ElementType {
    kNone,
    kVertex,
    kFace,
  };

  bool is_binary_little_endian = false;
  bool header_finished = false;
  ElementType current_element = ElementType::kNone;

  size_t vertex_property_count = 0;
  size_t face_color_channels = 0;

  char line[1024];
  while (std::fgets(line, sizeof(line), file) != nullptr) {
    std::string header_line(line);

    std::istringstream stream(header_line);
    std::string keyword;

    stream >> keyword;

    if (keyword.empty()) {
      continue;
    }

    if (keyword == "ply") {
      continue;
    }

    if (keyword == "format") {
      std::string format;
      stream >> format;
      if (format == "binary_little_endian") {
        is_binary_little_endian = true;
      } else if (format == "ascii") {
        std::cerr << "ERROR: ASCII PLY is not supported." << std::endl;
        close_file();
        return false;
      } else {
        std::cerr << "ERROR: Unsupported PLY format: " << format << std::endl;
        close_file();
        return false;
      }
      continue;
    }

    if (keyword == "element") {
      std::string element_name;
      size_t element_count = 0;
      stream >> element_name >> element_count;
      if (element_name == "vertex") {
        current_element = ElementType::kVertex;
        vertex_num = static_cast<unsigned int>(element_count);
      } else if (element_name == "face") {
        current_element = ElementType::kFace;
        face_num = static_cast<unsigned int>(element_count);
      } else {
        current_element = ElementType::kNone;
      }
      continue;
    }

    if (keyword == "property") {
      if (current_element == ElementType::kVertex) {
        ++vertex_property_count;
      } else if (current_element == ElementType::kFace) {
        std::string type;
        stream >> type;
        if (type != "list") {
          std::string property_name;
          stream >> property_name;
          if (property_name == "red" || property_name == "green" ||
              property_name == "blue" || property_name == "alpha") {
            ++face_color_channels;
          }
        }
      }
      continue;
    }

    if (keyword == "end_header") {
      header_finished = true;
      break;
    }
  }

  if (!header_finished) {
    std::cerr << "ERROR: Invalid PLY file. Header not terminated." << std::endl;
    close_file();
    return false;
  }

  if (!is_binary_little_endian) {
    std::cerr << "ERROR: Only binary little-endian PLY is supported."
              << std::endl;
    close_file();
    return false;
  }

  if (vertex_num == 0) {
    std::cerr << "ERROR: PLY file contains no vertices." << std::endl;
    close_file();
    return false;
  }

  if (vertex_property_count < 3) {
    std::cerr << "ERROR: Invalid vertex properties." << std::endl;
    close_file();
    return false;
  }

  if (face_color_channels != 0 && face_color_channels != 3 &&
      face_color_channels != 4) {
    std::cerr << "ERROR: Unsupported number of face color channels: "
              << face_color_channels << std::endl;
    close_file();
    return false;
  }

  vertices.resize(static_cast<size_t>(vertex_num) * 3);

  for (unsigned int i = 0; i < vertex_num; ++i) {
    float vertex[3];
    if (std::fread(vertex, sizeof(float), 3, file) != 3) {
      std::cerr << "ERROR: Failed to read vertex " << i << std::endl;
      close_file();
      return false;
    }
    vertices[3 * i + 0] = vertex[0];
    vertices[3 * i + 1] = vertex[1];
    vertices[3 * i + 2] = vertex[2];
    const size_t extra_properties = vertex_property_count - 3;
    if (extra_properties > 0) {
      const size_t bytes_to_skip = extra_properties * sizeof(float);
      if (std::fseek(file, static_cast<long>(bytes_to_skip), SEEK_CUR) != 0) {
        std::cerr << "ERROR: Failed to skip vertex properties." << std::endl;
        close_file();
        return false;
      }
    }
  }

  faces.resize(static_cast<size_t>(face_num) * 3);
  for (unsigned int i = 0; i < face_num; ++i) {
    uint8_t vertex_count = 0;

    if (std::fread(&vertex_count, sizeof(uint8_t), 1, file) != 1) {
      std::cerr << "ERROR: Failed to read face vertex count for face " << i
                << std::endl;
      close_file();
      return false;
    }

    if (vertex_count != 3) {
      std::cerr << "ERROR: Only triangular faces are supported. "
                << "Face " << i << " has " << static_cast<int>(vertex_count)
                << " vertices." << std::endl;
      close_file();
      return false;
    }

    int indices[3];
    if (std::fread(indices, sizeof(int), 3, file) != 3) {
      std::cerr << "ERROR: Failed to read face indices for face " << i
                << std::endl;
      close_file();
      return false;
    }
    for (int j = 0; j < 3; ++j) {
      if (indices[j] < 0 ||
          static_cast<unsigned int>(indices[j]) >= vertex_num) {
        std::cerr << "ERROR: Invalid vertex index in face " << i << std::endl;
        close_file();
        return false;
      }

      faces[3 * i + j] = static_cast<unsigned int>(indices[j]);
    }
    // Skip optional face colors.
    if (face_color_channels > 0) {
      if (std::fseek(file,
                     static_cast<long>(face_color_channels * sizeof(uint8_t)),
                     SEEK_CUR) != 0) {
        std::cerr << "ERROR: Failed to skip face color data." << std::endl;
        close_file();
        return false;
      }
    }
  }

  close_file();
  return true;
}