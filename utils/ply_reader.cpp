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

#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "ply_reader.h"
#include "tinyply.h"

namespace {
using Data = std::shared_ptr<tinyply::PlyData>;

// Request scalars separately: PLY permits reordered and mixed-type coordinates.
Data Request(tinyply::PlyFile& file, const std::string& element,
             const std::string& name, bool required = true) {
  for (const auto& e : file.get_elements()) {
    if (e.name != element) continue;
    for (const auto& p : e.properties) {
      if (p.name != name) continue;
      if (p.isList)
        throw std::runtime_error("Expected scalar property: " + name);
      return file.request_properties_from_element(element, {name});
    }
  }
  if (required) throw std::runtime_error("Missing PLY property: " + name);
  return {};
}

template <typename T>
double Scalar(const Data& data, std::size_t index) {
  T value;
  const std::size_t offset = index * sizeof(T);
  if (offset + sizeof(T) > data->buffer.size_bytes())
    throw std::runtime_error("PLY property buffer is too small");
  std::memcpy(&value, data->buffer.get_const() + offset, sizeof(T));
  return static_cast<double>(value);
}

double Value(const Data& data, std::size_t index) {
  using tinyply::Type;
  switch (data->t) {
    case Type::INT8:
      return Scalar<int8_t>(data, index);
    case Type::UINT8:
      return Scalar<uint8_t>(data, index);
    case Type::INT16:
      return Scalar<int16_t>(data, index);
    case Type::UINT16:
      return Scalar<uint16_t>(data, index);
    case Type::INT32:
      return Scalar<int32_t>(data, index);
    case Type::UINT32:
      return Scalar<uint32_t>(data, index);
    case Type::FLOAT32:
      return Scalar<float>(data, index);
    case Type::FLOAT64:
      return Scalar<double>(data, index);
    default:
      throw std::runtime_error("Unsupported PLY scalar type");
  }
}

struct PlyInput {
  std::ifstream stream;
  tinyply::PlyFile file;
  std::array<Data, 3> xyz;
  explicit PlyInput(const std::string& filename)
      : stream(filename, std::ios::binary) {
    std::string magic;
    if (!std::getline(stream, magic) || (magic != "ply" && magic != "ply\r"))
      throw std::runtime_error("Missing PLY signature");
    stream.seekg(0);
    if (!stream || !file.parse_header(stream))
      throw std::runtime_error("Cannot open or parse PLY header");
    const char* names[] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) xyz[i] = Request(file, "vertex", names[i]);
    if (!xyz[0]->count ||
        xyz[0]->count > std::numeric_limits<unsigned int>::max())
      throw std::runtime_error("Invalid PLY vertex count");
  }
  float Coordinate(int axis, std::size_t index) const {
    const float result = static_cast<float>(Value(xyz[axis], index));
    if (!std::isfinite(result))
      throw std::runtime_error("Non-finite PLY coordinate");
    return result;
  }
};
}  // namespace

bool LoadPLYMesh(const std::string& filename, unsigned int& vertex_num,
                 unsigned int& face_num, std::vector<float>& vertices,
                 std::vector<unsigned int>& faces) {
  vertex_num = face_num = 0;
  vertices.clear();
  faces.clear();
  try {
    PlyInput input(filename);
    Data indices;
    for (const auto& e : input.file.get_elements()) {
      if (e.name != "face") continue;
      for (const auto& p : e.properties) {
        if (p.name != "vertex_indices" && p.name != "vertex_index") continue;
        if (!p.isList || p.propertyType == tinyply::Type::FLOAT32 ||
            p.propertyType == tinyply::Type::FLOAT64)
          throw std::runtime_error("Face indices must be an integer list");
        // A hint of three validates every face, including binary fast paths.
        indices =
            input.file.request_properties_from_element("face", {p.name}, 3);
        break;
      }
    }
    if (!indices || !indices->count ||
        indices->count > std::numeric_limits<unsigned int>::max())
      throw std::runtime_error("PLY mesh requires triangular faces");
    input.file.read(input.stream);
    if (input.file.is_binary_file() && input.stream.eof())
      throw std::runtime_error("Truncated binary PLY data");
    std::vector<float> new_vertices(input.xyz[0]->count * 3);
    std::vector<unsigned int> new_faces(indices->count * 3);
    for (std::size_t i = 0; i < input.xyz[0]->count; ++i)
      for (int axis = 0; axis < 3; ++axis)
        new_vertices[3 * i + axis] = input.Coordinate(axis, i);
    for (std::size_t i = 0; i < new_faces.size(); ++i) {
      const double index = Value(indices, i);
      if (index < 0 || index >= input.xyz[0]->count)
        throw std::runtime_error("PLY face index is out of range");
      new_faces[i] = static_cast<unsigned int>(index);
    }
    vertices.swap(new_vertices);
    faces.swap(new_faces);
    vertex_num = static_cast<unsigned int>(vertices.size() / 3);
    face_num = static_cast<unsigned int>(faces.size() / 3);
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Failed to load mesh " << filename << ": " << e.what() << '\n';
    return false;
  }
}

bool LoadPLYCloud(const std::string& filename, scan2mesh::ColorCloud& cloud) {
  cloud.clear();
  try {
    PlyInput input(filename);
    std::array<Data, 3> colors;
    const char* names[] = {"red", "green", "blue"};
    for (int i = 0; i < 3; ++i)
      colors[i] = Request(input.file, "vertex", names[i], false);
    input.file.read(input.stream);
    if (input.file.is_binary_file() && input.stream.eof())
      throw std::runtime_error("Truncated binary PLY data");
    scan2mesh::ColorCloud result;
    result.points.resize(input.xyz[0]->count);
    for (std::size_t i = 0; i < result.size(); ++i) {
      auto& p = result.points[i];
      p.x = input.Coordinate(0, i);
      p.y = input.Coordinate(1, i);
      p.z = input.Coordinate(2, i);
      std::uint8_t* channels[] = {&p.r, &p.g, &p.b};
      for (int c = 0; c < 3; ++c) {
        if (!colors[c]) continue;
        const double value = Value(colors[c], i);
        if (!std::isfinite(value) || value < 0 || value > 255)
          throw std::runtime_error("PLY color is out of range");
        *channels[c] = static_cast<std::uint8_t>(value);
      }
    }
    cloud = std::move(result);
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Failed to load cloud " << filename << ": " << e.what()
              << '\n';
    return false;
  }
}

bool SavePLYCloud(const std::string& filename,
                  const scan2mesh::ColorCloud& cloud) {
  try {
    std::ofstream stream(filename, std::ios::binary);
    if (!stream) throw std::runtime_error("Cannot open output PLY");
    std::vector<float> xyz;
    std::vector<std::uint8_t> rgb;
    xyz.reserve(cloud.size() * 3);
    rgb.reserve(cloud.size() * 3);
    for (const auto& p : cloud.points) {
      xyz.insert(xyz.end(), {p.x, p.y, p.z});
      rgb.insert(rgb.end(), {p.r, p.g, p.b});
    }
    tinyply::PlyFile file;
    file.add_properties_to_element(
        "vertex", {"x", "y", "z"}, tinyply::Type::FLOAT32, cloud.size(),
        reinterpret_cast<const std::uint8_t*>(xyz.data()),
        tinyply::Type::INVALID, 0);
    file.add_properties_to_element("vertex", {"red", "green", "blue"},
                                   tinyply::Type::UINT8, cloud.size(),
                                   rgb.data(), tinyply::Type::INVALID, 0);
    file.write(stream, true);
    stream.flush();
    if (!stream) throw std::runtime_error("Failed to write PLY");
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Failed to save cloud " << filename << ": " << e.what()
              << '\n';
    return false;
  }
}
