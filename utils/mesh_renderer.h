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

#ifndef UTILS_INPUT_MESH_H_
#define UTILS_INPUT_MESH_H_

#include <pangolin/gl/gl.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

class MeshRenderer {
 public:
  MeshRenderer(const std::vector<float>& vertices,
               const std::vector<unsigned int>& faces)
      : vertices(vertices),
        faces(faces),
        vertex_count(vertices.size() / 3),
        triangle_count(faces.size() / 3),
        vertex_buffer(0),
        normal_buffer(0),
        triangle_index_buffer(0),
        filtered_triangle_index_buffer(0),
        filtered_wireframe_index_buffer(0),
        current_display_height(std::numeric_limits<float>::quiet_NaN()) {
    ComputeVertexNormals();
    CreateBuffers();
  }

  ~MeshRenderer() { ReleaseBuffers(); }

  MeshRenderer(const MeshRenderer&) = delete;
  MeshRenderer& operator=(const MeshRenderer&) = delete;

  // Draw the complete filled mesh.
  void DrawFilledTriangles() const {
    BindVertexAndNormalBuffers();

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, triangle_index_buffer);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);

    glColor3f(0.8f, 0.8f, 0.8f);

    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(faces.size()),
                   GL_UNSIGNED_INT, nullptr);

    UnbindBuffers();
  }

  // Draw only triangles below the specified height.
  void DrawFilledTriangles(float display_height) {
    UpdateFilteredIndices(display_height);

    if (filtered_triangle_indices.empty()) {
      return;
    }

    BindVertexAndNormalBuffers();

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, filtered_triangle_index_buffer);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);

    glColor3f(0.8f, 0.8f, 0.8f);

    glDrawElements(GL_TRIANGLES,
                   static_cast<GLsizei>(filtered_triangle_indices.size()),
                   GL_UNSIGNED_INT, nullptr);

    UnbindBuffers();
  }

  void DrawWireframe(float display_height) {
    UpdateFilteredIndices(display_height);

    if (filtered_wireframe_indices.empty()) {
      return;
    }

    glDisable(GL_LIGHTING);

    glLineWidth(1.0f);

    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);

    glVertexPointer(3, GL_FLOAT, 3 * sizeof(float), nullptr);
    glEnableClientState(GL_VERTEX_ARRAY);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, filtered_wireframe_index_buffer);

    glColor3f(0.3f, 0.3f, 0.3f);

    glDrawElements(GL_LINES,
                   static_cast<GLsizei>(filtered_wireframe_indices.size()),
                   GL_UNSIGNED_INT, nullptr);

    glDisableClientState(GL_VERTEX_ARRAY);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glLineWidth(1.0f);
  }

  // Visualize vertex normals for debugging.
  void DrawNormals(float normal_length = 0.1f, std::size_t step = 100) const {
    glDisable(GL_LIGHTING);

    glColor3f(1.0f, 0.0f, 0.0f);

    glBegin(GL_LINES);

    for (std::size_t i = 0; i < vertex_count; i += step) {
      const float x = vertices[3 * i];
      const float y = vertices[3 * i + 1];
      const float z = vertices[3 * i + 2];

      const float nx = normals[3 * i];
      const float ny = normals[3 * i + 1];
      const float nz = normals[3 * i + 2];

      glVertex3f(x, y, z);

      glVertex3f(x + nx * normal_length, y + ny * normal_length,
                 z + nz * normal_length);
    }

    glEnd();
  }

  std::size_t VertexCount() const { return vertex_count; }

  std::size_t TriangleCount() const { return triangle_count; }

 private:
  void UpdateFilteredIndices(float display_height) {
    if (std::fabs(display_height - current_display_height) < 1e-5f) {
      return;
    }

    current_display_height = display_height;

    filtered_triangle_indices.clear();
    filtered_wireframe_indices.clear();

    filtered_triangle_indices.reserve(faces.size());
    filtered_wireframe_indices.reserve(triangle_count * 6);

    for (std::size_t i = 0; i < triangle_count; ++i) {
      const unsigned int i0 = faces[3 * i];
      const unsigned int i1 = faces[3 * i + 1];
      const unsigned int i2 = faces[3 * i + 2];

      const float z0 = vertices[3 * i0 + 2];
      const float z1 = vertices[3 * i1 + 2];
      const float z2 = vertices[3 * i2 + 2];

      const float max_z = std::max(z0, std::max(z1, z2));

      if (max_z > display_height) {
        continue;
      }

      // Filled triangle indices.
      filtered_triangle_indices.push_back(i0);
      filtered_triangle_indices.push_back(i1);
      filtered_triangle_indices.push_back(i2);

      // Wireframe indices.
      filtered_wireframe_indices.push_back(i0);
      filtered_wireframe_indices.push_back(i1);

      filtered_wireframe_indices.push_back(i1);
      filtered_wireframe_indices.push_back(i2);

      filtered_wireframe_indices.push_back(i2);
      filtered_wireframe_indices.push_back(i0);
    }

    UploadFilteredIndices();
  }

  void UploadFilteredIndices() {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, filtered_triangle_index_buffer);

    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 filtered_triangle_indices.size() * sizeof(unsigned int),
                 filtered_triangle_indices.data(), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, filtered_wireframe_index_buffer);

    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 filtered_wireframe_indices.size() * sizeof(unsigned int),
                 filtered_wireframe_indices.data(), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  }

  void ComputeVertexNormals() {
    normals.assign(vertex_count * 3, 0.0f);

    for (std::size_t i = 0; i < triangle_count; ++i) {
      const unsigned int i0 = faces[3 * i];
      const unsigned int i1 = faces[3 * i + 1];
      const unsigned int i2 = faces[3 * i + 2];

      const Eigen::Vector3f v0(vertices[3 * i0], vertices[3 * i0 + 1],
                               vertices[3 * i0 + 2]);

      const Eigen::Vector3f v1(vertices[3 * i1], vertices[3 * i1 + 1],
                               vertices[3 * i1 + 2]);

      const Eigen::Vector3f v2(vertices[3 * i2], vertices[3 * i2 + 1],
                               vertices[3 * i2 + 2]);

      const Eigen::Vector3f face_normal = (v1 - v0).cross(v2 - v0);

      AccumulateNormal(i0, face_normal);
      AccumulateNormal(i1, face_normal);
      AccumulateNormal(i2, face_normal);
    }

    NormalizeVertexNormals();
  }

  void AccumulateNormal(unsigned int vertex_index,
                        const Eigen::Vector3f& normal) {
    normals[3 * vertex_index] += normal.x();
    normals[3 * vertex_index + 1] += normal.y();
    normals[3 * vertex_index + 2] += normal.z();
  }

  void NormalizeVertexNormals() {
    for (std::size_t i = 0; i < vertex_count; ++i) {
      Eigen::Vector3f normal(normals[3 * i], normals[3 * i + 1],
                             normals[3 * i + 2]);

      if (normal.squaredNorm() > 1e-12f) {
        normal.normalize();
      } else {
        normal.setZero();
      }

      normals[3 * i] = normal.x();
      normals[3 * i + 1] = normal.y();
      normals[3 * i + 2] = normal.z();
    }
  }

  void CreateBuffers() {
    glGenBuffers(1, &vertex_buffer);
    glGenBuffers(1, &normal_buffer);
    glGenBuffers(1, &triangle_index_buffer);
    glGenBuffers(1, &filtered_triangle_index_buffer);
    glGenBuffers(1, &filtered_wireframe_index_buffer);

    // Vertex buffer.
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);

    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float),
                 vertices.data(), GL_STATIC_DRAW);

    // Normal buffer.
    glBindBuffer(GL_ARRAY_BUFFER, normal_buffer);

    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(float),
                 normals.data(), GL_STATIC_DRAW);

    // Original triangle index buffer.
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, triangle_index_buffer);

    glBufferData(GL_ELEMENT_ARRAY_BUFFER, faces.size() * sizeof(unsigned int),
                 faces.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  }

  void BindVertexAndNormalBuffers() const {
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);

    glVertexPointer(3, GL_FLOAT, 3 * sizeof(float), nullptr);

    glEnableClientState(GL_VERTEX_ARRAY);

    glBindBuffer(GL_ARRAY_BUFFER, normal_buffer);

    glNormalPointer(GL_FLOAT, 3 * sizeof(float), nullptr);

    glEnableClientState(GL_NORMAL_ARRAY);
  }

  void UnbindBuffers() const {
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  }

  void ReleaseBuffers() {
    DeleteBuffer(vertex_buffer);
    DeleteBuffer(normal_buffer);
    DeleteBuffer(triangle_index_buffer);
    DeleteBuffer(filtered_triangle_index_buffer);
    DeleteBuffer(filtered_wireframe_index_buffer);
  }

  void DeleteBuffer(GLuint& buffer) {
    if (buffer != 0) {
      glDeleteBuffers(1, &buffer);
      buffer = 0;
    }
  }

 private:
  // Mesh data.
  const std::vector<float>& vertices;
  const std::vector<unsigned int>& faces;

  std::size_t vertex_count;
  std::size_t triangle_count;

  // CPU-side derived data.
  std::vector<float> normals;

  std::vector<unsigned int> filtered_triangle_indices;
  std::vector<unsigned int> filtered_wireframe_indices;

  // GPU buffers.
  GLuint vertex_buffer;
  GLuint normal_buffer;

  GLuint triangle_index_buffer;

  GLuint filtered_triangle_index_buffer;
  GLuint filtered_wireframe_index_buffer;

  float current_display_height;
};

#endif  // UTILS_INPUT_MESH_H_