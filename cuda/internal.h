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

#ifndef INTERNAL_HPP_
#define INTERNAL_HPP_

#include <stdlib.h>

#include <iostream>

#include "containers/device_array.hpp"
#include "cuda_runtime_api.h"

#if __CUDA_ARCH__ < 300
#define MAX_THREADS 512
#else
#define MAX_THREADS 1024
#endif

#if defined(__GNUC__)
#define cudaSafeCall(expr) ___cudaSafeCall(expr, __FILE__, __LINE__, __func__)
#else
#define cudaSafeCall(expr) ___cudaSafeCall(expr, __FILE__, __LINE__)
#endif

static inline void error(const char* error_string, const char* file,
                         const int line, const char* func) {
  std::cout << "Error: " << error_string << "\t" << file << ":" << line
            << std::endl;
  exit(0);
}

static inline void ___cudaSafeCall(cudaError_t err, const char* file,
                                   const int line, const char* func = "") {
  if (cudaSuccess != err) error(cudaGetErrorString(err), file, line, func);
}

struct Se3NormalEquation {
  float h00, h01, h02, h03, h04, h05, h06, h11, h12, h13, h14, h15, h16, h22,
      h23, h24, h25, h26, h33, h34, h35, h36, h44, h45, h46, h55, h56;
  float squared_error;
  float correspondence_count;
  __device__ inline void Accumulate(const Se3NormalEquation& other) {
    h00 += other.h00;
    h01 += other.h01;
    h02 += other.h02;
    h03 += other.h03;
    h04 += other.h04;
    h05 += other.h05;
    h06 += other.h06;

    h11 += other.h11;
    h12 += other.h12;
    h13 += other.h13;
    h14 += other.h14;
    h15 += other.h15;
    h16 += other.h16;

    h22 += other.h22;
    h23 += other.h23;
    h24 += other.h24;
    h25 += other.h25;
    h26 += other.h26;

    h33 += other.h33;
    h34 += other.h34;
    h35 += other.h35;
    h36 += other.h36;

    h44 += other.h44;
    h45 += other.h45;
    h46 += other.h46;

    h55 += other.h55;
    h56 += other.h56;
    squared_error += other.squared_error;
    correspondence_count += other.correspondence_count;
  }
};

struct PointXYZRGB {
  union __attribute__((aligned(16))) {
    float data[4];
    struct {
      float x;
      float y;
      float z;
    };
  };

  union {
    union {
      struct {
        unsigned char b;
        unsigned char g;
        unsigned char r;
        unsigned char a;
      };
      float rgb;
    };
    int rgba;
  };
};

struct PointXYZRGBNormal {
  union __attribute__((aligned(16))) {
    float data[4];
    struct {
      float x;
      float y;
      float z;
    };
  };

  union __attribute__((aligned(16))) {
    float data_n[4];
    float normal[3];
    struct {
      float normal_x;
      float normal_y;
      float normal_z;
    };
  };

  union {
    struct {
      union {
        union {
          struct {
            unsigned char b;
            unsigned char g;
            unsigned char r;
            unsigned char a;
          };
          float rgb;
        };
        int rgba;
      };
      float curvature;
    };
    float data_c[4];
  };
};

struct Matrix3f {
  float3 data[3];
};

template <class D, class Matx>
D& device_cast(Matx& matx) {
  return (*reinterpret_cast<D*>(matx.data()));
}

void ComputeICPNormalEquations(
    const Matrix3f& Rcurr, const float3& tcurr, const Matrix3f& R_imu_sta,
    const float3& t_imu_sta, const Matrix3f& Rprev_inv, const float3& tprev,
    const PtrSz<PointXYZRGB> cloud_in_hor,
    const PtrSz<PointXYZRGBNormal> cloud_cast_hor, float distance_threshold,
    float angle_threshold, DeviceArray<Se3NormalEquation>& sum,
    DeviceArray<Se3NormalEquation>& out, float* matrixA_host,
    float* vectorB_host, float* residual_host, int threads, int blocks,
    int cloud_number_hor);

#endif
