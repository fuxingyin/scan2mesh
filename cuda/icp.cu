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

#include "device.hpp"

namespace {

constexpr unsigned int kFullWarpMask = 0xffffffffu;

__device__ __forceinline__ Se3NormalEquation MakeZeroNormalEquation() {
  Se3NormalEquation result{};
  return result;
}

__device__ __forceinline__ void WarpShuffleAccumulate(Se3NormalEquation& value,
                                                      int offset) {
  value.h00 += __shfl_down_sync(kFullWarpMask, value.h00, offset);
  value.h01 += __shfl_down_sync(kFullWarpMask, value.h01, offset);
  value.h02 += __shfl_down_sync(kFullWarpMask, value.h02, offset);
  value.h03 += __shfl_down_sync(kFullWarpMask, value.h03, offset);
  value.h04 += __shfl_down_sync(kFullWarpMask, value.h04, offset);
  value.h05 += __shfl_down_sync(kFullWarpMask, value.h05, offset);
  value.h06 += __shfl_down_sync(kFullWarpMask, value.h06, offset);

  value.h11 += __shfl_down_sync(kFullWarpMask, value.h11, offset);
  value.h12 += __shfl_down_sync(kFullWarpMask, value.h12, offset);
  value.h13 += __shfl_down_sync(kFullWarpMask, value.h13, offset);
  value.h14 += __shfl_down_sync(kFullWarpMask, value.h14, offset);
  value.h15 += __shfl_down_sync(kFullWarpMask, value.h15, offset);
  value.h16 += __shfl_down_sync(kFullWarpMask, value.h16, offset);

  value.h22 += __shfl_down_sync(kFullWarpMask, value.h22, offset);
  value.h23 += __shfl_down_sync(kFullWarpMask, value.h23, offset);
  value.h24 += __shfl_down_sync(kFullWarpMask, value.h24, offset);
  value.h25 += __shfl_down_sync(kFullWarpMask, value.h25, offset);
  value.h26 += __shfl_down_sync(kFullWarpMask, value.h26, offset);

  value.h33 += __shfl_down_sync(kFullWarpMask, value.h33, offset);
  value.h34 += __shfl_down_sync(kFullWarpMask, value.h34, offset);
  value.h35 += __shfl_down_sync(kFullWarpMask, value.h35, offset);
  value.h36 += __shfl_down_sync(kFullWarpMask, value.h36, offset);

  value.h44 += __shfl_down_sync(kFullWarpMask, value.h44, offset);
  value.h45 += __shfl_down_sync(kFullWarpMask, value.h45, offset);
  value.h46 += __shfl_down_sync(kFullWarpMask, value.h46, offset);

  value.h55 += __shfl_down_sync(kFullWarpMask, value.h55, offset);
  value.h56 += __shfl_down_sync(kFullWarpMask, value.h56, offset);

  value.squared_error +=
      __shfl_down_sync(kFullWarpMask, value.squared_error, offset);
  value.correspondence_count +=
      __shfl_down_sync(kFullWarpMask, value.correspondence_count, offset);
}

__device__ __forceinline__ Se3NormalEquation
ReduceWarp(Se3NormalEquation value) {
  for (int offset = warpSize / 2; offset >= 1; offset >>= 1) {
    WarpShuffleAccumulate(value, offset);
  }

  return value;
}

__device__ __forceinline__ Se3NormalEquation
ReduceBlock(Se3NormalEquation value) {
  constexpr int kMaxWarpsPerBlock = 32;

  __shared__ Se3NormalEquation warp_sums[kMaxWarpsPerBlock];

  const int lane_id = threadIdx.x & (warpSize - 1);
  const int warp_id = threadIdx.x / warpSize;

  value = ReduceWarp(value);

  if (lane_id == 0) {
    warp_sums[warp_id] = value;
  }

  __syncthreads();

  Se3NormalEquation block_value = MakeZeroNormalEquation();

  if (warp_id == 0 && lane_id < (blockDim.x + warpSize - 1) / warpSize) {
    block_value = warp_sums[lane_id];
  }

  if (warp_id == 0) {
    block_value = ReduceWarp(block_value);
  }

  return block_value;
}

__global__ void ReduceNormalEquationsKernel(const Se3NormalEquation* input,
                                            Se3NormalEquation* output,
                                            int count) {
  Se3NormalEquation local_sum = MakeZeroNormalEquation();

  const int global_stride = blockDim.x * gridDim.x;

  for (int index = blockIdx.x * blockDim.x + threadIdx.x; index < count;
       index += global_stride) {
    local_sum.Accumulate(input[index]);
  }

  const Se3NormalEquation reduced_sum = ReduceBlock(local_sum);

  if (threadIdx.x == 0) {
    output[blockIdx.x] = reduced_sum;
  }
}

struct GpuIcpNormalEquationFunctor {
  Matrix3f current_rotation;
  float3 current_translation;

  Matrix3f sensor_to_imu_rotation;
  float3 sensor_to_imu_translation;

  Matrix3f previous_inverse_rotation;
  float3 previous_translation;

  PtrSz<PointXYZRGB> source_points;
  PtrSz<PointXYZRGBNormal> target_points;

  float distance_threshold;
  float angle_threshold;

  int point_count;

  Se3NormalEquation* block_results;

  __device__ __forceinline__ Se3NormalEquation EvaluatePoint(int index) const {
    const PointXYZRGB& source = source_points.data[index];
    const PointXYZRGBNormal& target = target_points.data[index];

    float3 source_point = make_float3(source.x, source.y, source.z);

    // Transform source point into the current world frame.
    float3 transformed_point =
        sensor_to_imu_rotation * source_point + sensor_to_imu_translation;

    transformed_point =
        current_rotation * transformed_point + current_translation;

    const float3 target_point = make_float3(target.x, target.y, target.z);

    const float3 target_normal =
        make_float3(target.normal_x, target.normal_y, target.normal_z);

    const float normal_length = norm(target_normal);
    const float point_distance = norm(target_point - transformed_point);

    Se3NormalEquation result = MakeZeroNormalEquation();

    if (point_distance > distance_threshold || normal_length <= 0.1f) {
      return result;
    }

    const float3 source_previous =
        previous_inverse_rotation * (transformed_point - previous_translation);

    const float3 target_previous =
        previous_inverse_rotation * (target_point - previous_translation);

    const float3 normal_previous = previous_inverse_rotation * target_normal;


    const float3 rotational_part = cross(source_previous, normal_previous);

    const float residual =
        dot(normal_previous, source_previous - target_previous);

    constexpr float kHuberThreshold = 0.1f;

    const float absolute_residual = fabsf(residual);

    float weight = 1.0f;

    if (absolute_residual > kHuberThreshold) {
      weight = rsqrtf(sqrtf(absolute_residual / kHuberThreshold));
    }

    const float j0 = weight * normal_previous.x;
    const float j1 = weight * normal_previous.y;
    const float j2 = weight * normal_previous.z;

    const float j3 = weight * rotational_part.x;
    const float j4 = weight * rotational_part.y;
    const float j5 = weight * rotational_part.z;

    const float weighted_residual = weight * residual;

    result.h00 = j0 * j0;
    result.h01 = j0 * j1;
    result.h02 = j0 * j2;
    result.h03 = j0 * j3;
    result.h04 = j0 * j4;
    result.h05 = j0 * j5;
    result.h06 = j0 * weighted_residual;

    result.h11 = j1 * j1;
    result.h12 = j1 * j2;
    result.h13 = j1 * j3;
    result.h14 = j1 * j4;
    result.h15 = j1 * j5;
    result.h16 = j1 * weighted_residual;

    result.h22 = j2 * j2;
    result.h23 = j2 * j3;
    result.h24 = j2 * j4;
    result.h25 = j2 * j5;
    result.h26 = j2 * weighted_residual;

    result.h33 = j3 * j3;
    result.h34 = j3 * j4;
    result.h35 = j3 * j5;
    result.h36 = j3 * weighted_residual;

    result.h44 = j4 * j4;
    result.h45 = j4 * j5;
    result.h46 = j4 * weighted_residual;

    result.h55 = j5 * j5;
    result.h56 = j5 * weighted_residual;

    result.squared_error = weighted_residual * weighted_residual;
    result.correspondence_count = 1.0f;

    return result;
  }

  __device__ __forceinline__ void operator()() const {
    Se3NormalEquation thread_sum = MakeZeroNormalEquation();

    const int stride = blockDim.x * gridDim.x;

    for (int index = blockIdx.x * blockDim.x + threadIdx.x; index < point_count;
         index += stride) {
      thread_sum.Accumulate(EvaluatePoint(index));
    }

    const Se3NormalEquation block_sum = ReduceBlock(thread_sum);

    if (threadIdx.x == 0) {
      block_results[blockIdx.x] = block_sum;
    }
  }
};

__global__ void ComputeIcpNormalEquationsKernel(
    GpuIcpNormalEquationFunctor functor) {
  functor();
}

}  // namespace

void ComputeICPNormalEquations(
    const Matrix3f& Rcurr, const float3& tcurr, const Matrix3f& R_imu_sta,
    const float3& t_imu_sta, const Matrix3f& Rprev_inv, const float3& tprev,
    const PtrSz<PointXYZRGB> cloud_in_hor,
    const PtrSz<PointXYZRGBNormal> cloud_cast_hor, float distance_threshold,
    float angle_threshold, DeviceArray<Se3NormalEquation>& sum,
    DeviceArray<Se3NormalEquation>& out, float* matrixA_host,
    float* vectorB_host, float* residual_host, int threads, int blocks,
    int cloud_number_hor) {
  GpuIcpNormalEquationFunctor functor{};

  functor.current_rotation = Rcurr;
  functor.current_translation = tcurr;

  functor.sensor_to_imu_rotation = R_imu_sta;
  functor.sensor_to_imu_translation = t_imu_sta;

  functor.previous_inverse_rotation = Rprev_inv;
  functor.previous_translation = tprev;

  functor.source_points = cloud_in_hor;
  functor.target_points = cloud_cast_hor;

  functor.distance_threshold = distance_threshold;
  functor.angle_threshold = angle_threshold;

  functor.point_count = cloud_number_hor;
  functor.block_results = sum;

  ComputeIcpNormalEquationsKernel<<<blocks, threads>>>(functor);

  ReduceNormalEquationsKernel<<<1, MAX_THREADS>>>(sum, out, blocks);

  cudaSafeCall(cudaGetLastError());

  Se3NormalEquation host_result{};
  out.download(&host_result);

  matrixA_host[0] = host_result.h00;
  matrixA_host[1] = host_result.h01;
  matrixA_host[2] = host_result.h02;
  matrixA_host[3] = host_result.h03;
  matrixA_host[4] = host_result.h04;
  matrixA_host[5] = host_result.h05;

  matrixA_host[6] = host_result.h01;
  matrixA_host[7] = host_result.h11;
  matrixA_host[8] = host_result.h12;
  matrixA_host[9] = host_result.h13;
  matrixA_host[10] = host_result.h14;
  matrixA_host[11] = host_result.h15;

  matrixA_host[12] = host_result.h02;
  matrixA_host[13] = host_result.h12;
  matrixA_host[14] = host_result.h22;
  matrixA_host[15] = host_result.h23;
  matrixA_host[16] = host_result.h24;
  matrixA_host[17] = host_result.h25;

  matrixA_host[18] = host_result.h03;
  matrixA_host[19] = host_result.h13;
  matrixA_host[20] = host_result.h23;
  matrixA_host[21] = host_result.h33;
  matrixA_host[22] = host_result.h34;
  matrixA_host[23] = host_result.h35;

  matrixA_host[24] = host_result.h04;
  matrixA_host[25] = host_result.h14;
  matrixA_host[26] = host_result.h24;
  matrixA_host[27] = host_result.h34;
  matrixA_host[28] = host_result.h44;
  matrixA_host[29] = host_result.h45;

  matrixA_host[30] = host_result.h05;
  matrixA_host[31] = host_result.h15;
  matrixA_host[32] = host_result.h25;
  matrixA_host[33] = host_result.h35;
  matrixA_host[34] = host_result.h45;
  matrixA_host[35] = host_result.h55;

  vectorB_host[0] = host_result.h06;
  vectorB_host[1] = host_result.h16;
  vectorB_host[2] = host_result.h26;
  vectorB_host[3] = host_result.h36;
  vectorB_host[4] = host_result.h46;
  vectorB_host[5] = host_result.h56;

  residual_host[0] = host_result.squared_error;
  residual_host[1] = host_result.correspondence_count;
}
