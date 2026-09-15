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

#ifndef VECTOR_MATH_HPP_
#define VECTOR_MATH_HPP_

#define DEFINE_VEC3_COMPOUND_OPERATOR(type, scalar_type, op)               \
  __host__ __device__ __forceinline__ type& operator op(type& lhs,         \
                                                        const type& rhs) { \
    lhs.x op rhs.x;                                                        \
    lhs.y op rhs.y;                                                        \
    lhs.z op rhs.z;                                                        \
    return lhs;                                                            \
  }                                                                        \
                                                                           \
  __host__ __device__ __forceinline__ type& operator op(type& lhs,         \
                                                        scalar_type rhs) { \
    lhs.x op rhs;                                                          \
    lhs.y op rhs;                                                          \
    lhs.z op rhs;                                                          \
    return lhs;                                                            \
  }

DEFINE_VEC3_COMPOUND_OPERATOR(float3, float, +=)
DEFINE_VEC3_COMPOUND_OPERATOR(float3, float, -=)
DEFINE_VEC3_COMPOUND_OPERATOR(float3, float, *=)

DEFINE_VEC3_COMPOUND_OPERATOR(short3, short, -=)

DEFINE_VEC3_COMPOUND_OPERATOR(int3, int, +=)

#undef DEFINE_VEC3_COMPOUND_OPERATOR

#define DEFINE_VEC3_BINARY_OPERATOR(type, scalar_type, op, compound_op)   \
  __host__ __device__ __forceinline__ type operator op(const type& lhs,   \
                                                       const type& rhs) { \
    type result = lhs;                                                    \
    result compound_op rhs;                                               \
    return result;                                                        \
  }                                                                       \
                                                                          \
  __host__ __device__ __forceinline__ type operator op(const type& lhs,   \
                                                       scalar_type rhs) { \
    type result = lhs;                                                    \
    result compound_op rhs;                                               \
    return result;                                                        \
  }

DEFINE_VEC3_BINARY_OPERATOR(float3, float, +, +=)
DEFINE_VEC3_BINARY_OPERATOR(float3, float, -, -=)
DEFINE_VEC3_BINARY_OPERATOR(float3, float, *, *=)

DEFINE_VEC3_BINARY_OPERATOR(short3, short, -, -=)

DEFINE_VEC3_BINARY_OPERATOR(int3, int, +, +=)

#undef DEFINE_VEC3_BINARY_OPERATOR


__host__ __device__ __forceinline__ float dot(const float3& lhs,
                                              const float3& rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

__host__ __device__ __forceinline__ float3 cross(const float3& lhs,
                                                 const float3& rhs) {
  return make_float3(lhs.y * rhs.z - lhs.z * rhs.y,
                     lhs.z * rhs.x - lhs.x * rhs.z,
                     lhs.x * rhs.y - lhs.y * rhs.x);
}

template <typename VectorType>
__host__ __device__ __forceinline__ float squared_norm(
    const VectorType& vector) {
  return dot(vector, vector);
}

template <typename VectorType>
__host__ __device__ __forceinline__ float norm(const VectorType& vector) {
  return sqrtf(squared_norm(vector));
}

template <typename VectorType>
__host__ __device__ __forceinline__ float inverse_norm(
    const VectorType& vector) {
  return rsqrtf(squared_norm(vector));
}

template <typename VectorType>
__host__ __device__ __forceinline__ VectorType
normalized(const VectorType& vector) {
  return vector * inverse_norm(vector);
}

template <typename VectorType>
__host__ __device__ __forceinline__ VectorType
normalized_safe(const VectorType& vector) {
  const float squared_length = squared_norm(vector);

  return squared_length > 0.0f ? vector * rsqrtf(squared_length) : vector;
}

__device__ __forceinline__ float3 operator*(const Matrix3f& matrix,
                                            const float3& vector) {
  return make_float3(dot(matrix.data[0], vector), dot(matrix.data[1], vector),
                     dot(matrix.data[2], vector));
}

#endif  // VECTOR_MATH_HPP_