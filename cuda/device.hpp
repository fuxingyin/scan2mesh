#ifndef DEVICE_HPP_
#define DEVICE_HPP_

#include "internal.h"
#include "cuda_vector_math.cuh"

#define FULL_MASK 0xFFFFFFFF

template <class T>
__device__ __host__ __forceinline__ void swap(T& a, T& b) {
  T c(a);
  a = b;
  b = c;
}

#endif
