# Copyright 2026 The Eternelle Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 2.6.0)

project(Scan2Mesh)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -O3)
add_link_options("-Wl,-rpath,$ORIGIN")
link_directories("${CMAKE_CURRENT_BINARY_DIR}")

find_package(CUDA REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost COMPONENTS thread serialization REQUIRED)
find_package(embree 4 REQUIRED)
find_package(urdfdom REQUIRED)
find_package(PCL 1.9 EXACT REQUIRED PATHS "$ENV{HOME}/software/pcl_install")

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${urdfdom_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

file(GLOB containers cuda/containers/*.cpp)
file(GLOB mesh_icp_cuda cuda/*.cu)
file(GLOB utils_srcs utils/*.cpp utils/*.h)
file(GLOB mesh_tracking_srcs mesh_tracking/*.cpp mesh_tracking/*.h*)

set(CUDA_ARCH_BIN "60 61 62 70 72 75 80 86"
  CACHE STRING
  "CUDA GPU architectures to generate native binaries for")
list(APPEND CUDA_NVCC_FLAGS
  "-Xcompiler"
  "-fPIC"
  "--ftz=true"
  "--prec-div=false"
  "--prec-sqrt=false"
  "--disable-warnings"
)
CUDA_COMPILE(mesh_icp_cuda_objs ${mesh_icp_cuda})

add_executable(mesh_raycast
  mesh_raycast/mesh_raycast_app.cpp ${utils_srcs}
)

target_link_libraries(mesh_raycast
  ${EMBREE_LIBRARIES}
  ${PCL_LIBRARIES}
  urdfdom_model
)

add_executable(mesh_tracking
  ${mesh_tracking_srcs}
  ${utils_srcs}
  ${mesh_icp_cuda}
  ${mesh_icp_cuda}
  ${mesh_icp_cuda_objs}
  ${containers}
)

target_link_libraries(mesh_tracking
  ${EMBREE_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  urdfdom_model
)
