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

cmake_minimum_required(VERSION 3.16.3)
project(Scan2Mesh LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SCAN2MESH_TRACKING_BACKEND "CPU" CACHE STRING "ICP backend: CPU or CUDA")
set_property(CACHE SCAN2MESH_TRACKING_BACKEND PROPERTY STRINGS CPU CUDA)
option(SCAN2MESH_BUILD_VISUALIZATION "Build the Pangolin tracking viewer" OFF)
if(NOT SCAN2MESH_TRACKING_BACKEND MATCHES "^(CPU|CUDA)$")
  message(FATAL_ERROR "SCAN2MESH_TRACKING_BACKEND must be CPU or CUDA")
endif()

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(embree 4 REQUIRED)
find_package(Boost CONFIG REQUIRED COMPONENTS thread)
find_package(urdfdom REQUIRED CONFIG)

add_library(scan2mesh_core
  utils/ply_reader.cpp utils/tinyply.cpp
  mesh_tracking/mesh_raycaster.cpp mesh_tracking/scan2mesh_icp.cpp
  mesh_tracking/voxel_filter.cpp)
target_include_directories(scan2mesh_core PUBLIC "${PROJECT_SOURCE_DIR}")
target_link_libraries(scan2mesh_core PUBLIC Eigen3::Eigen embree)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(scan2mesh_core PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra>)
endif()

if(SCAN2MESH_TRACKING_BACKEND STREQUAL "CUDA")
  enable_language(CUDA)
  if(CMAKE_VERSION VERSION_LESS 3.17)
    find_package(CUDA REQUIRED)
    target_include_directories(scan2mesh_core PUBLIC ${CUDA_INCLUDE_DIRS})
    target_link_libraries(scan2mesh_core PUBLIC ${CUDA_CUDART_LIBRARY})
  else()
    find_package(CUDAToolkit REQUIRED)
    target_link_libraries(scan2mesh_core PUBLIC CUDA::cudart)
  endif()
  # The kernels need C++14; CMake only gained CUDA C++17 support in 3.18.
  set_target_properties(scan2mesh_core PROPERTIES CUDA_STANDARD 14 CUDA_STANDARD_REQUIRED ON)
  if(CMAKE_VERSION VERSION_LESS 3.18 AND CMAKE_CUDA_ARCHITECTURES)
    # Honor the same numeric architecture list on older CMake versions.
    foreach(architecture IN LISTS CMAKE_CUDA_ARCHITECTURES)
      if(NOT architecture MATCHES "^[0-9]+$")
        message(FATAL_ERROR "CMake before 3.18 requires numeric CUDA architectures, e.g. 75;86")
      endif()
      target_compile_options(scan2mesh_core PRIVATE
        $<$<COMPILE_LANGUAGE:CUDA>:--generate-code=arch=compute_${architecture},code=sm_${architecture}>)
    endforeach()
  endif()
  target_sources(scan2mesh_core PRIVATE cuda/icp.cu
    cuda/containers/device_memory.cpp cuda/containers/initialization.cpp)
  target_compile_definitions(scan2mesh_core PUBLIC SCAN2MESH_USE_CUDA)
endif()

add_executable(mesh_raycast mesh_raycast/mesh_raycast_app.cpp)
target_link_libraries(mesh_raycast PRIVATE scan2mesh_core)

add_executable(mesh_tracking mesh_tracking/tracking_app.cpp
  mesh_tracking/scan2mesh_controller.cpp mesh_tracking/tracker_interface.cpp
  utils/urdf_loader.cpp)
target_include_directories(mesh_tracking PRIVATE ${urdfdom_INCLUDE_DIRS})
target_link_libraries(mesh_tracking PRIVATE scan2mesh_core Boost::thread ${urdfdom_LIBRARIES})
if(SCAN2MESH_BUILD_VISUALIZATION)
  find_package(Pangolin REQUIRED)
  target_sources(mesh_tracking PRIVATE mesh_tracking/mesh_tracking_visualizer.cpp)
  target_compile_definitions(mesh_tracking PRIVATE SCAN2MESH_BUILD_VISUALIZATION)
  target_include_directories(mesh_tracking PRIVATE ${Pangolin_INCLUDE_DIRS})
  target_link_libraries(mesh_tracking PRIVATE ${Pangolin_LIBRARIES})
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(scan2mesh_tests tests/core_tests.cpp)
  target_link_libraries(scan2mesh_tests PRIVATE scan2mesh_core)
  add_test(NAME scan2mesh_core COMMAND scan2mesh_tests)
  add_test(NAME scan2mesh_cli COMMAND ${CMAKE_COMMAND}
    -DTRACKER=$<TARGET_FILE:mesh_tracking> -DRAYCASTER=$<TARGET_FILE:mesh_raycast>
    -DSOURCE_DIR=${PROJECT_SOURCE_DIR} -DTEST_DIR=${PROJECT_BINARY_DIR}
    -P ${PROJECT_SOURCE_DIR}/tests/tracking_smoke.cmake)
endif()
message(STATUS "Scan2Mesh ICP backend: ${SCAN2MESH_TRACKING_BACKEND}")
