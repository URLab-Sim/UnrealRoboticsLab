# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Standalone (non-colcon) CMake project that builds the shared UrlabRclCore
# against a user-installed ROS 2, together with a self-verifying harness. This
# exists so the rcl-facing C++ can be built and tested OUTSIDE Unreal, and so the
# unproven Windows in-process leg can be validated before any UE wiring starts.
#
# It consumes the ROS 2 installation the user has sourced; it does not build any
# ROS package. The same UrlabRclCore.cpp is later compiled into the URLab module
# by UBT with UE's own toolchain.

# URLAB_LIBCXX switches to clang + libc++ for the Linux toolchain smoke build.
# It must be handled before project() so the compiler selection takes effect.
# It is a Linux-only concern: on Windows both UE and ROS 2 use MSVC, so there is
# no libc++/libstdc++ ABI mix to pre-validate.
if(URLAB_LIBCXX)
    set(CMAKE_C_COMPILER clang)
    set(CMAKE_CXX_COMPILER clang++)
endif()

cmake_minimum_required(VERSION 3.16)
project(urlab_rcl_test CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(URLAB_LIBCXX)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-stdlib=libc++>)
    add_link_options(-stdlib=libc++)
endif()

# Consume the sourced ROS 2 install via modern imported targets. Each package's
# find_package defines namespaced CMake targets (rcl::rcl,
# <pkg>::<pkg>__rosidl_typesupport_c, ...) whose INTERFACE include dirs and link
# libraries propagate transitively, so we link with plain target_link_libraries
# rather than ament_target_dependencies (that ament_cmake macro is not available
# in this ROS 2 build). This also mirrors how the UE Build.cs will link these
# libraries explicitly, without ament.
find_package(rcl REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

# The shared core, reached by relative path up into the plugin module tree.
set(URLAB_RCL_CORE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Source/URLab/Private/Ros/UrlabRclCore.cpp")

add_executable(urlab_rcl_test
    src/test_main.cpp
    "${URLAB_RCL_CORE}")

target_include_directories(urlab_rcl_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Source/URLab/Private/Ros")

# The standalone build is the one place URLAB_WITH_ROS2 is defined outside UBT,
# so the fenced core body compiles here for fast iteration and testing. The
# distro name is the compile-time pin reported by UrlabRcl_DistroName(); it is
# set here (rather than in the core source) so the core stays distro-agnostic.
target_compile_definitions(urlab_rcl_test PRIVATE
    URLAB_WITH_ROS2=1
    URLAB_ROS_DISTRO_NAME="lyrical")

target_link_libraries(urlab_rcl_test PRIVATE
    rcl::rcl
    rosidl_runtime_c::rosidl_runtime_c
    builtin_interfaces::builtin_interfaces__rosidl_typesupport_c
    std_msgs::std_msgs__rosidl_typesupport_c
    geometry_msgs::geometry_msgs__rosidl_typesupport_c
    sensor_msgs::sensor_msgs__rosidl_typesupport_c
    tf2_msgs::tf2_msgs__rosidl_typesupport_c
    rosgraph_msgs::rosgraph_msgs__rosidl_typesupport_c)

# winmm provides timeBeginPeriod/timeEndPeriod, used by the harness publish loop
# to get 1 ms timer resolution so the 50 Hz cadence is accurate on Windows.
if(WIN32)
    target_link_libraries(urlab_rcl_test PRIVATE winmm)
endif()
