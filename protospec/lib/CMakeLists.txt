cmake_minimum_required(VERSION 3.20)
project(protospec CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The authoring resolver uses MuJoCo's public orientation API. Default to the
# same staged runtime as URLab; build scripts may override MUJOCO_ROOT.
set(MUJOCO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/install/MuJoCo"
    CACHE PATH "Root of a prebuilt MuJoCo (a source build tree, or a staged install)")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT}/include")
if(EXISTS "${MUJOCO_ROOT}/build/lib/Release")
  set(MUJOCO_LIB_DIR "${MUJOCO_ROOT}/build/lib/Release")
else()
  set(MUJOCO_LIB_DIR "${MUJOCO_ROOT}/lib")
endif()
if(WIN32)
  set(MUJOCO_IMPORT_LIB "${MUJOCO_LIB_DIR}/mujoco.lib")
else()
  set(MUJOCO_IMPORT_LIB "${MUJOCO_LIB_DIR}/libmujoco.so")
endif()
if(NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h" OR
   NOT EXISTS "${MUJOCO_IMPORT_LIB}")
  message(FATAL_ERROR "ProtoSpec needs a built MuJoCo at MUJOCO_ROOT=${MUJOCO_ROOT}")
endif()

add_library(tinyxml2 STATIC third_party/tinyxml2/tinyxml2.cpp)
target_include_directories(tinyxml2 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyxml2
)

# Core resolvers: public MuJoCo orientation conversion plus the retained
# full-inertia conversion. The reader calls these at parse end.
add_library(protospec_core STATIC core/resolve.cc)
target_include_directories(protospec_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/core)
target_include_directories(protospec_core PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(protospec_core PUBLIC "${MUJOCO_IMPORT_LIB}")
if(MSVC)
  target_compile_options(protospec_core PRIVATE /W4 /permissive-)
else()
  target_compile_options(protospec_core PRIVATE -Wall -Wextra)
endif()

# Source-aware MJCF include expansion used by the direct UE XML importer.
add_library(protospec_mjcf STATIC io/include.cc)
target_include_directories(protospec_mjcf PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/io
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyxml2
)
target_link_libraries(protospec_mjcf PUBLIC tinyxml2)
if(MSVC)
  target_compile_options(protospec_mjcf PRIVATE /W4 /permissive-)
else()
  target_compile_options(protospec_mjcf PRIVATE -Wall -Wextra)
endif()

# Test-only native mjModel comparison helper; no plain document dependency.
add_subdirectory(harness)

# Editor-only executable: upstream's recursive dependency scanner can call
# mju_error on a missing candidate. Its process boundary protects the editor.
add_executable(urlab_mj_deps tools/mj_deps.cpp)
target_include_directories(urlab_mj_deps PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(urlab_mj_deps PRIVATE "${MUJOCO_IMPORT_LIB}")
if(UNIX AND NOT APPLE)
  # The installed helper sits beside the editor's staged libmujoco; its
  # ProtoSpec install copy can find the sibling MuJoCo install. Never embed
  # the developer's absolute install path in a helper that may be packaged.
  # The UE-compatible probe cache supplies one for probes; this target uses
  # only its relative install paths after the helper is staged.
  string(REGEX REPLACE "-Wl,-rpath,[^ ]+" ""
    CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
  set_target_properties(urlab_mj_deps PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "\$ORIGIN;\$ORIGIN/../../MuJoCo/lib")
endif()
