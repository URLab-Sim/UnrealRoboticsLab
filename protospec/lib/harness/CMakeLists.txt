cmake_minimum_required(VERSION 3.20)

# Buildable both standalone (`cmake -S protospec/lib/harness`) and as a subdirectory of
# the root project (which does add_subdirectory(harness) once this file exists).
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(protospec_harness CXX)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

set(MUJOCO_ROOT ""
    CACHE PATH "Root of a prebuilt MuJoCo (a source build tree, or a staged install)")

# Two tree shapes reach this. MuJoCo's own build tree keeps the import library
# and the runtime under build/lib/Release and build/bin/Release; the install
# tree that third_party/MuJoCo/build.{ps1,sh} stages under
# third_party/install/MuJoCo keeps them in lib/ and bin/. URLab's
# protospec/build.{ps1,sh} points at the second, so the caller is not required
# to know which shape they hold.
set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT}/include")
if(EXISTS "${MUJOCO_ROOT}/build/lib/Release")
  set(MUJOCO_LIB_DIR "${MUJOCO_ROOT}/build/lib/Release")
  set(MUJOCO_BIN_DIR "${MUJOCO_ROOT}/build/bin/Release")
else()
  set(MUJOCO_LIB_DIR "${MUJOCO_ROOT}/lib")
  set(MUJOCO_BIN_DIR "${MUJOCO_ROOT}/bin")
endif()
if(WIN32)
  set(MUJOCO_IMPORT_LIB "${MUJOCO_LIB_DIR}/mujoco.lib")
  set(MUJOCO_DLL        "${MUJOCO_BIN_DIR}/mujoco.dll")
else()
  # The unversioned .so is the symlink; linking it records the SONAME rather
  # than an absolute versioned path.
  set(MUJOCO_IMPORT_LIB "${MUJOCO_LIB_DIR}/libmujoco.so")
  set(MUJOCO_DLL        "${MUJOCO_IMPORT_LIB}")
endif()

# The first-party engine-plugin DLLs (elasticity/sdf/sensor/actuator) are
# prebuilt beside mujoco.dll; this is the default directory the plugin registry
# loads them from (overridable per tool via --plugin-dir or PROTOSPEC_PLUGIN_DIR).
set(MUJOCO_PLUGIN_DIR "${MUJOCO_BIN_DIR}")

if(NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mjxmacro.h")
  message(FATAL_ERROR
    "MuJoCo headers not found under ${MUJOCO_INCLUDE_DIR}; set -DMUJOCO_ROOT=...")
endif()
if(NOT EXISTS "${MUJOCO_IMPORT_LIB}")
  message(FATAL_ERROR
    "mujoco.lib not found at ${MUJOCO_IMPORT_LIB}; set -DMUJOCO_ROOT=...")
endif()

# First-party engine-plugin registration, shared by every tool that loads
# plugin-bearing corpus models (mj_model_diff, ps_native_diff, ps_compile).
# MuJoCo's plugin registry is process-global, so one small lift-free lib linked
# into each tool is the single registration point. Guarded so a parent project
# that has already defined it (or a re-run) does not redefine the target.
if(NOT TARGET plugin_registry)
  add_library(plugin_registry STATIC plugin_registry.cc)
  target_include_directories(plugin_registry PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    "${MUJOCO_INCLUDE_DIR}")
  target_link_libraries(plugin_registry PUBLIC "${MUJOCO_IMPORT_LIB}")
  target_compile_definitions(plugin_registry PRIVATE
    "PROTOSPEC_DEFAULT_PLUGIN_DIR=\"${MUJOCO_PLUGIN_DIR}\"")
  if(MSVC)
    target_compile_options(plugin_registry PRIVATE /W4 /permissive-)
  else()
    target_compile_options(plugin_registry PRIVATE -Wall -Wextra)
  endif()
endif()

# The mjModel comparison core, shared by the mj_model_diff CLI, the
# ps_native_diff three-way harness (impl-plan T0.2), and URLab's compile-parity
# goldens, which link the staged protospec_harness library and call
# ps::harness::DiffModels directly. MuJoCo only.
#
# `model_diff_lib` remains as an alias so the tools below, which name the
# comparison core rather than the staged artifact, keep reading as they did.
add_library(protospec_harness STATIC model_diff_lib.cc)
add_library(model_diff_lib ALIAS protospec_harness)
target_include_directories(protospec_harness PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  "${MUJOCO_INCLUDE_DIR}")
target_link_libraries(protospec_harness PUBLIC "${MUJOCO_IMPORT_LIB}")
if(MSVC)
  target_compile_options(protospec_harness PRIVATE /W4 /permissive-)
else()
  target_compile_options(protospec_harness PRIVATE -Wall -Wextra)
endif()

add_executable(mj_model_diff mj_model_diff.cc)
target_include_directories(mj_model_diff PRIVATE "${MUJOCO_INCLUDE_DIR}")
target_link_libraries(mj_model_diff PRIVATE model_diff_lib plugin_registry "${MUJOCO_IMPORT_LIB}")

if(MSVC)
  target_compile_options(mj_model_diff PRIVATE /W4 /permissive-)
else()
  target_compile_options(mj_model_diff PRIVATE -Wall -Wextra)
endif()

# The runtime DLL must sit next to the exe so it loads without PATH tweaks.
add_custom_command(TARGET mj_model_diff POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${MUJOCO_DLL}" "$<TARGET_FILE_DIR:mj_model_diff>"
  VERBATIM)
