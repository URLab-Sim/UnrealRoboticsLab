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
    CACHE PATH "Root of a prebuilt MuJoCo (contains include/ and build/)")

set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT}/include")
set(MUJOCO_IMPORT_LIB  "${MUJOCO_ROOT}/build/lib/Release/mujoco.lib")
set(MUJOCO_DLL         "${MUJOCO_ROOT}/build/bin/Release/mujoco.dll")

# The first-party engine-plugin DLLs (elasticity/sdf/sensor/actuator) are
# prebuilt beside mujoco.dll; this is the default directory the plugin registry
# loads them from (overridable per tool via --plugin-dir or PROTOSPEC_PLUGIN_DIR).
set(MUJOCO_PLUGIN_DIR "${MUJOCO_ROOT}/build/bin/Release")

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

# The mjModel comparison core, shared by the mj_model_diff CLI and the
# ps_native_diff three-way harness (impl-plan T0.2). MuJoCo only.
add_library(model_diff_lib STATIC model_diff_lib.cc)
target_include_directories(model_diff_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  "${MUJOCO_INCLUDE_DIR}")
target_link_libraries(model_diff_lib PUBLIC "${MUJOCO_IMPORT_LIB}")
if(MSVC)
  target_compile_options(model_diff_lib PRIVATE /W4 /permissive-)
else()
  target_compile_options(model_diff_lib PRIVATE -Wall -Wextra)
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
