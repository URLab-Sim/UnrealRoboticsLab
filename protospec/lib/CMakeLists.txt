cmake_minimum_required(VERSION 3.20)
project(protospec CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# MuJoCo-free static library: the generated object model + handwritten core.
add_library(protospec STATIC
  generated/types.cc
  generated/reflect.cc
  generated/keywords.cc
  generated/defaults.cc
  generated/xml_binding.cc
)
target_include_directories(protospec PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/generated
)

if(MSVC)
  target_compile_options(protospec PRIVATE /W4 /permissive-)
else()
  target_compile_options(protospec PRIVATE -Wall -Wextra)
endif()

# Vendored tinyxml2 (zlib license). Third-party: built with default warnings so
# the project's /W4 does not flag it. Only the MJCF IO libraries link it; the
# core stays dependency-free.
add_library(tinyxml2 STATIC third_party/tinyxml2/tinyxml2.cpp)
target_include_directories(tinyxml2 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyxml2
)

# Core resolvers: orientation + inertia canonicalization, a MuJoCo-free
# translation unit of lifted math. The reader folds authored orientation and
# inertia spellings into their canonical quat/diaginertia forms at parse end
# through this library, so the MJCF IO stays MuJoCo-free (no mujoco.h, no
# mujoco.dll runtime dependency for ps_roundtrip).
add_library(protospec_core STATIC core/resolve.cc)
target_include_directories(protospec_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/core)
target_link_libraries(protospec_core PUBLIC protospec)
if(MSVC)
  target_compile_options(protospec_core PRIVATE /W4 /permissive-)
else()
  target_compile_options(protospec_core PRIVATE -Wall -Wextra)
endif()

# MJCF IO, profile-independent half: everything keyed on ElementType, raw XML or
# diagnostics, plus numeric formatting and <include> expansion. This is the half
# URLab links; it instantiates the templated reader and writer for its own two
# document profiles in its own translation units.
add_library(protospec_mjcf STATIC
  io/numeric.cc
  io/include.cc
  io/mjcf_common.cc
)
target_include_directories(protospec_mjcf PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/io
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk
)
target_link_libraries(protospec_mjcf PUBLIC protospec protospec_core PRIVATE tinyxml2)
if(MSVC)
  target_compile_options(protospec_mjcf PRIVATE /W4 /permissive-)
else()
  target_compile_options(protospec_mjcf PRIVATE -Wall -Wextra)
endif()

# The plain profile: the reader and writer instantiated over the generated value
# types. It is a TEST FIXTURE, not shipped surface -- it exists so the retained
# reader and writer can be exercised over a corpus without booting Unreal. It is
# neither staged nor linked by URLab, which brings its own profiles.
add_library(protospec_io_plain STATIC
  io/mjcf_reader_plain.cc
  io/mjcf_writer_plain.cc
  io/default_classes.cc
)
target_include_directories(protospec_io_plain PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)
target_link_libraries(protospec_io_plain PUBLIC protospec_mjcf PRIVATE tinyxml2)
if(MSVC)
  # /bigobj: the reader TU's per-element template instantiations exceed the COFF
  # section limit without it.
  target_compile_options(protospec_io_plain PRIVATE /W4 /permissive- /bigobj)
else()
  target_compile_options(protospec_io_plain PRIVATE -Wall -Wextra)
endif()

# Round-trip tool: the harness contract (ps_roundtrip <in.xml>).
add_executable(ps_roundtrip tools/ps_roundtrip.cc)
target_link_libraries(ps_roundtrip PRIVATE protospec_io_plain)

enable_testing()
add_executable(protospec_tests test/test_model.cc)
target_link_libraries(protospec_tests PRIVATE protospec)
if(MSVC)
  target_compile_options(protospec_tests PRIVATE /W4 /permissive-)
else()
  target_compile_options(protospec_tests PRIVATE -Wall -Wextra)
endif()
add_test(NAME protospec_tests COMMAND protospec_tests)

add_executable(protospec_io_tests test/test_io.cc)
target_link_libraries(protospec_io_tests PRIVATE protospec_io_plain)
if(MSVC)
  target_compile_options(protospec_io_tests PRIVATE /W4 /permissive-)
else()
  target_compile_options(protospec_io_tests PRIVATE -Wall -Wextra)
endif()
add_test(NAME protospec_io_tests COMMAND protospec_io_tests)

# The differential/corpus harness lives in harness/ (owned separately) and needs
# a prebuilt MuJoCo. MUJOCO_ROOT is the same cache variable the harness uses;
# point it at a prebuilt MuJoCo tree (contains include/ and build/, or a staged
# install with lib/ and bin/) to enable it.
set(MUJOCO_ROOT ""
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

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/harness/CMakeLists.txt
   AND EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h"
   AND EXISTS "${MUJOCO_IMPORT_LIB}")
  add_subdirectory(harness)
else()
  message(STATUS "protospec: no prebuilt MuJoCo at MUJOCO_ROOT; harness targets skipped")
endif()

# SDK: the ergonomic authoring layer over the generated object model. Header-
# only (templated over the generated Visit/reflect hooks), MuJoCo-free. The
# query half (profile, detail, model_core, parents, classes) is shipped surface;
# the authoring verbs are fixture-only and live beside the tests they serve.
add_library(protospec_sdk INTERFACE)
target_include_directories(protospec_sdk INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)
target_link_libraries(protospec_sdk INTERFACE protospec)

# SDK save surface (protospec/save.h): the one compiled piece of the SDK. It
# reaches to disk and pulls in the MJCF writer, so it is a small static library
# kept separate from the header-only pure-tree SDK. Fixture-only, like the rest
# of the authoring layer.
add_library(protospec_sdk_io STATIC test/save.cc)
target_link_libraries(protospec_sdk_io PUBLIC protospec_sdk protospec_io_plain)
if(MSVC)
  target_compile_options(protospec_sdk_io PRIVATE /W4 /permissive-)
else()
  target_compile_options(protospec_sdk_io PRIVATE -Wall -Wextra)
endif()

add_executable(protospec_sdk_tests test/test_sdk.cc)
target_link_libraries(protospec_sdk_tests PRIVATE protospec_sdk protospec_sdk_io protospec_io_plain)
if(MSVC)
  # /bigobj: the SDK test TU (heavily templated builders + the added save cases)
  # exceeds the COFF section limit without it.
  target_compile_options(protospec_sdk_tests PRIVATE /W4 /permissive- /bigobj)
else()
  target_compile_options(protospec_sdk_tests PRIVATE -Wall -Wextra)
endif()
add_test(NAME protospec_sdk_tests COMMAND protospec_sdk_tests)

# Public-surface test: includes ONLY the curated <protospec/...> umbrella headers
# (never a generated or io header) and drives load -> edit -> save. Compiling it
# is the guarantee that the surface is self-sufficient.
add_executable(protospec_public_api_tests test/test_public_api.cc)
target_link_libraries(protospec_public_api_tests PRIVATE
  protospec_sdk protospec_sdk_io protospec_io_plain)
if(MSVC)
  target_compile_options(protospec_public_api_tests PRIVATE /W4 /permissive- /bigobj)
else()
  target_compile_options(protospec_public_api_tests PRIVATE -Wall -Wextra)
endif()
add_test(NAME protospec_public_api_tests COMMAND protospec_public_api_tests)
