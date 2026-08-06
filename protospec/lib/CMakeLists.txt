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
# the project's /W4 does not flag it. Only protospec_io links it; the core stays
# dependency-free.
add_library(tinyxml2 STATIC third_party/tinyxml2/tinyxml2.cpp)
target_include_directories(tinyxml2 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyxml2
)

# Core resolvers: orientation + inertia canonicalization (Q-ORIENT/Q-INERTIA),
# a MuJoCo-free translation unit of lifted math (plan.md Section 3 core/resolve).
# The reader folds authored orientation/inertia spellings into their canonical
# quat/diaginertia forms at parse end through this library, so protospec_io stays
# MuJoCo-free (no mujoco.h, no mujoco.dll runtime dependency for ps_roundtrip).
add_library(protospec_core STATIC core/resolve.cc)
target_include_directories(protospec_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/core)
target_link_libraries(protospec_core PUBLIC protospec)
if(MSVC)
  target_compile_options(protospec_core PRIVATE /W4 /permissive-)
else()
  target_compile_options(protospec_core PRIVATE -Wall -Wextra)
endif()

# MJCF IO: reader (tinyxml2 -> Model) + writer (Model -> MJCF), table-driven by
# the generated xml_binding, with handwritten quirk handlers. Links tinyxml2, the
# core object model, and the MuJoCo-free canonicalization resolvers.
# One profile per translation unit: mjcf_common.cc carries everything keyed on
# ElementType / raw XML / diagnostics (compiled once), and each *_plain.cc pulls
# in the matching .inc to instantiate the templated half for one profile. A
# second profile adds two more TUs and touches nothing here.
add_library(protospec_io STATIC
  io/numeric.cc
  io/include.cc
  io/default_classes.cc
  io/mjcf_common.cc
  io/mjcf_reader_plain.cc
  io/mjcf_writer_plain.cc
)
target_include_directories(protospec_io PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/io
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk
)
target_link_libraries(protospec_io PUBLIC protospec protospec_core PRIVATE tinyxml2)
if(MSVC)
  # /bigobj: the reader TU's per-element template instantiations exceed the COFF
  # section limit without it.
  target_compile_options(protospec_io PRIVATE /W4 /permissive- /bigobj)
else()
  target_compile_options(protospec_io PRIVATE -Wall -Wextra)
endif()

# Validation: three-tier structural/referential/semantic checks over a const
# Model (plan Section 9). MuJoCo-free and tinyxml2-free -- it only needs the
# generated tables in the core library.
add_library(protospec_validate STATIC
  validate/validate.cc
  validate/sizes.cc
)
target_include_directories(protospec_validate PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/validate
)
target_link_libraries(protospec_validate PUBLIC protospec)
if(MSVC)
  target_compile_options(protospec_validate PRIVATE /W4 /permissive-)
else()
  target_compile_options(protospec_validate PRIVATE -Wall -Wextra)
endif()

# Round-trip tool: the harness contract (ps_roundtrip <in.xml>).
add_executable(ps_roundtrip tools/ps_roundtrip.cc)
target_link_libraries(ps_roundtrip PRIVATE protospec_io)

# Validation CLI: the corpus-sanity harness contract (ps_validate <in.xml>).
add_executable(ps_validate tools/ps_validate.cc)
target_link_libraries(ps_validate PRIVATE protospec_io protospec_validate)

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
target_link_libraries(protospec_io_tests PRIVATE protospec_io)
if(MSVC)
  target_compile_options(protospec_io_tests PRIVATE /W4 /permissive-)
else()
  target_compile_options(protospec_io_tests PRIVATE -Wall -Wextra)
endif()
add_test(NAME protospec_io_tests COMMAND protospec_io_tests)

add_executable(protospec_validate_tests test/test_validate.cc)
target_link_libraries(protospec_validate_tests PRIVATE protospec_validate protospec_io)
if(MSVC)
  target_compile_options(protospec_validate_tests PRIVATE /W4 /permissive-)
else()
  target_compile_options(protospec_validate_tests PRIVATE -Wall -Wextra)
endif()
add_test(NAME protospec_validate_tests COMMAND protospec_validate_tests)

# --------------------------------------------------------------------------- #
# MuJoCo-dependent targets: the compile bridge (milestone 5) + its tools/tests. #
# All gated on MuJoCo being found so the MuJoCo-free core above still builds     #
# standalone. MUJOCO_ROOT is the same cache variable the harness uses; point it  #
# at a prebuilt MuJoCo tree (contains include/ and build/) to enable them. On    #
# POSIX the pytest suite compiles the harness directly with the host compiler    #
# instead (tests/test_path_diff.py), so this stays opt-in.                       #
# --------------------------------------------------------------------------- #
set(MUJOCO_ROOT ""
    CACHE PATH "Root of a prebuilt MuJoCo (a source build tree, or a staged install)")

# The same two tree shapes the harness accepts: MuJoCo's own build tree, and the
# install tree third_party/MuJoCo/build.{ps1,sh} stages under
# third_party/install/MuJoCo. Resolved identically here so the harness
# subdirectory below is reached from either.
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
  set(MUJOCO_IMPORT_LIB "${MUJOCO_LIB_DIR}/libmujoco.so")
  set(MUJOCO_DLL        "${MUJOCO_IMPORT_LIB}")
endif()

# NC3 asset-pipeline third-party deps, prebuilt alongside the vendored MuJoCo
# (the same static libs MuJoCo itself links). qhull (convex-hull graph) and
# tinyobjloader (OBJ parsing) are linked into the lifted TU; lodepng (PNG decode)
# is reserved for the file-texture/hfield wave. Their symbols are internal to
# mujoco.dll (not exported), so our own copies do not clash at link.
set(MUJOCO_DEPS "${MUJOCO_ROOT}/build/_deps")
set(QHULL_INCLUDE_DIR "${MUJOCO_DEPS}/qhull-src/src")
set(QHULL_LIB         "${MUJOCO_ROOT}/build/lib/Release/qhullstatic_r.lib")
set(TINYOBJ_INCLUDE_DIR "${MUJOCO_DEPS}/tinyobjloader-src")
set(TINYOBJ_LIB         "${MUJOCO_ROOT}/build/lib/Release/tinyobjloader.lib")
set(LODEPNG_INCLUDE_DIR "${MUJOCO_DEPS}/lodepng-src")
set(LODEPNG_LIB         "${MUJOCO_ROOT}/build/lib/Release/lodepng.lib")

# The differential/corpus harness lives in cpp/harness/ (owned separately); wire
# it in before the MuJoCo bridge tools below so its plugin_registry / model_diff
# targets exist when ps_compile and ps_native_diff link them.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/harness/CMakeLists.txt
   AND EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h"
   AND EXISTS "${MUJOCO_IMPORT_LIB}")
  add_subdirectory(harness)
else()
  message(STATUS "protospec: no prebuilt MuJoCo at MUJOCO_ROOT; harness + compile bridge targets skipped")
endif()

if(EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h" AND EXISTS "${MUJOCO_IMPORT_LIB}")
  # The one quarantine zone for mujoco.h besides the harness: Model -> mjModel +
  # Binding + Recompile (plan DR-5/DR-10/DR-11). Links the object model, the MJCF
  # writer (XML path), and MuJoCo. /W4 like every target.
  add_library(protospec_bridge STATIC
    compile/compile.cc
    compile/binding.cc
  )
  target_include_directories(protospec_bridge
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/compile
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/sdk
            ${MUJOCO_INCLUDE_DIR}
  )
  target_link_libraries(protospec_bridge PUBLIC protospec protospec_io
                                                protospec_validate
                                         PRIVATE "${MUJOCO_IMPORT_LIB}")
  if(MSVC)
    target_compile_options(protospec_bridge PRIVATE /W4 /permissive-)
  else()
    target_compile_options(protospec_bridge PRIVATE -Wall -Wextra)
  endif()

  # Copies mujoco.dll next to any exe that links the bridge.
  function(ps_copy_mujoco_dll target)
    add_custom_command(TARGET ${target} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${MUJOCO_DLL}" "$<TARGET_FILE_DIR:${target}>"
      VERBATIM)
  endfunction()

  # Corpus compile CLI (the harness contract: ps_compile <in.xml>). Links the
  # shared plugin_registry so the bridge's mj_loadXML resolves plugin models.
  add_executable(ps_compile tools/ps_compile.cc)
  target_include_directories(ps_compile PRIVATE
    ${MUJOCO_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/harness)
  target_link_libraries(ps_compile PRIVATE protospec_bridge plugin_registry "${MUJOCO_IMPORT_LIB}")
  ps_copy_mujoco_dll(ps_compile)

  # Compile-path differential harness (the permanent correctness net): compiles a
  # corpus twice and diffs the resulting mjModels field by field. The pytest
  # driver (tests/test_path_diff.py) compiles it directly with g++ on POSIX; this
  # target is the same tool for anyone building through CMake.
  add_executable(ps_path_diff harness/ps_path_diff.cc)
  target_include_directories(ps_path_diff PRIVATE
    ${MUJOCO_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/harness)
  target_link_libraries(ps_path_diff
    PRIVATE protospec_bridge model_diff_lib plugin_registry "${MUJOCO_IMPORT_LIB}")
  if(MSVC)
    target_compile_options(ps_path_diff PRIVATE /W4 /permissive-)
  else()
    target_compile_options(ps_path_diff PRIVATE -Wall -Wextra)
  endif()
  ps_copy_mujoco_dll(ps_path_diff)

  add_executable(protospec_bridge_tests test/test_bridge.cc)
  target_include_directories(protospec_bridge_tests PRIVATE ${MUJOCO_INCLUDE_DIR})
  target_link_libraries(protospec_bridge_tests PRIVATE protospec_bridge "${MUJOCO_IMPORT_LIB}")
  if(MSVC)
    target_compile_options(protospec_bridge_tests PRIVATE /W4 /permissive-)
  else()
    target_compile_options(protospec_bridge_tests PRIVATE -Wall -Wextra)
  endif()
  ps_copy_mujoco_dll(protospec_bridge_tests)
  add_test(NAME protospec_bridge_tests COMMAND protospec_bridge_tests)
endif()

# SDK: the ergonomic authoring layer over the generated object model. Header-
# only (templated over the generated Visit/reflect hooks), MuJoCo-free. Consumers
# link the core (types/reflect/defaults) plus this interface for the includes.
add_library(protospec_sdk INTERFACE)
target_include_directories(protospec_sdk INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk
)
target_link_libraries(protospec_sdk INTERFACE protospec)

# SDK save surface (protospec/save.h): the one compiled piece of the SDK. It
# reaches to disk and pulls in the MJCF writer, so it is a small static library
# kept separate from the header-only pure-tree SDK. Also carries the single
# shared asset-externalization implementation the studio editor delegates to.
add_library(protospec_sdk_io STATIC sdk/save.cc)
target_link_libraries(protospec_sdk_io PUBLIC protospec_sdk protospec_io)
if(MSVC)
  target_compile_options(protospec_sdk_io PRIVATE /W4 /permissive-)
else()
  target_compile_options(protospec_sdk_io PRIVATE -Wall -Wextra)
endif()

add_executable(protospec_sdk_tests test/test_sdk.cc)
target_link_libraries(protospec_sdk_tests PRIVATE protospec_sdk protospec_sdk_io protospec_io)
if(MSVC)
  # /bigobj: the SDK test TU (heavily templated builders + the added save cases)
  # exceeds the COFF section limit without it.
  target_compile_options(protospec_sdk_tests PRIVATE /W4 /permissive- /bigobj)
else()
  target_compile_options(protospec_sdk_tests PRIVATE -Wall -Wextra)
endif()
add_test(NAME protospec_sdk_tests COMMAND protospec_sdk_tests)

# The Phase 2 exit gate: the SDK, reader and writer over a SECOND emission
# profile that shares nothing with the plain one but the schema. mjcf_io_mock.cc
# is that profile's instantiation TU, the mirror of io/mjcf_*_plain.cc.
add_executable(protospec_profile_tests
  test/test_profile_mock.cc
  test/mjcf_io_mock.cc
)
target_include_directories(protospec_profile_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/io
  ${CMAKE_CURRENT_SOURCE_DIR}/test
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyxml2
)
target_link_libraries(protospec_profile_tests PRIVATE protospec_sdk protospec_io tinyxml2)
if(MSVC)
  target_compile_options(protospec_profile_tests PRIVATE /W4 /permissive- /bigobj)
else()
  target_compile_options(protospec_profile_tests PRIVATE -Wall -Wextra)
endif()
add_test(NAME protospec_profile_tests COMMAND protospec_profile_tests)

# Public-API surface test: includes ONLY the curated <protospec/...> umbrella
# headers (never a generated / io / bridge / validate / sdk-detail header) and
# drives the full load -> edit -> validate -> compile -> step -> save path.
# Compiling it is the guarantee that the public surface is self-sufficient -- a
# consumer needs nothing internal. Gated on the bridge (it steps a real mjModel).
if(TARGET protospec_bridge)
  add_executable(protospec_public_api_tests test/test_public_api.cc)
  target_include_directories(protospec_public_api_tests PRIVATE ${MUJOCO_INCLUDE_DIR})
  target_link_libraries(protospec_public_api_tests PRIVATE
    protospec_bridge protospec_validate protospec_sdk protospec_sdk_io
    "${MUJOCO_IMPORT_LIB}")
  if(MSVC)
    target_compile_options(protospec_public_api_tests PRIVATE /W4 /permissive- /bigobj)
  else()
    target_compile_options(protospec_public_api_tests PRIVATE -Wall -Wextra)
  endif()
  ps_copy_mujoco_dll(protospec_public_api_tests)
  add_test(NAME protospec_public_api_tests COMMAND protospec_public_api_tests)
endif()
