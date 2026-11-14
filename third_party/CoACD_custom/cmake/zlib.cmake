# A sysroot build (build_all.sh --engine) must not pick up the host's zlib:
# find_package() would find /usr/lib/.../libz.so while the compile runs against
# UE's sysroot, which ships no zlib.h. Fetch zlib in that case.
if(NOT "$ENV{CFLAGS} ${CMAKE_C_FLAGS} ${CMAKE_SYSROOT}" MATCHES "sysroot")
  find_package(ZLIB QUIET)
endif()

if(ZLIB_FOUND)
  message(STATUS "ZLIB found, skipping fetch.")
  return()
endif()
include(FetchContent)

if(CMAKE_VERSION VERSION_LESS 3.28)
  FetchContent_Declare(
    zlib
    GIT_REPOSITORY https://github.com/madler/zlib.git
    GIT_TAG v1.2.11
    OVERRIDE_FIND_PACKAGE)

  FetchContent_GetProperties(zlib)
  if(NOT zlib_POPULATED)
    FetchContent_Populate(zlib)
    add_subdirectory(${zlib_SOURCE_DIR} ${zlib_BINARY_DIR} EXCLUDE_FROM_ALL)
  endif()
else()

  FetchContent_Declare(
    zlib
    GIT_REPOSITORY https://github.com/madler/zlib.git
    GIT_TAG v1.2.11
    EXCLUDE_FROM_ALL OVERRIDE_FIND_PACKAGE)
  FetchContent_MakeAvailable(zlib)

endif()

set_target_properties(zlibstatic PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
add_library(ZLIB::ZLIB ALIAS zlibstatic)
set(ZLIB_LIBRARY ZLIB::ZLIB)
set(ZLIB_INCLUDE_DIR ${zlib_SOURCE_DIR})
set(ZLIB_FOUND TRUE)
target_include_directories(zlibstatic PUBLIC ${zlib_SOURCE_DIR}
                                             ${zlib_BINARY_DIR})
