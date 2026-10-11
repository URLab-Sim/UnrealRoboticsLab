cmake_minimum_required(VERSION 3.16)
project(urlab_mjshim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Where MuJoCo was installed by third_party/MuJoCo/build.ps1|sh. Defaulted
# rather than searched: this library is only ever built beside that install,
# and finding a different MuJoCo would be the one mistake worth failing on.
set(MUJOCO_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../install/MuJoCo"
    CACHE PATH "MuJoCo install prefix")

if(NOT EXISTS "${MUJOCO_INSTALL_DIR}/include/mujoco/mujoco.h")
  message(FATAL_ERROR
    "MuJoCo not found at ${MUJOCO_INSTALL_DIR}. Build it first: "
    "third_party/MuJoCo/build.ps1 (Windows) or build.sh (Linux/macOS).")
endif()

add_library(urlab_mjshim SHARED src/urlab_mjshim.cc)

target_include_directories(urlab_mjshim
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE "${MUJOCO_INSTALL_DIR}/include"
)

target_compile_definitions(urlab_mjshim PRIVATE URLAB_MJSHIM_BUILD)

# The allocator this library uses has to be the one MuJoCo uses, which is the
# whole point of it existing. On Windows that means the same dynamic CRT
# MuJoCo/build.ps1 selects; anything else reintroduces the boundary this is
# here to remove.
if(MSVC)
  set_property(TARGET urlab_mjshim PROPERTY
               MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
  target_link_libraries(urlab_mjshim PRIVATE "${MUJOCO_INSTALL_DIR}/lib/mujoco.lib")
else()
  set_target_properties(urlab_mjshim PROPERTIES
                        CXX_VISIBILITY_PRESET hidden
                        VISIBILITY_INLINES_HIDDEN ON)
  find_library(MUJOCO_LIBRARY NAMES mujoco PATHS "${MUJOCO_INSTALL_DIR}/lib" REQUIRED)
  target_link_libraries(urlab_mjshim PRIVATE "${MUJOCO_LIBRARY}")
endif()

install(TARGETS urlab_mjshim
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
