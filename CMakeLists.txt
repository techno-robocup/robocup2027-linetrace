cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(robocup2027_linetrace LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ----------------------------------------------------------------------------
# Per-machine target gating. Build only what the current machine can support:
#   training PC : -DBUILD_TRAINER=ON  (default)
#   laptop      : -DBUILD_CONTROL_STATION=ON  (needs SDL2)
#   Raspberry Pi: -DBUILD_COLLECTOR=ON -DBUILD_EXECUTOR=ON  (needs libcamera)
# ----------------------------------------------------------------------------
option(BUILD_TRAINER         "LibTorch trainer (Torch + OpenCV)"                ON)
option(BUILD_EXECUTOR        "Pi model executor (Torch + OpenCV + libcamera)"   OFF)
option(BUILD_COLLECTOR       "Pi data collector (OpenCV + libcamera)"           OFF)
option(BUILD_CONTROL_STATION "Laptop control station (SDL2 + OpenCV)"           OFF)
option(BUILD_TRANSFER_CLIENT "Transfer client library + CLI"                    ON)
option(BUILD_TESTS           "Unit / smoke tests"                               ON)

# Path to the libtorch tree (vendored alongside in yolo_cpp by default).
set(LIBTORCH_DIR "${CMAKE_SOURCE_DIR}/../yolo_cpp/libtorch"
    CACHE PATH "Path to the libtorch distribution")

# Anything that uses the model / preprocessing needs Torch + OpenCV.
set(_need_torch OFF)
if(BUILD_TRAINER OR BUILD_EXECUTOR OR BUILD_TESTS)
  set(_need_torch ON)
endif()

if(_need_torch)
  list(APPEND CMAKE_PREFIX_PATH "${LIBTORCH_DIR}")
  set(Torch_DIR "${LIBTORCH_DIR}/share/cmake/Torch")
  find_package(Torch REQUIRED)
  find_package(OpenCV REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
  message(STATUS "Torch:  ${LIBTORCH_DIR}")
  message(STATUS "OpenCV: ${OpenCV_VERSION}")
endif()

# Collector / control station need OpenCV but not Torch.
if((BUILD_COLLECTOR OR BUILD_CONTROL_STATION) AND NOT _need_torch)
  find_package(OpenCV REQUIRED)
endif()

# ----- libraries -----
add_subdirectory(libs/common)
if(_need_torch OR BUILD_COLLECTOR OR BUILD_EXECUTOR)
  add_subdirectory(libs/common_torch)
endif()
if(BUILD_TRANSFER_CLIENT OR BUILD_COLLECTOR)
  add_subdirectory(libs/transfer_client)
endif()
# esp32_driver is POSIX-only and lightweight; always build it (and uart_probe)
# so it can be compile-checked off-robot.
add_subdirectory(libs/esp32_driver)
if(BUILD_COLLECTOR OR BUILD_CONTROL_STATION OR BUILD_EXECUTOR)
  add_subdirectory(libs/netutil)
endif()
if(BUILD_COLLECTOR OR BUILD_EXECUTOR)
  add_subdirectory(libs/camera)
endif()

# ----- applications -----
if(BUILD_TRAINER)
  add_subdirectory(apps/trainer)
endif()
if(BUILD_TRANSFER_CLIENT OR BUILD_COLLECTOR)
  add_subdirectory(apps/push_session)
endif()
if(BUILD_CONTROL_STATION)
  add_subdirectory(apps/control_station)
endif()
if(BUILD_COLLECTOR)
  add_subdirectory(apps/collector)
endif()
if(BUILD_EXECUTOR)
  add_subdirectory(apps/executor)
endif()

if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
