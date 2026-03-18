cmake_minimum_required(VERSION 3.20)
cmake_policy(SET CMP0048 NEW)
project(rslidar_sdk)

#=======================================
# Custom Point Type (XYZI,XYZIRT, XYZIF, XYZIRTF)
#=======================================
set(POINT_TYPE XYZI)

option(ENABLE_TRANSFORM "Enable transform functions" OFF)
if(${ENABLE_TRANSFORM})
  add_definitions("-DENABLE_TRANSFORM")
  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif(${ENABLE_TRANSFORM})

option(ENABLE_EPOLL_RECEIVE "Receive packets with epoll() instead of select()" OFF)
if(${ENABLE_EPOLL_RECEIVE})
  add_definitions("-DENABLE_EPOLL_RECEIVE")
endif(${ENABLE_EPOLL_RECEIVE})

option(ENABLE_MODIFY_RECVBUF "Enable modify size of RECVBUF" ON)
if(${ENABLE_MODIFY_RECVBUF})
  add_definitions("-DENABLE_MODIFY_RECVBUF")
endif(${ENABLE_MODIFY_RECVBUF})

option(ENABLE_WAIT_IF_QUEUE_EMPTY "Enable waiting for a while in handle thread if the queue is empty" OFF)
if(${ENABLE_WAIT_IF_QUEUE_EMPTY})
  add_definitions("-DENABLE_WAIT_IF_QUEUE_EMPTY")
endif(${ENABLE_WAIT_IF_QUEUE_EMPTY})

option(ENABLE_DIFOP_PARSE "Enable parsing DIFOP Packet" ON)
if(${ENABLE_DIFOP_PARSE})
  add_definitions("-DENABLE_DIFOP_PARSE")
endif(${ENABLE_DIFOP_PARSE})

option(ENABLE_STAMP_WITH_LOCAL "Enable stamp point cloud with local time" OFF)
if(${ENABLE_STAMP_WITH_LOCAL})
  add_definitions("-DENABLE_STAMP_WITH_LOCAL")
endif(${ENABLE_STAMP_WITH_LOCAL})


option(ENABLE_SOURCE_PACKET_LEGACY "Enable ROS Source of MSOP/DIFOP Packet v1.3.x" OFF)
if(${ENABLE_SOURCE_PACKET_LEGACY})
  add_definitions("-DENABLE_SOURCE_PACKET_LEGACY")
endif(${ENABLE_SOURCE_PACKET_LEGACY})

option(ENABLE_IMU_DATA_PARSE           "Enable imu data parse" OFF)
if(${ENABLE_IMU_DATA_PARSE})

  message(=============================================================)
  message("-- Enable imu data parse")
  message(=============================================================)

  add_definitions("-DENABLE_IMU_DATA_PARSE")

endif(${ENABLE_IMU_DATA_PARSE})

#========================
# Project details / setup
#========================
set(PROJECT_NAME rslidar_sdk)

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
  add_definitions(-O3)
endif()

add_definitions(-std=c++17)
add_compile_options(-Wall)

#========================
# Point Type Definition
#========================
if(${POINT_TYPE} STREQUAL "XYZI")
  add_definitions(-DPOINT_TYPE_XYZI)
elseif(${POINT_TYPE} STREQUAL "XYZIRT")
  add_definitions(-DPOINT_TYPE_XYZIRT)
elseif(${POINT_TYPE} STREQUAL "XYZIF")
  add_definitions(-DPOINT_TYPE_XYZIF)
elseif(${POINT_TYPE} STREQUAL "XYZIRTF")
  add_definitions(-DPOINT_TYPE_XYZIRTF)
endif()

message(=============================================================)
message("-- POINT_TYPE is ${POINT_TYPE}")
message(=============================================================)

#========================
# Dependencies Setup
#========================

# Others
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml-cpp/yaml.h
  PATHS
    /opt/homebrew/include
    /opt/homebrew/opt/yaml-cpp/include
    /usr/local/include
    /usr/include
)

find_library(YAML_CPP_LIBRARY
  NAMES yaml-cpp
  PATHS
    /opt/homebrew/lib
    /opt/homebrew/opt/yaml-cpp/lib
    /usr/local/lib
    /usr/lib
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
)

if(NOT YAML_CPP_INCLUDE_DIR OR NOT YAML_CPP_LIBRARY)
  message(FATAL_ERROR "yaml-cpp not found. Please install yaml-cpp first.")
endif()

include_directories(${YAML_CPP_INCLUDE_DIR})

# Foxglove C++ SDK
if(APPLE AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64)$")
  set(FOXGLOVE_SDK_URL "https://github.com/foxglove/foxglove-sdk/releases/download/sdk/v0.20.0/foxglove-v0.20.0-cpp-aarch64-apple-darwin.zip")
  set(FOXGLOVE_SDK_HASH "5f0df707a7fb263207ea4bf04da4fb81de1dfe36a9f3c6a974447f86d2ab962b")
elseif(APPLE AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$")
  set(FOXGLOVE_SDK_URL "https://github.com/foxglove/foxglove-sdk/releases/download/sdk/v0.20.0/foxglove-v0.20.0-cpp-x86_64-apple-darwin.zip")
  set(FOXGLOVE_SDK_HASH "74adcd391319226c651a7e89e6bd034797f327725d45b0c16e2c9b6372cfc245")
elseif(LINUX AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(FOXGLOVE_SDK_URL "https://github.com/foxglove/foxglove-sdk/releases/download/sdk/v0.20.0/foxglove-v0.20.0-cpp-aarch64-unknown-linux-gnu.zip")
  set(FOXGLOVE_SDK_HASH "856e4f0ee590ef8e8d143b2cb4c339d9c495e73efac3728c2071d9040dc62e61")
elseif(LINUX AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$")
  set(FOXGLOVE_SDK_URL "https://github.com/foxglove/foxglove-sdk/releases/download/sdk/v0.20.0/foxglove-v0.20.0-cpp-x86_64-unknown-linux-gnu.zip")
  set(FOXGLOVE_SDK_HASH "112389a88f2f4656ccd37934bc538e478109fc006a38785c510713df02ede9bc")
else()
  message(FATAL_ERROR "Unsupported platform for bundled Foxglove SDK: ${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}. Supported: macOS(x86_64/arm64), Linux(x86_64/aarch64).")
endif()

include(FetchContent)
FetchContent_Declare(
  foxglove_sdk
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  URL ${FOXGLOVE_SDK_URL}
  URL_HASH SHA256=${FOXGLOVE_SDK_HASH}
)
FetchContent_MakeAvailable(foxglove_sdk)
if(EXISTS "${foxglove_sdk_SOURCE_DIR}/foxglove/include")
  set(FOXGLOVE_ROOT "${foxglove_sdk_SOURCE_DIR}/foxglove")
else()
  set(FOXGLOVE_ROOT "${foxglove_sdk_SOURCE_DIR}")
endif()
file(GLOB FOXGLOVE_CPP_SOURCES CONFIGURE_DEPENDS
  "${FOXGLOVE_ROOT}/src/*.cpp"
  "${FOXGLOVE_ROOT}/src/server/*.cpp")

#Include directory#
include_directories(${PROJECT_SOURCE_DIR}/src)

#Driver core#
add_subdirectory(src/rs_driver)
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})

#========================
# Build Setup
#========================

add_executable(rslidar_sdk_node
               node/rslidar_sdk_node.cpp)

target_sources(rslidar_sdk_node PRIVATE ${FOXGLOVE_CPP_SOURCES})
target_include_directories(rslidar_sdk_node PRIVATE
  "${FOXGLOVE_ROOT}/include"
  "${FOXGLOVE_ROOT}/include/foxglove")

find_package(Threads REQUIRED)

target_link_libraries(rslidar_sdk_node
                      ${rs_driver_LIBRARIES}
                      ${YAML_CPP_LIBRARY}
                      "${FOXGLOVE_ROOT}/lib/libfoxglove.a"
                      Threads::Threads)

if(APPLE)
  target_link_libraries(rslidar_sdk_node
                        "-framework CoreFoundation"
                        "-framework Security")
elseif(UNIX)
  target_link_libraries(rslidar_sdk_node
                        ${CMAKE_DL_LIBS}
                        m)
endif()

install(TARGETS
  rslidar_sdk_node
  DESTINATION bin)

install(DIRECTORY
  config
  scripts
  DESTINATION share/${PROJECT_NAME})

