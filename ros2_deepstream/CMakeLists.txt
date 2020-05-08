cmake_minimum_required(VERSION 3.5)
project(ros2_deepstream)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_deepstream_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
find_package(CUDA 10.0 REQUIRED)

pkg_check_modules(GLIB REQUIRED glib-2.0)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER-APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTREAMER-AUDIO REQUIRED gstreamer-audio-1.0)
pkg_check_modules(GSTREAMER-PBUTILS REQUIRED gstreamer-pbutils-1.0)
pkg_check_modules(GSTREAMER-FFT REQUIRED gstreamer-fft-1.0)

include_directories(
  ${GLIB_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER-APP_INCLUDE_DIRS}
  ${GSTREAMER-AUDIO_INCLUDE_DIRS}
  ${GSTREAMER-PBUTILS_INCLUDE_DIRS}
  ${GSTREAMER-FFT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  include
  /opt/nvidia/deepstream/deepstream-4.0/sources/includes
  ${CUDA_INCLUDE_DIRS}
)

link_directories(
  /opt/nvidia/deepstream/deepstream-4.0/lib/
)

set(all_gstreamer_libraries
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER-APP_LIBRARIES}
  ${GSTREAMER-AUDIO_LIBRARIES}
  ${GSTREAMER-PBUTILS_LIBRARIES}
  ${GSTREAMER-FFT_LIBRARIES}
  ${CUDA_LIBRARIES}
  nvdsgst_helper
  nvdsgst_meta
  nvds_meta
  nvbufsurface
  nvbufsurftransform
)

set(plugin_dependencies rclcpp sensor_msgs ros2_deepstream_msgs)

add_library(ds_rclcpp_publisher src/gst_ds_rclcpp_publisher.cpp)
ament_target_dependencies(ds_rclcpp_publisher rclcpp ros2_deepstream_msgs)
target_link_libraries(ds_rclcpp_publisher ${all_gstreamer_libraries})

ament_package()
