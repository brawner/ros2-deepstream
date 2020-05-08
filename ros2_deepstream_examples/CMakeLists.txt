cmake_minimum_required(VERSION 3.5)
project(ros2_deepstream_examples)

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
find_package(ros2_deepstream REQUIRED)

add_executable(object_identification_pointcloud_segmenter src/object_identification_pointcloud_segmenter.cpp)
target_include_directories(object_identification_pointcloud_segmenter PUBLIC include)
ament_target_dependencies(object_identification_pointcloud_segmenter rclcpp sensor_msgs ros2_deepstream_msgs ros2_deepstream)

ament_package()
