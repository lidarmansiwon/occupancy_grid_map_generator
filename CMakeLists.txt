cmake_minimum_required(VERSION 3.5)
project(occupancy_grid_map_generator)

set(occupancy_grid_map_generator_MAJOR_VERSION 0)
set(occupancy_grid_map_generator_MINOR_VERSION 0)
set(occupancy_grid_map_generator_PATCH_VERSION 0)
set(occupancy_grid_map_generator_VERSION
  ${occupancy_grid_map_generator_MAJOR_VERSION}.${occupancy_grid_map_generator_MINOR_VERSION}.${occupancy_grid_map_generator_PATCH_VERSION})

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED) # 추가
find_package(pcl REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

add_executable(occupancy_grid_map_generator src/occupancy_grid_map_generator.cpp)
ament_target_dependencies(occupancy_grid_map_generator rclcpp sensor_msgs nav_msgs pcl_conversions pcl_ros pcl)


install(TARGETS
  occupancy_grid_map_generator 
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})


ament_package()