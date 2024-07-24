cmake_minimum_required(VERSION 3.5)
project(lidar_transform)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)


add_executable(points_raw_transform src/points_raw_transform.cpp)
ament_target_dependencies(points_raw_transform rclcpp sensor_msgs tf2_geometry_msgs pcl_ros pcl_conversions)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS
points_raw_transform
  DESTINATION lib/${PROJECT_NAME})

ament_package()
