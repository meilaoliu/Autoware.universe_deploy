cmake_minimum_required(VERSION 3.5)
project(my_vehicle_interface)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(hunter_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)

install(PROGRAMS
  scripts/vehicle_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
