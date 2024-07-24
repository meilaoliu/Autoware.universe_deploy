cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0009 NEW)

project(all_sensors_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

