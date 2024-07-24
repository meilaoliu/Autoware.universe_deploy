cmake_minimum_required(VERSION 3.5)
project(starneto_mems)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fPIC -Wall -g -Wno-sign-compare -pthread")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath,'$ORIGIN/'" )
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath,'$ORIGIN/'" )

set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}")
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}")

# add_compile_options(-std=c++11)

# set(PROJECT_DEPS
#   roscpp
#   std_msgs
#   starneto_ros_msgs
#   serial
# )

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(starneto_ros_msgs REQUIRED)
find_package(serial REQUIRED)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
#   geometry_msgs
#   starneto_ros_msgs
#   serial
# )

include_directories(
  include
)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/starneto_mems.cpp
  src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  starneto_ros_msgs
  serial
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # $<INSTALL_INTERFACE:include>
)


# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   )

ament_export_dependencies(
  rclcpp
  std_msgs
  starneto_ros_msgs
  serial
)

ament_export_include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.xml"
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()