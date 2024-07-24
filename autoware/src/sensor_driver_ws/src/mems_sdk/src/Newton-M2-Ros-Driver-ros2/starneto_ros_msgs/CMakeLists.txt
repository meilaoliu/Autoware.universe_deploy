cmake_minimum_required(VERSION 3.5)
# project
project(starneto_ros_msgs)

# if(POLICY CMP0057)
#   cmake_policy(SET CMP0057 NEW)
# endif()

find_package(ament_cmake REQUIRED)
# find_package(genmsg REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclpy REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) # Needed to generate custom message types

set(msg_files
  "msg/Gpfpd.msg"
  "msg/Gtimu.msg"
  "msg/Pos320Nav.msg"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
  # geometry_msgs
  # sensor_msgs
  LIBRARY_NAME ${PROJECT_NAME}
  )

ament_register_extension(
  "rosidl_generate_interfaces"
  "rosidl_generator_cpp"
  "rosidl_generator_cpp_generate_interfaces.cmake"
  )

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  # geometry_msgs
  )

# add_message_files(  # Tell catkin which custom message files to include
#   FILES
#   Gpfpd.msg
#   Gtimu.msg
#   Pos320Nav.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   sensor_msgs
# )


# catkin_package(
#   CATKIN_DEPENDS message_runtime std_msgs geometry_msgs # Needed to generate custom messages
# )


ament_export_include_directories(
  include
  ${ament_INCLUDE_DIRS}
  # ${catkin_INCLUDE_DIRS}
  # ${rclcpp_INCLUDE_DIRS}
  # ${rclpy_INCLUDE_DIRS}
)

ament_package()