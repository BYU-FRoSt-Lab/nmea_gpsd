cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(topic_monitor CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Define the executable
add_executable(topic_monitor_node src/topic_monitor_node.cpp)

# Link necessary libraries
ament_target_dependencies(topic_monitor_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  ament_index_cpp
  rosidl_runtime_cpp
  rosidl_typesupport_introspection_cpp
)

target_link_libraries(topic_monitor_node yaml-cpp)

# Install the executable
install(TARGETS
  topic_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the config folder to the package's share directory
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  ament_index_cpp
  rosidl_runtime_cpp
  rosidl_typesupport_introspection_cpp
)

ament_package()
