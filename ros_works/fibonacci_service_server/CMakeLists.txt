cmake_minimum_required(VERSION 3.8)
project(fibonacci_service_server)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(fibonacci_service_server src/fibonacci_service_server.cpp)
ament_target_dependencies(fibonacci_service_server rclcpp example_interfaces)

install(TARGETS
  fibonacci_service_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()

