cmake_minimum_required(VERSION 3.8)
project(prbt_hardware_support)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pilz_msgs REQUIRED)

add_executable(fake_speed_override_node src/fake_speed_override_node.cpp)
ament_target_dependencies(fake_speed_override_node rclcpp pilz_msgs)

install(TARGETS fake_speed_override_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
