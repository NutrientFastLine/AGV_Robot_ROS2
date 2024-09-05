cmake_minimum_required(VERSION 3.8)
project(base_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(/usr/include/modbus)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# add_executable(base_control_node src/base_control_node.cpp)
# ament_target_dependencies(base_control_node 
#   rclcpp
#   std_msgs
#   geometry_msgs
#   nav_msgs
#   tf2
#   tf2_ros
# )

add_executable(DS2024_motor_driver_node src/DS2024_motor_driver_node.cpp)
ament_target_dependencies(DS2024_motor_driver_node 
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)
target_link_libraries(DS2024_motor_driver_node modbus)

install(TARGETS  
  DS2024_motor_driver_node
  # base_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
