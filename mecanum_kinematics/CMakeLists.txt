cmake_minimum_required(VERSION 3.8)
project(mecanum_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(odrive_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(inverse src/inverse_kinematics.cpp)
add_executable(forward src/forward_kinematics.cpp)

ament_target_dependencies(inverse
  rclcpp_lifecycle
  odrive_interface
  geometry_msgs)
ament_target_dependencies(forward
  rclcpp_lifecycle
  odrive_interface
  nav_msgs)

install(TARGETS
  inverse
  forward
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
