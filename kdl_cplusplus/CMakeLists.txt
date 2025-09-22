cmake_minimum_required(VERSION 3.8)
project(kdl_cplusplus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(arm_kinematics REQUIRED)

add_executable(ik_service_cpp src/ik_service_cpp.cpp)
target_compile_features(ik_service_cpp PUBLIC cxx_std_17)
ament_target_dependencies(ik_service_cpp
  rclcpp rclcpp_components
  geometry_msgs trajectory_msgs sensor_msgs builtin_interfaces
  kdl_parser orocos_kdl ament_index_cpp arm_kinematics)

install(TARGETS ik_service_cpp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/ik_test_client.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
