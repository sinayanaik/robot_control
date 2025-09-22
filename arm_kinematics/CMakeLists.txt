cmake_minimum_required(VERSION 3.8)
project(arm_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanJointTrajectory.srv"
  "srv/ComputeIK.srv"
  DEPENDENCIES trajectory_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

install(
  PROGRAMS
    src/trajectory_service.py
    src/trajectory_executor.py
    src/demo_plan_publish.py
    src/gui_ee.py
    src/motion_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


