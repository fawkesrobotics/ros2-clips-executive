cmake_minimum_required(VERSION 3.8)
project(cx_bringup)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_lifecycle_manager REQUIRED)
find_package(cx_feature_manager REQUIRED)
find_package(cx_clips REQUIRED)
find_package(cx_clips_executive REQUIRED)
find_package(PkgConfig REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  plansys2_domain_expert
  plansys2_problem_expert
  plansys2_planner
  plansys2_lifecycle_manager
  cx_feature_manager
  cx_clips
  cx_clips_executive
  PkgConfig
)
set(dependencies_psys2
  ament_cmake
  rclcpp
  plansys2_domain_expert
  plansys2_problem_expert
  plansys2_planner
  plansys2_lifecycle_manager
  PkgConfig
)

add_executable(plansys_node
  src/plansys2_node.cpp
)

add_executable(cx_node
  src/cx_node.cpp
)
ament_target_dependencies(plansys_node ${dependencies_psys2})
target_compile_definitions(plansys_node PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(cx_node ${dependencies})
target_compile_definitions(cx_node PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(cx_node ClipsNS::libclips_ns)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch params clips proto DESTINATION share/${PROJECT_NAME})

install(TARGETS
  plansys_node
  cx_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()
