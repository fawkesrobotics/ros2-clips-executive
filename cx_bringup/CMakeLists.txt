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
find_package(cx_features REQUIRED)
find_package(cx_clips REQUIRED)
find_package(cx_clips_executive REQUIRED)
find_package(PkgConfig REQUIRED)


set(dependencies
  ament_cmake
  rclcpp
  cx_features
  cx_clips
  cx_clips_executive
  PkgConfig
)

#import clipsmm via pkg-config
pkg_check_modules(CLIPSMM REQUIRED IMPORTED_TARGET clipsmm-1.0)


add_executable(cx_node
  src/cx_node.cpp
)

ament_target_dependencies(cx_node ${dependencies})
target_compile_definitions(cx_node PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(cx_node PkgConfig::CLIPSMM)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})

install(TARGETS
  cx_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()
