cmake_minimum_required(VERSION 3.8)
project(cx_clips)


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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(cx_msgs REQUIRED)
find_package(cx_utils REQUIRED)
find_package(cx_core REQUIRED)
# find_package(cx_features REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  PkgConfig
  lifecycle_msgs
  cx_msgs
  cx_utils
  cx_core
  # cx_features
)

include_directories(include)

set(CLIPS_SOURCES
  src/cx_clips/CLIPSEnvManagerClient.cpp
  src/cx_clips/CLIPSEnvManagerNode.cpp
)

#import clipsmm via pkg-config
pkg_check_modules(CLIPSMM REQUIRED IMPORTED_TARGET clipsmm-1.0)

add_library(${PROJECT_NAME} SHARED ${CLIPS_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} PkgConfig::CLIPSMM)


add_executable(clips_node src/clips_node.cpp)

ament_target_dependencies(clips_node ${dependencies})
target_link_libraries(clips_node ${PROJECT_NAME})
target_link_libraries(clips_node PkgConfig::CLIPSMM)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  clips_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
