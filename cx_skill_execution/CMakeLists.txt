cmake_minimum_required(VERSION 3.8)
project(cx_skill_execution)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(cx_msgs REQUIRED)
find_package(cx_utils REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  cx_msgs
  cx_utils
)

set(CLIPS_EXECUTIVE_SOURCES
  src/cx_skill_execution/SkillExecution.cpp
  src/cx_skill_execution/SkillExecutionMaster.cpp
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED ${CLIPS_EXECUTIVE_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(
  DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)
    add_subdirectory(test)
endif()

install(TARGETS
  ${PROJECT_NAME}
  # exec_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
