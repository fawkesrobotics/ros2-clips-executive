cmake_minimum_required(VERSION 3.8)
project(cx_example_skill_nodes)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(cx_skill_execution REQUIRED)
find_package(cx_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

set(DEPENDENCIES cx_msgs cx_skill_execution rclcpp rclcpp_lifecycle
                 lifecycle_msgs)

set(SOURCES src/cx_example_skill_nodes/dummy_skill.cpp)

include_directories(include)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

add_executable(dummy_skill_node src/dummy_skill_launch_node.cpp)
target_link_libraries(dummy_skill_node ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ${PROJECT_NAME} dummy_skill_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})

ament_package()
