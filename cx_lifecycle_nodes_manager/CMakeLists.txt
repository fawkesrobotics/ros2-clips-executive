cmake_minimum_required(VERSION 3.8)
project(cx_lifecycle_nodes_manager)

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
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  lifecycle_msgs
  rclcpp_lifecycle
)

include_directories(include)

set(LIFECYCLE_MANAGER_SOURCES
  src/LifecycleNodesClient.cpp
  src/LifecycleNodesManager.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${LIFECYCLE_MANAGER_SOURCES}
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(lifecycle_manager_node src/lifecycle_manager_node.cpp)
ament_target_dependencies(lifecycle_manager_node ${dependencies})
target_link_libraries(lifecycle_manager_node ${PROJECT_NAME})


install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  lifecycle_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
