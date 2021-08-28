cmake_minimum_required(VERSION 3.8)
project(cx_blackboard)

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
find_package(cx_msgs REQUIRED)
find_package(cx_utils REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(test_msgs REQUIRED)
find_package(ros2_introspection REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  cx_msgs
  cx_utils
  rosbag2_cpp
  test_msgs
  ros2_introspection
)

include_directories(include)

set(BLACKBOARD_SOURCES
  src/cx_blackboard/BlackboardNode.cpp
  src/cx_blackboard/InterfaceManager.cpp
  src/cx_blackboard/InterfaceNode.cpp
)

add_library(${PROJECT_NAME} SHARED ${BLACKBOARD_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# add_executable(blackboard_node src/blackboard.cpp)
# ament_target_dependencies(blackboard_node ${dependencies})
# target_link_libraries(blackboard_node ${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  # blackboard_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
