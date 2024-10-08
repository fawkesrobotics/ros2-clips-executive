cmake_minimum_required(VERSION 3.8)
project(cx_clips_executive)

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
find_package(cx_clips REQUIRED)
find_package(cx_utils REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(clips_vendor REQUIRED)
find_package(clips REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  cx_clips
  cx_utils
  yaml-cpp
  std_msgs
  clips
)

include_directories(include)

set(CLIPS_EXECUTIVE_SOURCES
  src/cx_clips_executive/ClipsExecutive.cpp
#   src/cx_clips_executive/
)

add_library(${PROJECT_NAME} SHARED ${CLIPS_EXECUTIVE_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ClipsNS::libclips_ns)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

add_executable(clips_executive_node src/clips_executive_node.cpp)

ament_target_dependencies(clips_executive_node ${dependencies})
target_link_libraries(clips_executive_node ${PROJECT_NAME})
target_link_libraries(clips_executive_node ClipsNS::libclips_ns)
target_link_libraries(clips_executive_node yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(plansys2_problem_expert REQUIRED)
  find_package(plansys2_domain_expert REQUIRED)
  find_package(plansys2_planner REQUIRED)
  find_package(pluginlib REQUIRED)
  add_subdirectory(test)
endif()

install(
  DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY clips DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
  ${PROJECT_NAME}
  clips_executive_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
