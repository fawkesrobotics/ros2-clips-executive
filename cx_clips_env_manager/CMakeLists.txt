cmake_minimum_required(VERSION 3.8)
project(cx_clips_env_manager)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies of env manager
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(cx_msgs REQUIRED)
find_package(cx_utils REQUIRED)
find_package(clips_vendor REQUIRED)
find_package(clips REQUIRED)

# additional dependencies of plugin manager
find_package(cx_plugin REQUIRED)
find_package(pluginlib REQUIRED)

set(cx_env_manager_deps
  ament_cmake
  ament_index_cpp
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  cx_msgs
  cx_utils
)

set(cx_plugin_manager_deps
  ament_cmake
  rclcpp
  rclcpp_lifecycle
  cx_plugin
  cx_utils
  cx_msgs
  pluginlib
)

include_directories(include)

set(cx_env_manager_sources
  src/clips_env_manager.cpp
)

set(cx_plugin_manager_sources
  src/clips_plugin_manager.cpp
)

add_library(cx_plugin_manager SHARED ${cx_plugin_manager_sources})
ament_target_dependencies(cx_plugin_manager ${cx_plugin_manager_deps})
target_link_libraries(cx_plugin_manager ClipsNS::libclips_ns)

add_library(${PROJECT_NAME} SHARED ${cx_env_manager_sources})
ament_target_dependencies(${PROJECT_NAME} ${cx_env_manager_deps})
target_link_libraries(${PROJECT_NAME} ClipsNS::libclips_ns cx_plugin_manager)

add_executable(clips_node src/clips_node.cpp)

ament_target_dependencies(clips_node ${dependencies})
target_link_libraries(clips_node ${PROJECT_NAME})
target_link_libraries(clips_node spdlog::spdlog ClipsNS::libclips_ns)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  cx_plugin_manager
  clips_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} cx_plugin_manager)
ament_export_dependencies(${cx_env_manager_deps} ${cx_plugin_manager_deps})

ament_package()
