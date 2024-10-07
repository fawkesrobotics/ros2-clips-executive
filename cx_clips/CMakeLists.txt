cmake_minimum_required(VERSION 3.8)
project(cx_clips)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(cx_msgs REQUIRED)
find_package(cx_utils REQUIRED)
find_package(cx_core REQUIRED)
find_package(spdlog REQUIRED)
find_package(clips_vendor REQUIRED)
find_package(clips REQUIRED)

# find_package(cx_features REQUIRED)
set(dependencies
  ament_cmake
  ament_index_cpp
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  PkgConfig
  lifecycle_msgs
  cx_msgs
  cx_utils
  cx_core
  spdlog

  # cx_features
)

include_directories(include)

set(CLIPS_SOURCES
  src/cx_clips/CLIPSEnvManagerClient.cpp
  src/cx_clips/CLIPSEnvManagerNode.cpp
)

add_library(${PROJECT_NAME} SHARED ${CLIPS_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} spdlog::spdlog ClipsNS::libclips_ns)

add_executable(clips_node src/clips_node.cpp)

ament_target_dependencies(clips_node ${dependencies})
target_link_libraries(clips_node ${PROJECT_NAME})
target_link_libraries(clips_node ClipsNS::libclips_ns)

if(BUILD_TESTING)
  find_package(ament_lint_auto)

  if(ament_lint_auto_FOUND)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest)

    if(ament_cmake_gtest)
      add_subdirectory(test)
    endif()
  endif()
endif()

install(
  DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY src/${PROJECT_NAME}/clips DESTINATION share/${PROJECT_NAME})

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
