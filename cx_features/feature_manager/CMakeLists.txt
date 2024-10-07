cmake_minimum_required(VERSION 3.8)
project(cx_feature_manager)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# set(ryml_DIR /home/userin4o/rapidyaml/install/lib/cmake/ryml)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cx_feature REQUIRED)
find_package(cx_clips REQUIRED)
find_package(cx_config_feature REQUIRED)
find_package(cx_utils REQUIRED)
find_package(clips_vendor REQUIRED)
find_package(clips REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  rclcpp_lifecycle
  cx_feature
  cx_clips
  cx_utils
  cx_config_feature
  pluginlib
)

set(CX_FEATURES_SOURCES
  src/cx_features/clips_feature_manager.cpp
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/ClipsFeaturesManager.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ClipsNS::libclips_ns)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)

add_executable(features_node src/features_node.cpp)
target_link_libraries(features_node ${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  features_node
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
