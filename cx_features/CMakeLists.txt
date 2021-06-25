cmake_minimum_required(VERSION 3.8)
project(cx_features)

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
find_package(PkgConfig REQUIRED)
find_package(cx_clips REQUIRED)
find_package(cx_core REQUIRED)
find_package(cx_msgs REQUIRED)
find_package(cx_utils REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  PkgConfig
  cx_clips
  cx_core
  cx_msgs
  cx_utils
)

set(CX_FEATURES_SOURCES
   src/cx_features/MockFeature.cpp
   src/cx_features/ClipsFeaturesManager.cpp
)

pkg_check_modules(CLIPSMM REQUIRED IMPORTED_TARGET clipsmm-1.0)

include_directories(include)

add_library(${PROJECT_NAME} SHARED ${CX_FEATURES_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} PkgConfig::CLIPSMM)

add_executable(main src/main.cpp)

ament_target_dependencies(main ${dependencies})
target_link_libraries(main ${PROJECT_NAME})
target_link_libraries(main PkgConfig::CLIPSMM)


install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  main
  ${PROJECT_NAME}
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
