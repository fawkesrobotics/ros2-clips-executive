cmake_minimum_required(VERSION 3.8)
project(cx_utils)

# packages to ignore: colcon build --symlink-install --packages-ignore plansys2_executor plansys2_bt_actions plansys2_terminal plansys2_bringup plansys2_tests

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

set(dependencies
  ament_cmake
  rclcpp
)

include_directories(include)

set(CX_UTILS_SOURCES
  src/LockSharedPtr.cpp
  src/NodeThread.cpp
  src/map_skill.cpp
)

add_library(${PROJECT_NAME} SHARED ${CX_UTILS_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY ros_comm_generator/templates
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES ros_comm_generator/generator.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
