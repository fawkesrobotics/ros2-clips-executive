cmake_minimum_required(VERSION 3.8)
project(cx_plansys2_feature)

# Default to C++17
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
find_package(pluginlib REQUIRED)
find_package(cx_core REQUIRED)
find_package(cx_utils REQUIRED)
find_package(clips_vendor REQUIRED)
find_package(clips REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_planner REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  cx_core
  cx_utils
  pluginlib
  plansys2_problem_expert
  plansys2_domain_expert
  plansys2_planner
)

include_directories(include)

pluginlib_export_plugin_description_file(cx_core plansys2_feature_plugin.xml)

add_library(${PROJECT_NAME} SHARED src/Plansys2Feature.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ClipsNS::libclips_ns)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES plansys2_feature_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
