cmake_minimum_required(VERSION 3.8)
project(cx_protobuf_feature)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set(ryml_DIR /home/userin4o/rapidyaml/install/lib/cmake/ryml)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cx_feature REQUIRED)
find_package(cx_utils REQUIRED)
find_package(ClipsProtobuf REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  PkgConfig
  cx_feature
  cx_utils
  ClipsProtobuf
  pluginlib
)

include_directories(include)

pluginlib_export_plugin_description_file(cx_feature protobuf_feature_plugin.xml)

add_library(${PROJECT_NAME} SHARED src/protobuf_feature.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ClipsProtobuf::clips_protobuf)
target_link_libraries(${PROJECT_NAME} ClipsNS::libclips_ns)

# END OF PLUGINS REGISTRATION
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES protobuf_feature_plugin.xml
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
