cmake_minimum_required(VERSION 3.8)
project(cx_protobuf_plugin)

# Uses c++ 20 or above
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cx_plugin REQUIRED)
find_package(cx_utils REQUIRED)
find_package(Protobuf REQUIRED)
find_package(ProtobufComm REQUIRED)
find_package(clips_vendor REQUIRED)
find_package(clips REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  PkgConfig
  cx_plugin
  cx_utils
  pluginlib
)

include_directories(include)

pluginlib_export_plugin_description_file(cx_plugin protobuf_plugin.xml)

add_library(clips_protobuf_comm SHARED src/communicator.cpp)
target_link_libraries(
  clips_protobuf_comm protobuf::libprotobuf ProtobufComm::protobuf_comm
  ClipsNS::libclips_ns spdlog::spdlog)
target_include_directories(
  clips_protobuf_comm PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/> $<INSTALL_INTERFACE:include/>)
set_property(TARGET clips_protobuf_comm PROPERTY CXX_STANDARD ${CMAKE_CXX_STANDARD})

add_library(${PROJECT_NAME} SHARED src/protobuf_plugin.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} clips_protobuf_comm)
target_link_libraries(${PROJECT_NAME} ClipsNS::libclips_ns)

# END OF PLUGINS REGISTRATION
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES protobuf_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY clips/
  DESTINATION share/${PROJECT_NAME}/clips/${PROJECT_NAME}
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
