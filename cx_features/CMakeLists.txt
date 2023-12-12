cmake_minimum_required(VERSION 3.8)
project(cx_features)

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
find_package(PkgConfig REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cx_clips REQUIRED)
find_package(cx_core REQUIRED)
find_package(cx_msgs REQUIRED)
find_package(cx_utils REQUIRED)
find_package(cx_skill_execution REQUIRED)
find_package(ClipsPddlParser REQUIRED)
find_package(ClipsProtobuf REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_planner REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  PkgConfig
  cx_clips
  cx_core
  cx_msgs
  cx_utils
  cx_skill_execution
  ClipsPddlParser
  ClipsProtobuf
  yaml-cpp
  pluginlib
  rosbag2_cpp
  plansys2_problem_expert
  plansys2_domain_expert
  plansys2_planner
)

set(CX_FEATURES_SOURCES
  src/cx_features/redefine_warning_feature/RedefineWarningFeature.cpp
  src/cx_features/config_feature/ConfigFeature.cpp
  src/cx_features/ClipsFeaturesManager.cpp
)

pkg_check_modules(CLIPSMM REQUIRED IMPORTED_TARGET clipsmm-1.0)

include_directories(include)

pluginlib_export_plugin_description_file(cx_core features_plugin.xml)

# All features registered as plugins
add_library(mock_feature SHARED src/cx_features/MockFeature.cpp)
ament_target_dependencies(mock_feature ${dependencies})
target_link_libraries(mock_feature PkgConfig::CLIPSMM)

add_library(plansys2_feature SHARED src/cx_features/plansys2_feature/Plansys2Feature.cpp)
ament_target_dependencies(plansys2_feature ${dependencies})
target_link_libraries(plansys2_feature PkgConfig::CLIPSMM)

add_library(clips_pddl_parser_feature SHARED src/cx_features/clips_pddl_parser_feature/ClipsPddlParserFeature.cpp)
ament_target_dependencies(clips_pddl_parser_feature ${dependencies})
target_link_libraries(clips_pddl_parser_feature ClipsPddlParser::clips_pddl_parser)
target_link_libraries(clips_pddl_parser_feature PkgConfig::CLIPSMM)

add_library(clips_protobuf_feature SHARED src/cx_features/clips_protobuf_feature/ClipsProtobufFeature.cpp)
ament_target_dependencies(clips_protobuf_feature ${dependencies})
target_link_libraries(clips_protobuf_feature ClipsProtobuf::clips_protobuf)
target_link_libraries(clips_protobuf_feature PkgConfig::CLIPSMM)

add_library(skill_execution_feature SHARED src/cx_features/skill_execution_feature/SkillExecutionFeature.cpp)
ament_target_dependencies(skill_execution_feature ${dependencies})
target_link_libraries(skill_execution_feature PkgConfig::CLIPSMM)

add_library(ros_service_requester_feature SHARED src/cx_features/ros_communication_feature/RosServiceRequesterFeature.cpp)
ament_target_dependencies(ros_service_requester_feature ${dependencies})
target_link_libraries(ros_service_requester_feature PkgConfig::CLIPSMM)

add_library(ros_topic_publisher_feature SHARED src/cx_features/ros_communication_feature/RosTopicPublisherFeature.cpp)
ament_target_dependencies(ros_topic_publisher_feature ${dependencies})
target_link_libraries(ros_topic_publisher_feature PkgConfig::CLIPSMM)

add_library(ros_topic_subscriber_feature SHARED src/cx_features/ros_communication_feature/RosTopicSubscriberFeature.cpp)
ament_target_dependencies(ros_topic_subscriber_feature ${dependencies})
target_link_libraries(ros_topic_subscriber_feature PkgConfig::CLIPSMM)

add_library(navgraph_compute SHARED src/cx_features/ros_communication_feature/NavGraphCompute.cpp)
ament_target_dependencies(navgraph_compute ${dependencies})
target_link_libraries(navgraph_compute PkgConfig::CLIPSMM)

add_library(navgraph_set_bounding_box SHARED src/cx_features/ros_communication_feature/NavGraphSetBoundingBox.cpp)
ament_target_dependencies(navgraph_set_bounding_box ${dependencies})
target_link_libraries(navgraph_set_bounding_box PkgConfig::CLIPSMM)

add_library(navgraph_generate_waitzones SHARED src/cx_features/ros_communication_feature/NavGraphGenerateWaitzones.cpp)
ament_target_dependencies(navgraph_generate_waitzones ${dependencies})
target_link_libraries(navgraph_generate_waitzones PkgConfig::CLIPSMM)

add_library(navgraph_update_station_by_tag SHARED src/cx_features/ros_communication_feature/NavGraphUpdateStationByTag.cpp)
ament_target_dependencies(navgraph_update_station_by_tag ${dependencies})
target_link_libraries(navgraph_update_station_by_tag PkgConfig::CLIPSMM)


# END OF PLUGINS REGISTRATION
add_library(${PROJECT_NAME} SHARED ${CX_FEATURES_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} PkgConfig::CLIPSMM)
target_link_libraries(${PROJECT_NAME} yaml-cpp)
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_executable(features_node src/features_node.cpp)
ament_target_dependencies(features_node ${dependencies})
target_link_libraries(features_node ${PROJECT_NAME})
target_link_libraries(features_node PkgConfig::CLIPSMM)
target_link_libraries(features_node yaml-cpp)
target_compile_definitions(features_node PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES features_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  features_node
  mock_feature
  plansys2_feature
  skill_execution_feature
  ros_service_requester_feature
  ros_topic_subscriber_feature
  ros_topic_publisher_feature
  navgraph_compute
  navgraph_set_bounding_box
  navgraph_generate_waitzones
  navgraph_update_station_by_tag
  clips_pddl_parser_feature
  clips_protobuf_feature
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto)

  if(ament_lint_auto_FOUND)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest)
    find_package(cx_clips_executive REQUIRED)

    if(ament_cmake_gtest)
      add_subdirectory(test)
    endif()
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
