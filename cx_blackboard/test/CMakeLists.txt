set(TEST_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_launch_files)

ament_add_gtest(test_interface_nodes test_interface_nodes.cpp)
ament_target_dependencies(test_interface_nodes ${dependencies})
target_link_libraries(test_interface_nodes ${PROJECT_NAME})
