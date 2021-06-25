set(TEST_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_launch_files)

ament_add_gtest(test_mock_feature test_mock_feature.cpp)
ament_target_dependencies(test_mock_feature ${dependencies})
target_link_libraries(test_mock_feature ${PROJECT_NAME})
