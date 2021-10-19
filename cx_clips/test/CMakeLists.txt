set(TEST_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_launch_files)

ament_add_gtest(clips_manager_test clips_manager_test.cpp)
ament_target_dependencies(clips_manager_test ${dependencies})
target_link_libraries(clips_manager_test ${PROJECT_NAME})
