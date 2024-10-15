ament_add_gtest(test_skill_master test_skill_master.cpp)
ament_target_dependencies(test_skill_master ${dependencies})
target_link_libraries(test_skill_master ${PROJECT_NAME})
