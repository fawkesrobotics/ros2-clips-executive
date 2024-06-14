ament_add_gtest(cx_with_psys2 clips_executive_with_psys2.cpp)
ament_target_dependencies(cx_with_psys2 ${dependencies} cx_clips_executive)
target_link_libraries(cx_with_psys2 ${PROJECT_NAME})

ament_add_gtest(clips_pddl_parser_test clips_pddl_parser_test.cpp)
ament_target_dependencies(clips_pddl_parser_test ${dependencies} cx_clips_executive)
target_link_libraries(clips_pddl_parser_test ${PROJECT_NAME})
