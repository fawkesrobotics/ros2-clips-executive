ament_add_gtest(cx_with_psys2 clips_executive_with_psys2.cpp)
ament_target_dependencies(cx_with_psys2 ${dependencies} plansys2_problem_expert plansys2_domain_expert plansys2_planner)
target_link_libraries(cx_with_psys2 ${PROJECT_NAME})