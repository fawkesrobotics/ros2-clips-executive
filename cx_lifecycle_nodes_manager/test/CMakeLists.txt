ament_add_gtest(lc_nodes_manager_test lc_nodes_manager_test.cpp)
ament_target_dependencies(lc_nodes_manager_test ${dependencies} cx_clips cx_features PkgConfig)
target_link_libraries(lc_nodes_manager_test ${PROJECT_NAME})
pkg_check_modules(CLIPSMM REQUIRED IMPORTED_TARGET clipsmm-1.0)
target_link_libraries(lc_nodes_manager_test PkgConfig::CLIPSMM)

