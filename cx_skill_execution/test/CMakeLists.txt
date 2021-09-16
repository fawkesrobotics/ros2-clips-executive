set(TEST_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_launch_files)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

add_subdirectory(unit)
