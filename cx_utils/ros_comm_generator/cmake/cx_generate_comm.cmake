function(cx_helper_to_snake_case input_string output_variable)
    # Step 1: Handle consecutive uppercase letters properly
    # Insert underscores between capital-lower or capital-capital-lower sequences.
    string(REGEX REPLACE "([A-Z]+)([A-Z][a-z])" "\\1_\\2" temp_string "${input_string}")
    # Step 2: Replace remaining single capital letters with an underscore before them
    string(REGEX REPLACE "([a-z0-9])([A-Z])" "\\1_\\2" temp_string "${temp_string}")
    # Step 3: Convert the whole string to lowercase
    string(TOLOWER "${temp_string}" temp_string)
    # Step 4: Remove leading underscores, if any
    string(REGEX REPLACE "^_" "" temp_string "${temp_string}")
    # Step 5: Return the result
    set(${output_variable} "${temp_string}" PARENT_SCOPE)
endfunction()

macro(cx_generate_msg_bindings package msg_name)
  # generate bindings to publish/subscribe to ros topics of messages from
  # a given message type (defined by it's package and name).
  # Example: cx_generate_msg_bindings(std_msgs String)
  # Provides the cx::CXStdMsgsStringFeature
  # which defines functions:
  #  - cx-std-msgs-string-feature-subscribe
  #  - cx-std-msgs-string-feature-unsubscribe
  #  - cx-std-msgs-string-feature-set-field
  #  - cx-std-msgs-string-feature-get-field
  #  - cx-std-msgs-string-feature-create-message
  #  - cx-std-msgs-string-feature-create-publisher
  #  - cx-std-msgs-string-feature-publish
  # and templates:

  cx_helper_to_snake_case(${package} snake_case_package)
  cx_helper_to_snake_case(${msg_name} snake_case_msg_name)
  set(feature_name cx_${snake_case_package}_${snake_case_msg_name}_feature)
  message(STATUS "Generate bindings for ${package}/msg/${msg_name}")

  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(${package} REQUIRED)
  find_package(cx_core REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(cx_utils REQUIRED)
  find_package(clips_vendor REQUIRED)
  find_package(clips REQUIRED)

  # Generator script
  set(GENERATOR_SCRIPT "${cx_utils_DIR}/../../../lib/cx_utils/generator.py")
  set(TEMPLATES_DIR "${cx_utils_DIR}/../templates/")
  if(NOT EXISTS "${GENERATOR_SCRIPT}")
      message(FATAL_ERROR "Python script ${GENERATOR_SCRIPT} does not exist")
  endif()
  add_custom_command(
      OUTPUT  ${feature_name}.cpp ${feature_name}.hpp ${feature_name}_plugin.xml
      COMMAND ${Python3_EXECUTABLE} ${GENERATOR_SCRIPT} message ${package} ${msg_name}
      DEPENDS ${GENERATOR_SCRIPT} ${TEMPLATES_DIR}/msg.jinja.cpp ${TEMPLATES_DIR}/msg.jinja.hpp ${TEMPLATES_DIR}/feature_plugin.jinja.xml
      COMMENT "Generate cx feature for ${package} ${msg_name}"
  )

  # Build plugin from library
  add_library(${feature_name} SHARED ${feature_name}.cpp)
  target_link_libraries(${feature_name} ClipsNS::libclips_ns)
  ament_target_dependencies(${feature_name} cx_core pluginlib ${package})
  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/${feature_name}.hpp
    DESTINATION include/
  )
  file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/include)
  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/${feature_name}_plugin.xml
    DESTINATION share/${PROJECT_NAME}
  )
  install(TARGETS
    ${feature_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  get_filename_component(relative_dir "${relative_filename}" DIRECTORY)
  # this is just the relevant part from
  # pluginlib_export_plugin_description_file without checking for existence of the file
  set(plugin_category cx_core)
  set(relative_filename ${feature_name}_plugin.xml)
  set(relative_dir "")
  set(__PLUGINLIB_CATEGORY_CONTENT__${plugin_category}
    "${__PLUGINLIB_CATEGORY_CONTENT__${plugin_category}}share/${PROJECT_NAME}/${relative_filename}\n")
  list(APPEND __PLUGINLIB_PLUGIN_CATEGORIES ${plugin_category})  # duplicates are removes on use
endmacro()
