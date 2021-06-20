cmake_minimum_required(VERSION 3.8)
project(cx_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

set(MSGS
  "msg/ClipsEnvironment.msg"
)

# set(SERVICES

# )

# set(ACTIONS

# )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSGS}
  # ${services}
  # ${actions}
  DEPENDENCIES std_msgs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
