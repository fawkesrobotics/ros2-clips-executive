cmake_minimum_required(VERSION 3.8)
project(cx_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

set(MSGS
  "msg/ClipsEnvironment.msg"
  "msg/ClipsFeature.msg"
  "msg/ClipsContext.msg"
  "msg/PddlGenInterface.msg"
  "msg/PddlGenerateMessage.msg"
  "msg/PddlGenInterfaceMessages.msg"
  "msg/SkillExecution.msg"
  "msg/SkillActionExecInfo.msg"
)

set(SERVICES
  "srv/DestroyClipsEnv.srv"
  "srv/CreateClipsEnv.srv"
  "srv/AddClipsFeatures.srv"
  "srv/ClipsFeatureContext.srv"
  "srv/ClipsRemoveFeatures.srv"
  "srv/PddlGenInterfaceSrv.srv"
  "srv/OpenInterface.srv"
  "srv/NavGraphInterfaceCompute.srv"
  "srv/NavGraphInterfaceGenerateWaitzones.srv"
  "srv/NavGraphInterfaceSetBoundingBox.srv"
  "srv/NavGraphInterfaceUpdateStationByTag.srv"
)

# set(ACTIONS

# )
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSGS}
  ${SERVICES}

  # ${actions}
  DEPENDENCIES std_msgs std_srvs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
