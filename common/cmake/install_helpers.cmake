# Configure selected helpers into each consumer's build tree.
set(_demo_start_controllers_settings NODE_PREFIX LABEL HARDWARE CONTROLLER)
set(_demo_check_ready_settings NODE_PREFIX SIMULATOR JOINTS CONTROLLER)
set(_demo_verify_motion_settings NODE_PREFIX SIMULATOR LABEL BASE_FRAME EE_FRAME CHECK_TF_AGE USE_SIM_TIME)
set(_demo_resume_gazebo_when_controller_ready_settings "")
if(NOT DEFINED DEMO_HELPERS)
  message(FATAL_ERROR "Missing DEMO_HELPERS for ${PROJECT_NAME}")
endif()
foreach(_demo_helper IN LISTS DEMO_HELPERS)
  if(NOT DEFINED _demo_${_demo_helper}_settings)
    message(FATAL_ERROR "Unknown demo helper: ${_demo_helper}")
  endif()
  foreach(_demo_setting IN LISTS _demo_${_demo_helper}_settings)
    if(NOT DEFINED DEMO_${_demo_setting})
      message(FATAL_ERROR "Missing DEMO_${_demo_setting} for ${PROJECT_NAME}/${_demo_helper}")
    endif()
  endforeach()
  configure_file(
    "${CMAKE_CURRENT_LIST_DIR}/../scripts/${_demo_helper}.py.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${_demo_helper}.py"
    @ONLY)
  install(PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/${_demo_helper}.py"
    DESTINATION lib/${PROJECT_NAME})
endforeach()
