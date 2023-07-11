###
set(TARGET PyRobotAssembler)

add_cnoid_python_module(${TARGET}
  PyRobotAssemblerModule.cpp
  PyRobotAssemblerSettings.cpp
  PyRobotAssembler.cpp
  PyRobotAssemblerHelper.cpp
  )

if(${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
  # Build as an independent project
  target_link_libraries(${TARGET}
    ${catkin_LIBRARIES}
    ${CHOREONOID_BODY_LIBRARIES}
    )
  target_compile_definitions(${TARGET} PUBLIC ${CHOREONOID_COMPILE_DEFINITIONS})
  ## Not tested
  message("CHOREONOID_BIN_DIR: ${CHOREONOID_BIN_DIR}")
  if ("${CHOREONOID_BIN_DIR}" MATCHES ".*install.*")
    ## install
    message("in INSTALLSPACE")
    install(TARGETS ${TARGET} LIBRARY DESTINATION ${choreonoid_PYTHON_DIR})
  else()
    ## devel
    message("in DEVELSPACE")
    set_target_properties(${TARGET}
      PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PYTHON_DIR})# copy the library for calling 'import cnoid.hoge'
  endif()
else()
  # Build as a bundled project / under ext
  target_link_libraries(${TARGET}
    ${catkin_LIBRARIES}
    RobotAssembler CnoidBody CnoidUtil
    )
endif()
