set(sources
  #src/RobotAssemblerSettings.cpp
  #src/RobotAssembler.cpp
  #src/RobotAssemblerBody.cpp
  #src/RobotAssemblerHelper.cpp
  #src/RobotAssemblerInfo.cpp
  src/RobotAssemblerPlugin.cpp
  src/AssemblerItem.cpp
  src/AssemblerManager.cpp
  src/AssemblerView.cpp
  src/AssemblerBar.cpp
  src/AssemblerSwapModeBar.cpp
  src/AssemblerTreeView.cpp
  src/AssemblerPartsView.cpp
  src/AssemblerForBase.cpp
  )
set(headers
  #src/irsl_choreonoid/Coordinates.h
  #src/RobotAssemblerSettings.h
  #src/RobotAssembler.h
  #src/RobotAssemblerBody.h
  #src/RobotAssemblerHelper.h
  #src/RobotAssemblerInfo.h
  #src/ValueTreeUtil.h
  src/RobotAssemblerPlugin.h
  src/AssemblerItem.h
  src/AssemblerManager.h
  src/AssemblerView.h
  src/AssemblerBar.h
  src/AssemblerSwapModeBar.h
  src/AssemblerTreeView.h
  src/AssemblerPartsView.h
  src/AssemblerForBase.h
  src/exportdecl.h
  )
set(libsources
  src/RobotAssemblerSettings.cpp
  src/RobotAssembler.cpp
  src/RobotAssemblerBody.cpp
  src/RobotAssemblerHelper.cpp
  src/RobotAssemblerInfo.cpp
  #src/RobotAssemblerPlugin.cpp
  #src/AssemblerItem.cpp
  #src/AssemblerManager.cpp
  #src/AssemblerView.cpp
  #src/AssemblerBar.cpp
  #src/AssemblerSwapModeBar.cpp
  #src/AssemblerTreeView.cpp
  #src/AssemblerPartsView.cpp
  #src/AssemblerForBase.cpp
  )
set(libheaders
  # src/irsl_choreonoid/Coordinates.h
  src/RobotAssemblerSettings.h
  src/RobotAssembler.h
  src/RobotAssemblerBody.h
  src/RobotAssemblerHelper.h
  src/RobotAssemblerInfo.h
  src/ValueTreeUtil.h
  src/exportdecl_lib.h
  #src/RobotAssemblerPlugin.h
  #src/AssemblerItem.h
  #src/AssemblerManager.h
  #src/AssemblerView.h
  #src/AssemblerBar.h
  #src/AssemblerSwapModeBar.h
  #src/AssemblerTreeView.h
  #src/AssemblerPartsView.h
  #src/AssemblerForBase.h
  )
## TODO install irsl_choreonoid/Coordinates.h
if (NOT WIN32)
  add_definitions(-Wno-deprecated-declarations)
endif()
add_definitions(-DIRSL_DEBUG)
option(BUILD_TEST "build executables for test" OFF)

QT5_ADD_RESOURCES(RC_SRCS src/RA.qrc)

if(${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
  Message ("Build : RobotAssemblerPlugin (independent)")
  # Build as an independent project
  cmake_minimum_required(VERSION 3.10)
  project(RobotAssemblerPlugin)
  find_package(Choreonoid REQUIRED)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  set(targetlib RobotAssembler)
  set(targetplugin CnoidRobotAssemblerPlugin)
  choreonoid_add_library(${targetlib} SHARED ${libsources} ${RC_SRCS} HEADERS ${libheaders})
  target_link_libraries(${targetlib} Choreonoid::CnoidBody Choreonoid::CnoidUtil ${CHOREONOID_QT_LIBRARIES})
  choreonoid_add_plugin(${targetplugin} ${sources})#HEADERS ${headers})
  target_link_libraries(${targetplugin} ${targetlib})
  target_link_libraries(${targetplugin} Choreonoid::CnoidBody Choreonoid::CnoidUtil)
  if(BUILD_TEST)
    add_subdirectory(test)
  endif()
else()
  Message ("Build : RobotAssemblerPlugin (under choreonoid/ext)")
  # Build as a bundled project / under ext
  set(targetlib RobotAssembler)
  set(targetplugin CnoidRobotAssemblerPlugin)
  ### [todo] build libRoboassembler.so and link it
  choreonoid_add_library(${targetlib} SHARED ${libsources} ${RC_SRCS} HEADERS ${libheaders})
  target_link_libraries(${targetlib} PUBLIC CnoidBody CnoidUtil ${CHOREONOID_QT_LIBRARIES})
  choreonoid_add_plugin(${targetplugin} ${sources} HEADERS ${headers})
  target_link_libraries(${targetplugin} PUBLIC ${targetlib})
  target_link_libraries(${targetplugin} PUBLIC CnoidBody CnoidUtil)## boost_filesystem
  if(BUILD_TEST)
    add_subdirectory(test)
  endif()
  add_subdirectory(pybind11)
  file(RELATIVE_PATH rel_src_dir ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  install(FILES src/irsl_choreonoid/Coordinates.h DESTINATION ${CHOREONOID_HEADER_SUBDIR}/cnoid/${rel_src_dir}/irsl_choreonoid)
endif()

install(FILES config/assembler.cnoid DESTINATION ${CHOREONOID_SHARE_SUBDIR}/robot_assembler/layout)
install(FILES config/original.cnoid  DESTINATION ${CHOREONOID_SHARE_SUBDIR}/robot_assembler/layout)

install(DIRECTORY tools       DESTINATION ${CHOREONOID_SHARE_SUBDIR}/robot_assembler)
# install(DIRECTORY docs        DESTINATION ${CHOREONOID_SHARE_SUBDIR}/robot_assembler)
