set(sources
  #src/RobotAssemblerSettings.cpp
  #src/RobotAssembler.cpp
  #src/RobotAssemblerBody.cpp
  src/RobotAssemblerPlugin.cpp
  src/RobotAssemblerHelper.cpp
  src/AssemblerItem.cpp
  src/AssemblerManager.cpp
  src/AssemblerView.cpp
  src/AssemblerBar.cpp
  src/AssemblerTreeView.cpp
  src/AssemblerPartsView.cpp
  #src/RobotAssemblerInfo.cpp
  )
set(headers
  #src/irsl_choreonoid/Coordinates.h
  #src/RobotAssemblerSettings.h
  #src/RobotAssembler.h
  #src/RobotAssemblerBody.h
  src/RobotAssemblerPlugin.h
  src/RobotAssemblerHelper.h
  src/AssemblerItem.h
  src/AssemblerManager.h
  src/AssemblerView.h
  src/AssemblerBar.h
  src/AssemblerTreeView.h
  src/AssemblerPartsView.h
  #src/RobotAssemblerInfo.h
  #src/ValueTreeUtil.h
  )
set(libsources
  src/RobotAssemblerSettings.cpp
  src/RobotAssembler.cpp
  src/RobotAssemblerBody.cpp
  #src/RobotAssemblerPlugin.cpp
  #src/RobotAssemblerHelper.cpp
  #src/AssemblerItem.cpp
  #src/AssemblerManager.cpp
  #src/AssemblerView.cpp
  #src/AssemblerBar.cpp
  #src/AssemblerTreeView.cpp
  #src/AssemblerPartsView.cpp
  src/RobotAssemblerInfo.cpp
  )
set(libheaders
  src/irsl_choreonoid/Coordinates.h
  src/RobotAssemblerSettings.h
  src/RobotAssembler.h
  src/RobotAssemblerBody.h
  #src/RobotAssemblerPlugin.h
  #src/RobotAssemblerHelper.h
  #src/AssemblerItem.h
  #src/AssemblerManager.h
  #src/AssemblerView.h
  #src/AssemblerBar.h
  #src/AssemblerTreeView.h
  #src/AssemblerPartsView.h
  src/RobotAssemblerInfo.h
  src/ValueTreeUtil.h
  )
## TODO install irsl_choreonoid/Coordinates.h
add_definitions(-Wno-deprecated-declarations)
add_definitions(-DIRSL_DEBUG)
if(${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
  # Build as a master project
  cmake_minimum_required(VERSION 3.10)
  project(RobotAssemblerPlugin)
  find_package(Choreonoid REQUIRED)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  set(targetlib RobotAssmbler)
  set(targetplugin CnoidRobotAssemblerPlugin)
  choreonoid_add_library(${targetlib} SHARED ${libsources} HEADERS ${libheaders})
  target_link_libraries(${targetlib} Choreonoid::CnoidBody Choreonoid::CnoidUtil)
  choreonoid_add_plugin(${targetplugin} ${sources})#HEADERS ${headers})
  target_link_libraries(${targetplugin} Choreonoid::CnoidBody)
  # add_subdirectory(test)
else()
  Message ("Build : RobotAssemblerPlugin")
  # Build as a bundled project
  set(targetlib RobotAssmbler)
  set(targetplugin CnoidRobotAssemblerPlugin)
  ### [todo] build libRoboassmbler.so and link it
  choreonoid_add_library(${targetlib} SHARED ${libsources} HEADERS ${libheaders})
  target_link_libraries(${targetlib} CnoidBody CnoidUtil)
  choreonoid_add_plugin(${targetplugin} ${sources} HEADERS ${headers})
  target_link_libraries(${targetplugin} ${targetlib})
  target_link_libraries(${targetplugin} CnoidBody CnoidUtil)## boost_filesystem
  # add_subdirectory(test)
endif()
