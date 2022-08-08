set(sources
  src/RobotAssemblerSettings.cpp
  src/RobotAssembler.cpp
  src/RobotAssemblerBody.cpp
  src/RobotAssemblerPlugin.cpp
  src/RobotAssemblerHelper.cpp
  src/AssemblerItem.cpp
  src/AssemblerManager.cpp
  src/AssemblerView.cpp
  src/AssemblerBar.cpp
  src/AssemblerTreeView.cpp
  )
set(headers
  src/irsl_choreonoid/Coordinates.h
  src/RobotAssemblerSettings.h
  src/RobotAssembler.h
  src/RobotAssemblerBody.h
  src/RobotAssemblerPlugin.h
  src/RobotAssemblerHelper.h
  src/AssemblerItem.h
  src/AssemblerManager.h
  src/AssemblerView.h
  src/AssemblerBar.h
  src/AssemblerTreeView.h
  )
add_definitions(-Wno-deprecated-declarations)
if(${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
  # Build as a master project
  cmake_minimum_required(VERSION 3.10)
  project(RobotAssemblerPlugin)
  find_package(Choreonoid REQUIRED)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  choreonoid_add_plugin(CnoidRobotAssemblerPlugin ${sources})
  target_link_libraries(CnoidRobotAssemblerPlugin Choreonoid::CnoidBody)
  ### [todo] build libRoboassmbler.so and link it

  # add_subdirectory(test)
else()
  Message ("Build : RobotAssemblerPlugin")
  # Build as a bundled project
  choreonoid_add_plugin(CnoidRobotAssemblerPlugin ${sources} HEADERS ${headers})
  target_link_libraries(CnoidRobotAssemblerPlugin CnoidBody CnoidUtil)## boost_filesystem
endif()
