
set(GTEST_DIR ${CMAKE_CURRENT_BINARY_DIR}/gtest)
set(GTEST_BUILD_DIR ${GTEST_DIR}/build)
set(GTEST_LIBRARY_DIR ${GTEST_DIR}/lib)
set(GTEST_INCLUDE_DIR ${GTEST_DIR}/include)

if(EXISTS ${GTEST_BUILD_DIR})
  execute_process(COMMAND rm -rf ${GTEST_BUILD_DIR})
endif()
execute_process(COMMAND mkdir -p ${GTEST_BUILD_DIR})

if(EXISTS /usr/src/gtest)
  ## build gtest
  message("EXIST GTEST : ${GTEST_BUILD_DIR}")
  execute_process(COMMAND cmake -DCMAKE_INSTALL_PREFIX=${GTEST_DIR} /usr/src/gtest
                  WORKING_DIRECTORY ${GTEST_BUILD_DIR} )
  execute_process(COMMAND make install
                  WORKING_DIRECTORY ${GTEST_BUILD_DIR} )
else()
  ## TODO: build gtest cloning from github?
  message("FAILED: EXIST GTEST")
endif()

### test-sample
link_directories(${GTEST_LIBRARY_DIR} ${CHOREONOID_LIBRARY_DIRS})
add_executable(test-sample test-sample.cpp)
target_include_directories(test-sample PUBLIC ${GTEST_INCLUDE_DIR} ${CHOREONOID_INCLUDE_DIRS})
target_link_libraries(test-sample ${targetlib} gtest gtest_main pthread)

###
set(ldsettings robotassembler_load_settings)
choreonoid_add_executable(${ldsettings} load_settings.cpp)
target_include_directories(${ldsettings} PUBLIC ${CHOREONOID_INCLUDE_DIRS})
target_link_libraries(${ldsettings} ${targetlib} pthread)

###
set(body2urdf choreonoid_body2urdf)
choreonoid_add_executable(${body2urdf} body2urdf.cpp)
target_include_directories(${body2urdf} PUBLIC ${CHOREONOID_INCLUDE_DIRS})
target_link_libraries(${body2urdf} ${targetlib} ${ASSIMP_LIBRARIES} pthread)
