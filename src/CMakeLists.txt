cmake_minimum_required (VERSION 2.6)
project (oppt)

cmake_policy(SET CMP0012 NEW)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=c++11 -pg")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og -std=c++11 -pg")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -DEIGEN_NO_DEBUG")

# RelWithDebInfo is the default build type
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE RelWithDebInfo) 
endif()

string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)
if (NOT CMAKE_BUILD_TYPE MATCHES RELWITHDEBINFO AND NOT CMAKE_BUILD_TYPE MATCHES DEBUG AND NOT CMAKE_BUILD_TYPE MATCHES RELEASE)
   message(FATAL_ERROR "${CMAKE_BUILD_TYPE} is not a valid build type. Supported types are Debug, Release and RelWithDebInfo (default)")
endif()
if (CMAKE_BUILD_TYPE MATCHES RELWITHDEBINFO OR CMAKE_BUILD_TYPE MATCHES DEBUG)
   set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
   set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")
endif()


add_definitions(-DUSE_DOUBLE_PRECISION=true)

include(FindPkgConfig)
include(GNUInstallDirs)

set(ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DROOT_PATH=${ROOT_PATH})

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT_PATH}/../bin")
set (OPPT_CMAKE_DIR ${ROOT_PATH}/../cmake)
include (${OPPT_CMAKE_DIR}/SearchForStuff.cmake)
include (${OPPT_CMAKE_DIR}/OPPTMacros.cmake)


include(${ROOT_PATH}/oppt/options/CMakeLists.txt)
include(${ROOT_PATH}/oppt/opptCore/CMakeLists.txt)
include(${ROOT_PATH}/oppt/plugin/CMakeLists.txt)
include(${ROOT_PATH}/oppt/robotEnvironment/CMakeLists.txt)
include(${ROOT_PATH}/oppt/robots/CMakeLists.txt)
include(${ROOT_PATH}/oppt/gazeboInterface/CMakeLists.txt)
include(${ROOT_PATH}/oppt/utils/CMakeLists.txt)
include(${ROOT_PATH}/oppt/filter/CMakeLists.txt)
include(${ROOT_PATH}/oppt/problemEnvironment/CMakeLists.txt)
include(${ROOT_PATH}/oppt/robotHeaders/CMakeLists.txt)
include(${ROOT_PATH}/oppt/solver/CMakeLists.txt)
include(${ROOT_PATH}/plugins/CMakeLists.txt)
include(${ROOT_PATH}/oppt/CMakeLists.txt)
if (USE_RVIZ)
  include(${ROOT_PATH}/oppt/viewerPublisher/CMakeLists.txt)
endif()

########################################

set(COMMON_SRCS
    ${THIRD_PARTY_SRC}
    ${CMAKE_CURRENT_LIST_DIR}/oppt/global.cpp)
    
set(OPPT_SRC
    ${COMMON_SRCS}
    ${OPTIONS_SRC}
    ${ROBOT_PROBLEM_SRC}    
    ${ROBOT_ENVIRONMENT_SRC}
    ${GAZEBO_INTERFACE_SRC}    
    ${ROBOTS_SRC}
    ${ROBOT_IMPL_SRC}    
    ${OPPT_UTILS_SRC}
    ${FILTER_SRC}
    ${VIEWER_PUBLISHER_SRC}    
    ${CHANGES_PARSER_SRC})
    
set(OPPT_LIBS
    ${Boost_LIBRARIES}
    ${FCL_LIBRARIES}                         
    ${TCLAP_LIBRARIES}                        
    ${LIBSPATIALINDEX_LIBRARIES}    
    ${catkin_LIBRARIES}
    ${SDF_LIBRARIES}
    ${GAZEBO_LIBRARIES}          
    ${URDFDOM_LIBRARIES}    
    ${ASSIMP_LIBRARIES}
    ${TINYXML_LIBRARIES}
    ${SPATIALINDEX_LIBRARIES}  
    ${THIRD_PARTY_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${KDL_PARSER_LIBRARIES} 
    ${TRAC_IK_LIBRARIES} 
    ${CMAKE_DL_LIBS})
    
# Add the OPPT library target
add_library( oppt SHARED ${OPPT_SRC} )
target_link_libraries (oppt ${OPPT_LIBS})
#set_target_properties(oppt                      
#                      PROPERTIES
#                      LIBRARY_OUTPUT_DIRECTORY ${ROOT_PATH}/../lib)
install(TARGETS oppt DESTINATION ${CMAKE_INSTALL_LIBDIR})

include(${ROOT_PATH}/solvers/CMakeLists.txt)
if(${USE_RVIZ})
    #include(${ROOT_PATH}/tools/CMakeLists.txt)
    include(${ROOT_PATH}/viewer/CMakeLists.txt)    
endif()

set(cmake_conf_file_in "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/opptConfig.cmake.in")
set(cmake_conf_file_out "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/opptConfig.cmake")
configure_file("${cmake_conf_file_in}" "${cmake_conf_file_out}" @ONLY)
install(FILES ${cmake_conf_file_out} DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/oppt/ COMPONENT cmake)
install(FILES ${ROOT_PATH}/../setup.sh DESTINATION ${CMAKE_INSTALL_DATADIR}/oppt/)
install(DIRECTORY ${ROOT_PATH}/../models DESTINATION ${CMAKE_INSTALL_DATADIR}/oppt/)

find_package(Doxygen)
if (DOXYGEN_FOUND)   
   set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/../docs/Doxyfile.in)
   set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
   configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
   add_custom_target( doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
endif()

