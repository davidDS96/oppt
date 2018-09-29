SET(BOOST_MIN_VERSION "1.55.0")

include(FindPkgConfig)
include(GNUInstallDirs)

set(oppt_LIBRARY_DIRS "")

############################# LOAD TINYXML #############################
include(${CMAKE_CURRENT_LIST_DIR}/FindTinyXML.cmake)
if (NOT TINYXML_FOUND)
    message(FATAL_ERROR "TinyXML could not be found")
endif()
include_directories(${TINYXML_INCLUDE_DIRS})
link_directories(${TINYXML_LIBRARY_DIRS})
list(APPEND oppt_LIBRARY_DIRS "${TINYXML_LIBRARY_DIRS}")

############################# LOAD LIBSPATIALINDEX #############################
include(${CMAKE_CURRENT_LIST_DIR}/FindSpatialIndex.cmake)
if (NOT SPATIALINDEX_FOUND)
    message(FATAL_ERROR "libspatialindex could not be found")
endif()
include_directories(${SPATIALINDEX_INCLUDE_DIRS})
link_directories(${SPATIALINDEX_LIBRARY_DIRS})
list(APPEND oppt_LIBRARY_DIRS "${SPATIALINDEX_LIBRARY_DIRS}")

############################# LOAD SDFORMAT #############################
set (SDFormat_VERSION 4.1.0)
find_package(SDFormat ${SDFormat_MIN_VERSION} REQUIRED)
if(NOT SDFormat_FOUND)   
   message(FATAL_ERROR "-- SDF could not be found")
endif()
include_directories(${SDFormat_INCLUDE_DIRS})
link_directories(${SDFormat_LIBRARY_DIRS})
list(APPEND oppt_LIBRARY_DIRS "${SDFormat_LIBRARY_DIRS}")   

############################# LOAD EIGEN #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(EIGEN eigen3)
    if(NOT EIGEN_FOUND)
       message(FATAL_ERROR "EIGEN could not be found")
    endif()
    include_directories(${EIGEN_INCLUDE_DIRS})    
endif()

############################# LOAD ASSIMP #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(ASSIMP assimp)
    if(NOT ASSIMP_FOUND)
       message(FATAL_ERROR "ASSIMP could not be found")
    endif()       
endif()
include_directories(${ASSIMP_INCLUDE_DIRS})
link_directories(${ASSIMP_LIBRARY_DIRS})
list(APPEND oppt_LIBRARY_DIRS "${ASSIMP_LIBRARY_DIRS}")   

############################# LOAD FCL #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(FCL fcl)    
    if(NOT FCL_FOUND)
       message(FATAL_ERROR "FCL could not be found")
    endif()
    if (FCL_VERSION GREATER 0.4.0)
        add_definitions(-DFCL_GT_0_4)
    endif()    
    include_directories(${FCL_INCLUDE_DIRS})
    link_directories(${FCL_LIBRARY_DIRS}) 
    message("-- FCL LIB DIRS ${FCL_LIBRARY_DIRS}")    
endif()
list(APPEND oppt_LIBRARY_DIRS "${FCL_LIBRARY_DIRS}")   

############################# LOAD ROS #############################
set(USE_RVIZ False)
#find_package(catkin COMPONENTS roscpp rviz QUIET)
find_package(roscpp REQUIRED)
#find_package(rviz REQUIRED)
if (roscpp_FOUND)
  message("-- ROS and Rviz have been found compiling with viewer support")
  set(USE_RVIZ True)
  add_definitions(-DUSE_RVIZ)
  include_directories(${roscpp_INCLUDE_DIRS})
  link_directories(${roscpp_LIBRARY_DIRS})
  list(APPEND oppt_LIBRARY_DIRS "${roscpp_LIBRARY_DIRS}")   
endif()
#set(USE_RVIZ False)

############################# LOAD GAZEBO #############################
include(${CMAKE_CURRENT_LIST_DIR}/FindGazebo.cmake)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND oppt_LIBRARY_DIRS "${GAZEBO_LIBRARY_DIRS}") 

############################# LOAD KDL PARSER #########################
set(SUPPORTS_IK True)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(KDL_PARSER kdl_parser)
    pkg_check_modules(TRAC_IK trac_ik_lib)    
    if(NOT KDL_PARSER_FOUND)
       message("-- kdl_parser could not be found. Building without IK support")
       set(SUPPORTS_IK False)
    endif()
    if(NOT TRAC_IK_FOUND)
       message("-- trac_ik_lib could not be found. Building without IK support")
       set(SUPPORTS_IK False)
    endif()   
    if (SUPPORTS_IK)       
       add_definitions(-DSUPPORTS_IK)
       include_directories(${KDL_PARSER_INCLUDE_DIRS})
       link_directories(${KDL_PARSER_LIBRARY_DIRS}) 
       include_directories(${TRAC_IK_INCLUDE_DIRS})
       link_directories(${TRAC_IK_LIBRARY_DIRS})
    endif()
endif()

############################# LOAD BOOST #############################
find_package(Boost ${BOOST_MIN_VERSION} 
             REQUIRED 
             system 
             thread 
             timer 
             filesystem 
             serialization)
if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
    list(APPEND oppt_LIBRARY_DIRS "${Boost_LIBRARY_DIRS}") 
endif ()



