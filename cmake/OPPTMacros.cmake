MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

macro(ADD_TRANSITION_PLUGIN name src)
   set( _SRC_NAMES_LIST ${src} ${ARGN} )   
   add_library(${name} SHARED ${_SRC_NAMES_LIST})
   if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT) 
       message("AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHh")    
       set_target_properties(${name}
                             PROPERTIES
                             LIBRARY_OUTPUT_DIRECTORY ${ROOT_PATH}/../plugins/transitionPlugins)
   endif()   
   install(TARGETS ${name} DESTINATION ${CMAKE_INSTALL_DATADIR}/oppt/plugins/transitionPlugins/)
endmacro()

macro(ADD_OBSERVATION_PLUGIN name src)
   set( _SRC_NAMES_LIST ${src} ${ARGN} )   
   add_library(${name} SHARED ${_SRC_NAMES_LIST})  
   if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)  
	   set_target_properties(${name}
		                     PROPERTIES
		                     LIBRARY_OUTPUT_DIRECTORY ${ROOT_PATH}/../plugins/observationPlugins)
   endif()  
   install(TARGETS ${name} DESTINATION ${CMAKE_INSTALL_DATADIR}/oppt/plugins/observationPlugins/)
endmacro()

macro(ADD_INITIAL_BELIEF_PLUGIN name src)
   set( _SRC_NAMES_LIST ${src} ${ARGN} )   
   add_library(${name} SHARED ${_SRC_NAMES_LIST})
   if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)    
   set_target_properties(${name}
                         PROPERTIES
                         LIBRARY_OUTPUT_DIRECTORY ${ROOT_PATH}/../plugins/initialBeliefPlugins)
   endif()   
   install(TARGETS ${name} DESTINATION ${CMAKE_INSTALL_DATADIR}/oppt/plugins/initialBeliefPlugins/)
endmacro()

macro(ADD_TERMINAL_PLUGIN name src)
   set( _SRC_NAMES_LIST ${src} ${ARGN} )   
   add_library(${name} SHARED ${_SRC_NAMES_LIST})
   if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)    
   set_target_properties(${name}
                         PROPERTIES
                         LIBRARY_OUTPUT_DIRECTORY ${ROOT_PATH}/../plugins/terminalPlugins)
   endif()   
   install(TARGETS ${name} DESTINATION ${CMAKE_INSTALL_DATADIR}/oppt/plugins/terminalPlugins/)
endmacro()

macro(ADD_HEURISTIC_PLUGIN name src)   
   set( _SRC_NAMES_LIST ${src} ${ARGN} )   
   add_library(${name} SHARED ${_SRC_NAMES_LIST})
   if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)   
   set_target_properties(${name}
                         PROPERTIES
                         LIBRARY_OUTPUT_DIRECTORY ${ROOT_PATH}/../plugins/heuristicPlugins)
   endif()   
   install(TARGETS ${name} DESTINATION ${CMAKE_INSTALL_DATADIR}/oppt/plugins/heuristicPlugins/)
endmacro()

macro(ADD_REWARD_PLUGIN name src)   
   set( _SRC_NAMES_LIST ${src} ${ARGN} )   
   add_library(${name} SHARED ${_SRC_NAMES_LIST})
   if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)   
   set_target_properties(${name}
                         PROPERTIES
                         LIBRARY_OUTPUT_DIRECTORY ${ROOT_PATH}/../plugins/rewardPlugins) 
   endif()  
   install(TARGETS ${name} DESTINATION ${CMAKE_INSTALL_DATADIR}/oppt/plugins/rewardPlugins/)
endmacro()
