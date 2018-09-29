set(ROBOT_ENVIRONMENT_SRC   
   ${CMAKE_CURRENT_LIST_DIR}/RobotEnvironment.cpp
            ${CMAKE_CURRENT_LIST_DIR}/Obstacle.cpp 
            ${CMAKE_CURRENT_LIST_DIR}/SceneImpl.cpp          
            ${CMAKE_CURRENT_LIST_DIR}/BoxObstacle.cpp
            ${CMAKE_CURRENT_LIST_DIR}/SphereObstacle.cpp
            ${CMAKE_CURRENT_LIST_DIR}/MeshObstacle.cpp
            ${CMAKE_CURRENT_LIST_DIR}/CylinderObstacle.cpp
            ${CMAKE_CURRENT_LIST_DIR}/Mesh.cpp
            ${CMAKE_CURRENT_LIST_DIR}/Box.cpp
            ${CMAKE_CURRENT_LIST_DIR}/Sphere.cpp
            ${CMAKE_CURRENT_LIST_DIR}/Cylinder.cpp
            ${CMAKE_CURRENT_LIST_DIR}/SDFParser.cpp
            ${CMAKE_CURRENT_LIST_DIR}/CollisionObject.cpp)
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/oppt/
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hxx" PATTERN "*.hpp"
  PATTERN ".DS_Store" EXCLUDE
)
