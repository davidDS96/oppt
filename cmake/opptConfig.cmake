# ...
 # (compute PREFIX relative to file location)
 # ...
set(oppt_INCLUDE_DIRS /usr/local/include)
set(oppt_LIBRARIES /usr/local/lib/liboppt.so)
set(oppt_LIBRARY_DIRS /usr/local/lib;;;;/home/marcus/PhD/scripts/ros_catkin_ws/install_isolated/lib;/usr/local/lib;/usr/local/lib/gazebo-7/plugins;/usr/lib/x86_64-linux-gnu;/usr/lib/x86_64-linux-gnu)

add_definitions(-DUSE_DOUBLE_PRECISION=true)

find_package(gazebo REQUIRED)
if(NOT gazebo_FOUND)
   message(FATAL_ERROR "Gazebo could not be found")
endif()

FIND_PATH( GAZEBO_INCLUDE_PATH "gazebo.hh"
           PATH_SUFFIXES "gazebo-${GAZEBO_MAJOR_VERSION}/gazebo" )
if (NOT GAZEBO_INCLUDE_PATH)
    message(FATAL_ERROR "Gazebo header could not be found")
endif()
list(APPEND GAZEBO_INCLUDE_DIRS "${GAZEBO_INCLUDE_PATH}")

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND oppt_LIBRARIES /usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_timer.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libpthread.so;fcl;/usr/local/lib/libgazebo.so;/usr/local/lib/libgazebo_client.so;/usr/local/lib/libgazebo_gui.so;/usr/local/lib/libgazebo_sensors.so;/usr/local/lib/libgazebo_rendering.so;/usr/local/lib/libgazebo_physics.so;/usr/local/lib/libgazebo_ode.so;/usr/local/lib/libgazebo_transport.so;/usr/local/lib/libgazebo_msgs.so;/usr/local/lib/libgazebo_util.so;/usr/local/lib/libgazebo_common.so;/usr/local/lib/libgazebo_gimpact.so;/usr/local/lib/libgazebo_opcode.so;/usr/local/lib/libgazebo_opende_ou.so;/usr/local/lib/libgazebo_math.so;/usr/lib/x86_64-linux-gnu/libprotobuf.so;-lpthread;/usr/lib/x86_64-linux-gnu/libsdformat.so;/usr/lib/x86_64-linux-gnu/libignition-math2.so;optimized;/usr/lib/x86_64-linux-gnu/libOgreMain.so;debug;/usr/lib/x86_64-linux-gnu/libOgreMain.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libpthread.so;optimized;/usr/lib/x86_64-linux-gnu/libOgreTerrain.so;debug;/usr/lib/x86_64-linux-gnu/libOgreTerrain.so;optimized;/usr/lib/x86_64-linux-gnu/libOgrePaging.so;debug;/usr/lib/x86_64-linux-gnu/libOgrePaging.so;/usr/lib/x86_64-linux-gnu/libignition-math2.so;assimp;/usr/lib/x86_64-linux-gnu/libtinyxml.so;/usr/local/lib/libspatialindex.so;/home/marcus/PhD/scripts/ros_catkin_ws/install_isolated/lib/libroscpp.so;/usr/lib/x86_64-linux-gnu/libboost_signals.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/home/marcus/PhD/scripts/ros_catkin_ws/install_isolated/lib/librosconsole.so;/home/marcus/PhD/scripts/ros_catkin_ws/install_isolated/lib/librosconsole_print.so;/home/marcus/PhD/scripts/ros_catkin_ws/install_isolated/lib/librosconsole_backend_interface.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so;/home/marcus/PhD/scripts/ros_catkin_ws/install_isolated/lib/libroscpp_serialization.so;/home/marcus/PhD/scripts/ros_catkin_ws/install_isolated/lib/librostime.so;/home/marcus/PhD/scripts/ros_catkin_ws/install_isolated/lib/libxmlrpcpp.so;/home/marcus/PhD/scripts/ros_catkin_ws/install_isolated/lib/libcpp_common.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libpthread.so;/usr/local/lib/libconsole_bridge.so.0.3;kdl_parser;urdf;rosconsole_bridge;roscpp;pthread;rosconsole;rosconsole_print;rosconsole_backend_interface;roscpp_serialization;rostime;xmlrpcpp;cpp_common;trac_ik;kdl_parser;urdf;rosconsole_bridge;roscpp;pthread;rosconsole;rosconsole_print;rosconsole_backend_interface;roscpp_serialization;rostime;xmlrpcpp;cpp_common;dl)
