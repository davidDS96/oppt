#!/bin/bash

res="succ"
PKGS=""
BASE_PKG=""

command_exists() {
    type "$1" &> /dev/null ;
}

# Check if command in $1 exists.
# If not, add $2 to the list of installed libraries
checkCMDAndAppend() {
    if ! command_exists $1
    then
       echo $1
       BASE_PKG="$BASE_PKG $2"
    else 
       echo "$2 is already installed. Skipping..."
    fi
}

# Check if package in $1 exists and can be found by pkg-config. 
# If not, add $2 to the list of installed libraries
checkPKGAndAppend() {
    $(pkg-config --exists $1) &> /dev/null    
    if [ $? -eq 0 ]; then
        echo "$1 found by pkg-config. Skipping..."
    else        
        PKGS="$PKGS $2"
    fi 
}

# Check if package $1 can be found by dpkg. 
# If not, add it to the list of installed packages
checkAndAppend() {
    dpkg -s $1 &> /dev/null
    if [ $? -eq 0 ]; then
        echo "$1 already installed. Skipping..."
    else
        PKGS="$PKGS $1"
    fi  
}

# Check the current status of the script
checkStatus() {
    if [ $res != "succ" ]; then
        echo "Installation failed"
        exit
    fi
}

# Detect Ubuntu version
detect_ubuntu_version() {
    UBUNTU_MAJOR_VERSION=$(lsb_release -cs)
    ARCH=$(uname -m)    
    if [ $UBUNTU_MAJOR_VERSION == "wily" ]
    then
       ROS_DISTR="indigo"
       if [ $ARCH == "x86_64" ]
       then      
          GAZEBO_SOURCES="http://packages.osrfoundation.org/gazebo/ubuntu-stable"
          ROS_DISTR="kinetic"
          GAZEBO_PKG="gazebo-stable.list"
       else
          echo "Detected a compatible Ubuntu release $UBUNTU_MAJOR_VERSION but incompatible architecture $ARCH"
          res="fail"
       fi
    elif [ $UBUNTU_MAJOR_VERSION == "xenial" ]
    then
       ROS_DISTR="kinetic"
       GAZEBO_SOURCES="http://packages.osrfoundation.org/gazebo/ubuntu-stable"
       GAZEBO_PKG="gazebo-stable.list"
    else
       echo "Your Ubuntu version is too old. OPPT requires Ubuntu 15.10 (Wily) or higher"
       res="fail"
    fi    
}

install_common_dependencies() {
    checkAndAppend build-essential
    checkCMDAndAppend git git 
    checkCMDAndAppend hg mercurial
    checkCMDAndAppend pkg-config pkg-config
    if [ -n "$BASE_PKG" ]; then
        sudo apt-get install $BASE_PKG || res="fail" && return
        sudo ldconfig
    fi
    
    checkAndAppend libboost-all-dev
    checkAndAppend libtinyxml-dev
    checkPKGAndAppend eigen3 libeigen3-dev
    checkPKGAndAppend assimp libassimp-dev
    checkPKGAndAppend fcl libfcl-dev
    
    if [ -n "$PKGS" ]; then
        #echo "Packages being installed: $PKGS"
        sudo apt-get install $PKGS || res="fail" && return
    else
        echo "Required base packages already installed"
    fi    
}

install_gazebo_dependencies() {
    # First check if we need to install Gazebo at all    
    if command_exists gzclient
    then
        echo "Gazebo is already installed. Skipping..."
    	return   
    fi
    
	# Installs the dependencies for gazebo-7
	sudo sh -c 'echo "deb '$GAZEBO_SOURCES' `lsb_release -cs` main" > /etc/apt/sources.list.d/'$GAZEBO_PKG''
	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
	wget https://bitbucket.org/osrf/release-tools/raw/default/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh
	sudo apt-get update
	sleep 1
	ROS_DISTRO=$ROS_DISTR GAZEBO_MAJOR_VERSION=7 . /tmp/dependencies.sh 
	sudo apt-get install $(sed 's:\\ ::g' <<< $BASE_DEPENDENCIES) $(sed 's:\\ ::g' <<< $GAZEBO_BASE_DEPENDENCIES)
	sudo apt-get install libsdformat4 libsdformat4-dev libignition-math2-dev libignition-math2 
	sudo apt-get install gazebo7 libgazebo7-dev	
	sleep 1
}

install_ros() {
    # First check if we can run Rviz
    if command_exists rviz
    then
        return
    fi

    # Installs the ros-base variant of ROS plus the visualization stack
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-get update
    sudo apt-get install ros-$ROS_DISTR-ros-base ros-$ROS_DISTR-rviz    
    source /opt/ros/$ROS_DISTR/setup.bash    
}

install_libspatialindex() {
    # Check if libspatialindex is already installed
    siOut=$(ldconfig -p | grep libspatialindex)
    if [ `echo $siOut | grep -c "libspatialindex" ` -gt 0 ]
	then
	    echo "libspatialindex is already installed. Skipping..."
  		return
	fi
    # Downloads, compiles and installs libspatialindex-1.8.5
    wget http://download.osgeo.org/libspatialindex/spatialindex-src-1.8.5.tar.gz
    tar zxfv spatialindex-src-1.8.5.tar.gz
    cd spatialindex-src-1.8.5
    mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
    make -j2 && sudo make install
    cd ../../
    rm -rf spatialindex-src-1.8.5
    rm spatialindex-src-1.8.5.tar.gz
}

USE_ROS=false
if [ -z $1 ]
then
  echo "Installing without ROS"
else  
  if [ $1 = "--use-ros" ]
  then
      echo "Installing with ROS"
      USE_ROS=true
  else
      echo "Command line option '$1' not recognized. Available option: '--use-ros'"
      exit
  fi
fi

detect_ubuntu_version
checkStatus
install_common_dependencies
checkStatus
install_gazebo_dependencies
checkStatus
if [ "$USE_ROS" = true ]
then   
   install_ros
fi
checkStatus
install_libspatialindex
checkStatus
echo "Done."
