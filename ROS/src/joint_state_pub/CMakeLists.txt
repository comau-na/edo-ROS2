
  
cmake_minimum_required(VERSION 2.8.3)
project(edo_core)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  rosbridge_library
  rosbridge_server
  roscpp
  #rosserial_msgs
  #rosserial_client
  std_msgs
  edo_core_msgs
  roslib
  cmake_modules
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/opt/orl/include/")


#find_package(TinyXML REQUIRED)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   #INCLUDE_DIRS include
#  LIBRARIES edo_core_pkg
   CATKIN_DEPENDS  roscpp  std_msgs edo_core_msgs
   DEPENDS TinyXML
)

###########
## Build ##
###########

set( SOURCE_STATE_MACHINE src/edo_state_machine_node.cpp
						  src/StateManager.cpp
						  src/CalibrateState.cpp
						  src/NotCalibrateState.cpp
						  src/State.cpp
						  src/SubscribePublish.cpp
						  src/CommandState.cpp
						  src/InitState.cpp
						  src/CommonService.cpp
						  src/JogState.cpp
						  src/MoveCommandState.cpp
						  src/ErrorState.cpp
						  src/BrakeState.cpp)



# edo_jnt_handler node
add_executable(edo_jnt_handler JointStateArrayPub.cpp)
add_dependencies(edo_jnt_handler ${catkin_EXPORTED_TARGETS})
target_link_libraries(edo_jnt_handler ${catkin_LIBRARIES})



## Specify additional locations of header files
## Your package locations should be listed before other locations
set(NODE_INCLUDE "include/")
include_directories(
# include
  ${NODE_INCLUDE}
  ${catkin_INCLUDE_DIRS}
)

