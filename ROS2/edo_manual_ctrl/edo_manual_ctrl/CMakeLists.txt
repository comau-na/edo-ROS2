cmake_minimum_required(VERSION 3.5)
project(edo_manual_ctrl)



# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#######################################################

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(edo_core_msgs REQUIRED)
find_package(Curses REQUIRED)

#######################################################

include_directories(
include
${CURSES_INCLUDE_DIR})

#######################################################
#    cpp files for classes ############################
#######################################################
add_library(CLASS_CPP_FILES SHARED 
src/StateChecker.cpp
src/EdoFunctions.cpp)

target_compile_definitions(CLASS_CPP_FILES
  PRIVATE "MINIMAL_COMPOSITION_DLL")
target_link_libraries(CLASS_CPP_FILES ${CURSES_LIBRARY})
ament_target_dependencies(CLASS_CPP_FILES rclcpp edo_core_msgs)



#######################################################
#      edo functions ##################################
#######################################################
#add_library(EDO_FUNCTIONS SHARED src/EdoFunctions.cpp)

#target_compile_definitions(EDO_FUNCTIONS
  #PRIVATE "MINIMAL_COMPOSITION_DLL")

#ament_target_dependencies(EDO_FUNCTIONS rclcpp edo_core_msgs)


#######################################################

add_executable(sample_cli src/SampleCLI.cpp)
target_link_libraries(sample_cli ${CURSES_LIBRARY} CLASS_CPP_FILES)
ament_target_dependencies(sample_cli rclcpp std_msgs edo_core_msgs ) 

install(TARGETS
  CLASS_CPP_FILES
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS

  sample_cli
  DESTINATION lib/${PROJECT_NAME})


ament_package()
