cmake_minimum_required(VERSION 3.5)
project(edo_core_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/JointState.msg"
   "msg/JointStateArray.msg"
   "msg/JointControl.msg"
   "msg/JointControlArray.msg"
   "msg/MovementCommand.msg"
   "msg/MovementFeedback.msg"
   "msg/JointConfiguration.msg"
   "msg/JointInit.msg"
   "msg/JointConfigurationArray.msg"
   "msg/JointCalibration.msg"
   "msg/JointReset.msg"
   "msg/CartesianPose.msg"
   "msg/JointsPositions.msg"
   "msg/JointMonitoring.msg"
   "msg/MachineState.msg"
   "msg/NodeSwVersion.msg"
   "msg/NodeSwVersionArray.msg"
   "msg/JointFwVersion.msg"
   "msg/JointFwVersionArray.msg"
   "msg/Point.msg"
   "msg/Frame.msg"
   "msg/SystemCommand.msg"
   )
     #CollisionThreshold.msg
  # "msg/CollisionAlgoToState.msg"
   #"msg/BrakesCheckAck.msg"
   #"msg/Payload.msg"
   #"msg/AppStateArray.msg"
  # "msg/AppState.msg"

set (srv_files
  "srv/JointsNumber.srv"
   "srv/DirectKinematics.srv"
   "srv/InverseKinematics.srv"
   "srv/SoftwareVersion.srv"
   "srv/ControlSwitch.srv"
   "srv/LoadConfigurationFile.srv"
   )
   #"srv/SystemCommandSrv.srv"

rosidl_generate_interfaces(${PROJECT_NAME}
   
  ${msg_files}
  ${srv_files}
   
 )
 ament_export_dependencies(rosidl_default_runtime)

ament_package()
