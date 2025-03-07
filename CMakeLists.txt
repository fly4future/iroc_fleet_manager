cmake_minimum_required(VERSION 3.15.0)
project(iroc_fleet_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-O0)

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  nodelet
  sensor_msgs
  mrs_msgs
  mrs_robot_diagnostics
  mrs_lib
  mrs_mission_manager
  actionlib
  actionlib_msgs
  genmsg
  )

set(LIBRARIES
  IROCFleetManager
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

#actions
add_action_files(
  DIRECTORY action 
  FILES WaypointFleetManager.action
  )
# messages
add_message_files(DIRECTORY msg FILES
  WaypointMissionRobot.msg
  WaypointMissionRobotFeedback.msg
  WaypointMissionInfo.msg
  )
# messages
add_service_files(DIRECTORY srv FILES
  ChangeRobotMissionStateSrv.srv
  )
generate_messages(
  DEPENDENCIES 
  actionlib_msgs 
  mrs_msgs
  std_msgs
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

# IROCFleetManager

## Declare a C++ library
add_library(IROCFleetManager
  src/iroc_fleet_manager.cpp
  )

add_dependencies(IROCFleetManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(IROCFleetManager
  ${catkin_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
