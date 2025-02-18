cmake_minimum_required(VERSION 3.15.0)
project(iroc_mission_management)

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
  )

set(LIBRARIES
  IROCMissionManagement
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  INCLUDE_DIRS include
  ${catkin_INCLUDE_DIRS}
  include
  )

# IROCMissionManagement

## Declare a C++ library
add_library(IROCMissionManagement
  src/iroc_mission_management.cpp
  )

add_dependencies(IROCMissionManagement
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(IROCMissionManagement
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
