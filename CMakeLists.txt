cmake_minimum_required(VERSION 3.15.0)
project(iroc_fleet_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-O0)

set(COVERAGE_PLANNER_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/CoveragePlannerLib")

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  nodelet
  sensor_msgs
  mrs_msgs
  mrs_robot_diagnostics
  mrs_lib
  iroc_mission_handler
  actionlib
  actionlib_msgs
  geometry_msgs
  genmsg
)

set(LIBRARIES
  IROCFleetManager
  IROC_CoverageManager
  IROCAutonomyTestManager
  CoveragePlannerLib
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
)

#actions
add_action_files(DIRECTORY action FILES
  WaypointFleetManager.action
  AutonomyTest.action
  CoverageMission.action
)
# messages
add_message_files(DIRECTORY msg FILES
  AutonomyTestRobot.msg
  CoverageMission.msg
  CoverageMissionRobot.msg
  WaypointMissionInfo.msg
)
# messages
add_service_files(DIRECTORY srv FILES
  ChangeRobotMissionStateSrv.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  iroc_mission_handler
  mrs_msgs
  std_msgs
  iroc_mission_handler
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${COVERAGE_PLANNER_SRC_DIR}/include
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

## External library for coverage planner
add_library(CoveragePlannerLib
  ${COVERAGE_PLANNER_SRC_DIR}/src/coverage_planner.cpp
  ${COVERAGE_PLANNER_SRC_DIR}/src/MapPolygon.cpp
  ${COVERAGE_PLANNER_SRC_DIR}/src/EnergyCalculator.cpp
  ${COVERAGE_PLANNER_SRC_DIR}/src/ShortestPathCalculator.cpp
  ${COVERAGE_PLANNER_SRC_DIR}/src/algorithms.cpp
  ${COVERAGE_PLANNER_SRC_DIR}/src/utils.cpp
  ${COVERAGE_PLANNER_SRC_DIR}/src/mstsp_solver/MstspSolver.cpp
  ${COVERAGE_PLANNER_SRC_DIR}/src/mstsp_solver/TargetSet.cpp
)

# find_package(yaml-cpp REQUIRED)

target_include_directories(CoveragePlannerLib
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${COVERAGE_PLANNER_SRC_DIR}/include/CoveragePlannerLib
)

# IROC_CoverageManager
## Declare a C++ library
add_library(IROC_CoverageManager
  src/iroc_coverage_planner.cpp
)

add_dependencies(IROC_CoverageManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(IROC_CoverageManager
  CoveragePlannerLib
  ${catkin_LIBRARIES}
)

# IROCAutonomyTestManager
## Declare a C++ library
add_library(IROCAutonomyTestManager
  src/iroc_autonomy_test.cpp
)

add_dependencies(IROCAutonomyTestManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(IROCAutonomyTestManager
  ${catkin_LIBRARIES}
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

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
