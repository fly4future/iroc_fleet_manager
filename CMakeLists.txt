cmake_minimum_required(VERSION 3.15.0)
project(iroc_fleet_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-O0)

set(ENERGY_AWARE_MCPP_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/EnergyAwareMCPP")

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
  IROCFleetManager_WaypointPlanner
  IROCFleetManager_CoveragePlanner
  IROCFleetManager_AutonomyTestPlanner
  EnergyAwareMCPP
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
)

#actions
add_action_files(DIRECTORY action FILES
  IROCFleetManager.action
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
  GetWorldOriginSrv.srv
  GetSafetyBorderSrv.srv
  GetObstaclesSrv.srv
  GetMissionPointsSrv.srv
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
  ${ENERGY_AWARE_MCPP_SRC_DIR}/include
)

# IROCFleetManager

## Declare a C++ library
add_library(IROCFleetManager
  src/fleet_manager.cpp
  src/utils/json_var_parser.cpp
  )

add_dependencies(IROCFleetManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(IROCFleetManager
  # IROCFleetManager
  ${catkin_LIBRARIES}
  )

# IROCFleetManager_WaypointPlanner

## Declare a C++ library
add_library(IROCFleetManager_WaypointPlanner
  src/planner_plugins/waypoint_planner.cpp
)

add_dependencies(IROCFleetManager_WaypointPlanner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(IROCFleetManager_WaypointPlanner
  ${catkin_LIBRARIES}
)

## External library for coverage planner
add_library(EnergyAwareMCPP
  ${ENERGY_AWARE_MCPP_SRC_DIR}/src/coverage_planner.cpp
  ${ENERGY_AWARE_MCPP_SRC_DIR}/src/MapPolygon.cpp
  ${ENERGY_AWARE_MCPP_SRC_DIR}/src/EnergyCalculator.cpp
  ${ENERGY_AWARE_MCPP_SRC_DIR}/src/ShortestPathCalculator.cpp
  ${ENERGY_AWARE_MCPP_SRC_DIR}/src/algorithms.cpp
  ${ENERGY_AWARE_MCPP_SRC_DIR}/src/utils.cpp
  ${ENERGY_AWARE_MCPP_SRC_DIR}/src/mstsp_solver/MstspSolver.cpp
  ${ENERGY_AWARE_MCPP_SRC_DIR}/src/mstsp_solver/TargetSet.cpp
)

# find_package(yaml-cpp REQUIRED)

target_include_directories(EnergyAwareMCPP
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${ENERGY_AWARE_MCPP_SRC_DIR}/include/EnergyAwareMCPP
)

# IROCFleetManager_CoveragePlanner
## Declare a C++ library
add_library(IROCFleetManager_CoveragePlanner
  src/planner_plugins/coverage_planner.cpp
)

add_dependencies(IROCFleetManager_CoveragePlanner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(IROCFleetManager_CoveragePlanner
  EnergyAwareMCPP
  ${catkin_LIBRARIES}
)

# IROCFleetManager_AutonomyTestPlanner
## Declare a C++ library
add_library(IROCFleetManager_AutonomyTestPlanner
  src/planner_plugins/autonomy_test_planner.cpp
)

add_dependencies(IROCFleetManager_AutonomyTestPlanner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(IROCFleetManager_AutonomyTestPlanner
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
