cmake_minimum_required(VERSION 3.28)
project(iroc_fleet_manager)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[iroc_fleet_manager]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# Set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")
  add_definitions("-O1")
  add_definitions("-g")
endif()

## --------------------------------------------------------------
## |                    Find Dependencies                       |
## --------------------------------------------------------------

set(DEPENDENCIES
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  geometry_msgs
  builtin_interfaces
  action_msgs
  mrs_msgs
  mrs_lib
  mrs_robot_diagnostics
  iroc_mission_handler
  pluginlib
  rosidl_default_generators
  nlohmann_json
)

set(LIBRARIES
  IROCFleetManager
  IROCFleetManager_WaypointPlanner
  IROCFleetManager_CoveragePlanner
  # IROCFleetManager_AutonomyTestPlanner
  EnergyAwareMCPP
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

## --------------------------------------------------------------
## |                      Generate Interfaces                   |
## --------------------------------------------------------------

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionGoal.msg"
  "msg/AutonomyTestRobot.msg"
  "msg/CoverageMission.msg"
  "msg/CoverageMissionRobot.msg"
  "msg/WaypointMissionInfo.msg"
  "srv/ChangeRobotMissionStateSrv.srv"
  "srv/GetWorldOriginSrv.srv"
  "srv/GetSafetyBorderSrv.srv"
  "srv/GetObstaclesSrv.srv"
  "srv/GetMissionPointsSrv.srv"
  "srv/UploadFleetMissionSrv.srv"
  "action/ExecuteMission.action"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    mrs_msgs
    iroc_mission_handler
)

## --------------------------------------------------------------
## |                      Interface Library                     |
## --------------------------------------------------------------

add_library(IrocFleetManager_IrocFleetManager INTERFACE)
add_library(iroc_fleet_manager::iroc_fleet_manager ALIAS IrocFleetManager_IrocFleetManager) 

set_target_properties(IrocFleetManager_IrocFleetManager
  PROPERTIES
    EXPORT_NAME iroc_fleet_manager 
    OUTPUT_NAME iroc_fleet_manager 
)

target_sources(IrocFleetManager_IrocFleetManager
  INTERFACE FILE_SET HEADERS BASE_DIRS "include"
  FILES
    "include/iroc_fleet_manager/planner.h"
    "include/iroc_fleet_manager/common_handlers.h"
    "include/iroc_fleet_manager/utils/json_var_parser.h"
    "include/iroc_fleet_manager/utils/types.h"
    "include/iroc_fleet_manager/utils/conversions.h"
)

target_link_libraries(IrocFleetManager_IrocFleetManager
  INTERFACE
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_components::component
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    iroc_mission_handler::iroc_mission_handler
    nlohmann_json::nlohmann_json
    ${iroc_mission_handler_TARGETS}
    ${mrs_msgs_TARGETS}
)

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

## | ------------ IROCFleetManager ------------ |

add_library(IROCFleetManager SHARED
  src/fleet_manager.cpp
  src/utils/json_var_parser.cpp
)

target_link_libraries(IROCFleetManager
  PRIVATE
    iroc_fleet_manager::iroc_fleet_manager
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_components::component
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    mrs_robot_diagnostics::mrs_robot_diagnostics
    ${mrs_msgs_TARGETS}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(IROCFleetManager PRIVATE ${cpp_typesupport_target})

rclcpp_components_register_nodes(IROCFleetManager "iroc_fleet_manager::IROCFleetManager")
target_compile_definitions(IROCFleetManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## | ----------- IROCFleetManager_WaypointPlanner ------------ |

add_library(IROCFleetManager_WaypointPlanner SHARED
  src/planner_plugins/waypoint_planner.cpp
)

target_link_libraries(IROCFleetManager_WaypointPlanner
  PRIVATE
    IROCFleetManager
    iroc_fleet_manager::iroc_fleet_manager
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action           
    rclcpp_components::component
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    ${mrs_msgs_TARGETS}
)

# Link to generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(IROCFleetManager_WaypointPlanner PRIVATE ${cpp_typesupport_target})

rclcpp_components_register_nodes(IROCFleetManager_WaypointPlanner "iroc_fleet_manager::WaypointPlanner")
target_compile_definitions(IROCFleetManager_WaypointPlanner PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## | ----------- EnergyAwareMCPP ------------ |

set(ENERGY_AWARE_MCPP_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/EnergyAwareMCPP")

add_library(EnergyAwareMCPP SHARED
  ${ENERGY_AWARE_MCPP_SRC_DIR}/src/coverage_planner.cpp
  ${ENERGY_AWARE_MCPP_SRC_DIR}/src/MapPolygon.cpp
  ${ENERGY_AWARE_MCPP_SRC_DIR}/src/EnergyCalculator.cpp
  ${ENERGY_AWARE_MCPP_SRC_DIR}/src/ShortestPathCalculator.cpp
  ${ENERGY_AWARE_MCPP_SRC_DIR}/src/algorithms.cpp
  ${ENERGY_AWARE_MCPP_SRC_DIR}/src/utils.cpp
  ${ENERGY_AWARE_MCPP_SRC_DIR}/src/mstsp_solver/MstspSolver.cpp
  ${ENERGY_AWARE_MCPP_SRC_DIR}/src/mstsp_solver/TargetSet.cpp
)

target_include_directories(EnergyAwareMCPP
  PUBLIC
  $<BUILD_INTERFACE:${ENERGY_AWARE_MCPP_SRC_DIR}/include/EnergyAwareMCPP>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}/EnergyAwareMCPP>
)

ament_target_dependencies(EnergyAwareMCPP PUBLIC mrs_msgs)

# ## | ----------- IROCFleetManager_CoveragePlanner ------------ |

add_library(IROCFleetManager_CoveragePlanner SHARED
  src/planner_plugins/coverage_planner.cpp
)

target_link_libraries(IROCFleetManager_CoveragePlanner
  PRIVATE
    IROCFleetManager
    iroc_fleet_manager::iroc_fleet_manager
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_components::component
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    ${mrs_msgs_TARGETS}
  PUBLIC
    EnergyAwareMCPP
)

target_include_directories(IROCFleetManager_CoveragePlanner
  PRIVATE
    ${ENERGY_AWARE_MCPP_SRC_DIR}/include
)

# Link to generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(IROCFleetManager_CoveragePlanner PRIVATE ${cpp_typesupport_target})

rclcpp_components_register_nodes(IROCFleetManager_CoveragePlanner "iroc_fleet_manager::CoveragePlanner")
target_compile_definitions(IROCFleetManager_CoveragePlanner PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# ## | -------- IROCFleetManager_AutonomyTestPlanner ----------- |
# add_library(IROCFleetManager_AutonomyTestPlanner SHARED
#   src/planner_plugins/autonomy_test_planner.cpp
# )
#
# target_link_libraries(IROCFleetManager_AutonomyTestPlanner
#   PRIVATE
#     iroc_fleet_manager::iroc_fleet_manager
#     rclcpp::rclcpp
#     rclcpp_action::rclcpp_action
#     rclcpp_components::component
#     pluginlib::pluginlib
#     mrs_lib::mrs_lib
#     ${mrs_msgs_TARGETS}
# )
#
# # Link to generated interfaces
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(IROCFleetManager_AutonomyTestPlanner PRIVATE ${cpp_typesupport_target})
#
# rclcpp_components_register_nodes(IROCFleetManager_AutonomyTestPlanner "iroc_fleet_manager::AutonomyTestPlanner")
# target_compile_definitions(IROCFleetManager_AutonomyTestPlanner PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                       Export plugins                       |
## --------------------------------------------------------------

pluginlib_export_plugin_description_file(iroc_fleet_manager plugins.xml)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(
  TARGETS
    IrocFleetManager_IrocFleetManager
    ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  FILE_SET HEADERS DESTINATION "include/${PROJECT_NAME}"
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY ${ENERGY_AWARE_MCPP_SRC_DIR}/include/EnergyAwareMCPP
  DESTINATION include/${PROJECT_NAME}
)

## --------------------------------------------------------------
## |                           Export                           |
## --------------------------------------------------------------

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_libraries(
  ${LIBRARIES}
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
