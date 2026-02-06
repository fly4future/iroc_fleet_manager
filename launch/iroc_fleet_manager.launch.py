#!/usr/bin/env python3

import os
import yaml
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import (
    EnvironmentVariable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def expand_to_pwd_if_relative(arg_name: str):
    """
    ""          -> ""
    "/abs/..."  -> "/abs/..."
    "rel/..."   -> "$PWD/rel/..."
    """
    cfg = LaunchConfiguration(arg_name)
    return IfElseSubstitution(
        condition=PythonExpression([
            '"', cfg, '" != "" and not "', cfg, '".startswith("/")'
        ]),
        if_value=PathJoinSubstitution([EnvironmentVariable("PWD"), cfg]),
        else_value=cfg,
    )


def load_robot_names(network_config_path: str) -> list[str]:
    if not network_config_path:
        return []
    with open(network_config_path, "r") as f:
        cfg = yaml.safe_load(f) or {}
    return (cfg.get("network", {}) or {}).get("robot_names", []) or []


def launch_setup(context, *args, **kwargs):

    pkg_share = get_package_share_directory("iroc_fleet_manager")
    # Resolve runtime values
    ground_station = LaunchConfiguration("ground_station").perform(context)

    # These were expanded via SetLaunchConfiguration before OpaqueFunction
    network_config = LaunchConfiguration("network_config").perform(context)
    custom_config = LaunchConfiguration("custom_config").perform(context)

    use_sim_time = LaunchConfiguration("use_sim_time").perform(
        context).lower() in ("true", "1", "yes")

    robot_names = load_robot_names(network_config)

    static_remappings = [
        ("~/fleet_manager_feedback_in", f"/{ground_station}/iroc_fleet_manager/feedback"),
        ("~/mission_action_client_in", "iroc_fleet_manager"),
        ("~/change_fleet_mission_state_svc_in", "iroc_fleet_manager/change_fleet_mission_state"),
        ("~/change_robot_mission_state_svc_in", "iroc_fleet_manager/change_robot_mission_state"),
        ("~/get_world_origin_svc_in", "iroc_fleet_manager/get_world_origin"),
        ("~/get_safety_border_svc_in", "iroc_fleet_manager/get_safety_border"),
        ("~/get_obstacles_svc_in", "iroc_fleet_manager/get_obstacles"),
        ("~/get_mission_data_svc_in", "iroc_fleet_manager/get_mission_data"),
    ]

    default_config = os.path.join(pkg_share, "config", "config.yaml")

    dynamic_remappings = []
    for robot_name in robot_names:
        dynamic_remappings.extend([
            # Topics (subs)
            (f"/{robot_name}/general_robot_info_in",
             f"/{robot_name}/state_monitor/general_robot_info"),
            (f"/{robot_name}/state_estimation_info_in",
             f"/{robot_name}/state_monitor/state_estimation_info"),
            (f"/{robot_name}/control_info_in", f"/{robot_name}/state_monitor/control_info"),
            (f"/{robot_name}/collision_avoidance_info_in",
             f"/{robot_name}/state_monitor/collision_avoidance_info"),
            (f"/{robot_name}/uav_info_in", f"/{robot_name}/state_monitor/uav_info"),
            (f"/{robot_name}/system_health_info_in",
             f"/{robot_name}/state_monitor/system_health_info"),
            (f"/{robot_name}/safety_area_info_in",
             f"/{robot_name}/safety_area_manager/diagnostics"),

            (f"/{robot_name}/mission_activation_svc_in",
             f"/{robot_name}/mission_handler/mission_activation"),
            (f"/{robot_name}/mission_pausing_svc_in",
             f"/{robot_name}/mission_handler/mission_pausing"),
            # Action
            (f"/{robot_name}/action_client_mission_in",
             f"/{robot_name}/mission_handler"),
        ])

    # Container + component
    container = ComposableNodeContainer(
        name="iroc_fleet_manager_container",
        namespace=ground_station,
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        # arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],
        composable_node_descriptions=[
            ComposableNode(
                package="iroc_fleet_manager",
                plugin="iroc_fleet_manager::IROCFleetManager",
                name="iroc_fleet_manager",
                namespace=ground_station,
                parameters=[
                    {"config": default_config},
                    {"custom_config": custom_config},
                    {"network_config": network_config},
                    {"use_sim_time": use_sim_time},
                ],
                remappings=static_remappings + dynamic_remappings,
            )
        ],
    )

    return [container]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "ground_station",
            default_value=os.getenv("GS_NS", "gs"),
            description="Namespace of the ground station.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=os.getenv("USE_SIM_TIME", "false"),
            description="Use simulation time.",
        ),
        DeclareLaunchArgument(
            "custom_config",
            default_value="",
            description="Custom config path (abs or relative to PWD).",
        ),
        DeclareLaunchArgument(
            "network_config",
            default_value="",
            description="Network config YAML path (abs or relative to PWD).",
        ),

        # Expand relative paths before launch_setup runs
        SetLaunchConfiguration("custom_config", expand_to_pwd_if_relative("custom_config")),
        SetLaunchConfiguration("network_config", expand_to_pwd_if_relative("network_config")),

        OpaqueFunction(function=launch_setup),
    ])
