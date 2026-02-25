# iroc_fleet_manager

ROS 2 coordination layer between `iroc_bridge` (HTTP gateway) and per-robot `iroc_mission_handler` nodes. Handles fleet-wide mission staging, distribution, monitoring, and environment validation.

---

## Table of Contents

- [Package Purpose](#package-purpose)
- [System Position](#system-position)
- [Fleet Mission State Machine](#fleet-mission-state-machine)
- [ROS Interfaces](#ros-interfaces)
  - [Action Server](#action-server)
  - [Service Servers](#service-servers)
  - [Service Clients (per robot)](#service-clients-per-robot)
  - [Action Clients (per robot)](#action-clients-per-robot)
  - [Subscriptions (per robot)](#subscriptions-per-robot)
- [Mission Upload and Execution Flow](#mission-upload-and-execution-flow)
- [Planner Plugin System](#planner-plugin-system)
- [Configuration](#configuration)

---

## Package Purpose

`iroc_fleet_manager` is responsible for:

- Validating and staging missions on all robots (synchronous, all-or-nothing)
- Distributing missions to robots via action clients
- Monitoring per-robot progress and aggregating feedback into a fleet-wide view
- Managing fleet state transitions (IDLE, STAGED, EXECUTING, PAUSED)
- Querying and validating environment data (world origin, safety borders, obstacles) across the fleet

---

## System Position

```
Web Client / UI
      │ HTTP REST / WebSocket
[iroc_bridge]
      │ ROS Services + Actions
[iroc_fleet_manager]   ← this node
      │ ROS Actions + Services (per robot)
[iroc_mission_handler] × N
      │
MRS UAV Core / Hardware
```

---

## Fleet Mission State Machine

Fleet state is tracked by a single `std::atomic<fleet_mission_state_t>`. The enum is defined in `include/iroc_fleet_manager/enums/fleet_mission_state.h`. Every transition is logged as `[FleetMissionState] OLD -> NEW`.

### States

| State | Description |
|---|---|
| `IDLE` | No mission loaded or executing |
| `STAGED` | Mission uploaded and validated on all robots, awaiting start command |
| `EXECUTING` | Mission active; `timerMain` monitors per-robot completion |
| `PAUSED` | Mission suspended; `timerMain` is gated, feedback still published |

### Transitions

```
              POST /mission (upload)
  IDLE ─────────────────────────────► STAGED
   ▲                                     │
   │                                     │ POST /mission/start (initial)
   │                                     ▼
   │                               EXECUTING ◄──────────────────┐
   │                              /         \                    │
   │    cancel / all robots done /     pause \                   │ resume
   └────────────────────────────(IDLE)      PAUSED ─────────────┘
                                               │
                              POST /mission/stop (async cancels fired,
                              timerMain detects completion → IDLE)
```

**Revert rules:**
- If ALL robots fail a pause call, the fleet state reverts from `PAUSED` to `EXECUTING`.
- If ALL robots fail a resume call, the fleet state reverts from `EXECUTING` to `PAUSED`.

---

## ROS Interfaces

### Action Server

**Name:** `iroc_fleet_manager`
**Type:** `iroc_fleet_manager/action/ExecuteMission`

#### Goal Fields

| Field | Type | Description |
|---|---|---|
| `type` | `string` | Planner name, e.g. `"WaypointPlanner"`. Empty string uses the pre-staged mission. |
| `details` | `string` | JSON mission details (planner-specific payload) |
| `uuid` | `string` | Client-side identifier for this mission |

#### Feedback

Published at `feedback_timer_rate` Hz during both `EXECUTING` and `PAUSED` states.

```
WaypointMissionInfo info
  string state          # one of: "trajectories_loaded", "mission_executing",
                        #         "mission_paused", "mission_aborted",
                        #         "mission_error", "mission_invalid"
  string message        # human-readable summary
  float64 progress      # fleet-wide average progress, 0.0–1.0
  MissionFeedback[] robot_feedbacks  # per-robot progress breakdown
```

#### Result

```
bool success
string message
MissionResult[] robot_results   # per-robot: name, success, message
```

---

### Service Servers

All service servers are advertised under the node's private namespace (`~/`).

| Topic | Type | Description |
|---|---|---|
| `~/upload_fleet_mission_svc_out` | `UploadFleetMissionSrv` | Upload and validate mission on all robots (synchronous, all-or-nothing). Transitions `IDLE` → `STAGED`. |
| `~/change_fleet_mission_state_svc_out` | `ChangeFleetMissionStateSrv` | `TYPE_START` / `TYPE_PAUSE` / `TYPE_STOP` for the whole fleet. |
| `~/change_robot_mission_state_svc_out` | `ChangeRobotMissionStateSrv` | `TYPE_START` / `TYPE_PAUSE` / `TYPE_STOP` for one robot. Does not affect fleet state. |
| `~/get_mission_data_svc_out` | `GetMissionPointsSrv` | Returns current mission waypoints. Available only in `EXECUTING` or `PAUSED`. |
| `~/get_world_origin_svc_out` | `GetWorldOriginSrv` | Validates all robots share the same world origin; returns it. |
| `~/get_safety_border_svc_out` | `GetSafetyBorderSrv` | Validates and returns safety border across fleet. |
| `~/get_obstacles_svc_out` | `GetObstaclesSrv` | Validates and returns obstacles across fleet. |

---

### Service Clients (per robot)

Clients are connected on demand during `sendRobotGoals`.

| Topic Pattern | Type | Purpose |
|---|---|---|
| `/{robot}/upload_mission_svc_in` | `UploadMissionSrv` | Stage mission on robot (trajectory generation + safety check) |
| `/{robot}/unload_mission_svc_in` | `UnloadMissionSrv` | Roll back staged mission on upload failure |
| `/{robot}/mission_activation_svc_in` | `std_srvs/Trigger` | Activate or resume robot mission |
| `/{robot}/mission_pausing_svc_in` | `std_srvs/Trigger` | Pause robot mission |

---

### Action Clients (per robot)

Connected on demand.

| Topic Pattern | Type | Purpose |
|---|---|---|
| `/{robot}/action_client_mission_in` | `iroc_mission_handler/Mission` | Send mission goal to the robot's mission handler |

---

### Subscriptions (per robot)

Persistent subscriptions created for each robot at startup.

| Topic Suffix | Type |
|---|---|
| `/{robot}/general_robot_info_in` | `mrs_msgs/GeneralRobotInfo` |
| `/{robot}/state_estimation_info_in` | `mrs_msgs/StateEstimationInfo` |
| `/{robot}/control_info_in` | `mrs_msgs/ControlInfo` |
| `/{robot}/collision_avoidance_info_in` | `mrs_msgs/CollisionAvoidanceInfo` |
| `/{robot}/uav_info_in` | `mrs_msgs/UavInfo` |
| `/{robot}/system_health_info_in` | `mrs_msgs/SystemHealthInfo` |
| `/{robot}/safety_area_info_in` | `mrs_msgs/SafetyAreaManagerDiagnostics` |

---

## Mission Upload and Execution Flow

```
POST /mission
  └─► uploadFleetMissionCallback()
        ├─ reject if EXECUTING or PAUSED (409)
        ├─ reject if already STAGED (409)
        ├─ planner->createGoal()          # validate JSON + generate per-robot MissionGoal list
        ├─ uploadRobotMissions()          # UploadMissionSrv to each robot (blocking)
        │     ├─ any failure → rollbackUpload() → return error
        │     └─ all success → commit staged_mission_robots_
        └─ fleet_state_ = STAGED

POST /mission/start  (initial)
  └─► bridge sends ExecuteMission action goal with empty type
  └─► handle_goal()    # reject if EXECUTING/PAUSED; reject if STAGED not set and type empty
  └─► handle_accepted()
        ├─ fleet_state_ == STAGED → use staged_mission_robots_ (fast path)
        ├─ sendRobotGoals(staged_robots, auto_activate=true)
        └─ fleet_state_ = EXECUTING

POST /mission/start  (resume from PAUSED)
  └─► changeFleetMissionStateCallback(TYPE_START)
        ├─ fleet_state_ = EXECUTING  (optimistic)
        ├─ sc_robot_activation per robot
        └─ if all fail → fleet_state_ = PAUSED  (revert)

POST /mission/pause
  └─► changeFleetMissionStateCallback(TYPE_PAUSE)
        ├─ fleet_state_ = PAUSED  (optimistic, gates timerMain immediately)
        ├─ sc_robot_pausing per robot
        └─ if all fail → fleet_state_ = EXECUTING  (revert)

POST /mission/stop
  └─► changeFleetMissionStateCallback(TYPE_STOP)
        └─ cancelRobotClients() (async cancel sent to each robot)
            timerMain detects robots done → fleet_state_ = IDLE
```

---

## Planner Plugin System

Planners are loaded at startup via `pluginlib`. Each planner implements the following interface:

```cpp
void initialize(node, name, namespace, common_handlers);
void activate();
std::tuple<result_t, std::vector<MissionGoal>> createGoal(type, details_json, uuid);
```

`CommonHandlers_t` gives planners read-only access to the latest diagnostics for all robots (position, health, safety area, etc.).

### Built-in Planners

| Name | Input | Output |
|---|---|---|
| `WaypointPlanner` | Per-robot waypoint list with `frame_id`, `height_id`, `terminal_action`, optional subtasks | `MissionGoal` per robot with waypoints passed through verbatim |
| `CoveragePlanner` | Robot name list + polygon search area + flight height | Energy-optimised lawnmower paths via the EnergyAwareMCPP library |

### Adding a Custom Planner

1. Implement `iroc_fleet_manager::planners::Planner`.
2. Export the class with `PLUGINLIB_EXPORT_CLASS`.
3. Register the planner name and plugin address in `config/config.yaml` under `fleet_manager.planners`.

---

## Configuration

`config/config.yaml`:

```yaml
fleet_manager:
  main_timer_rate: 100.0      # [Hz] timerMain and timerUpdateCommonHandlers polling rate
  feedback_timer_rate: 100.0  # [Hz] timerFeedback broadcast rate
  no_message_timeout: 5.0     # [s] subscriber no-message warning threshold

  planners:
    planner_names: ["WaypointPlanner", "CoveragePlanner"]

    WaypointPlanner:
      address: "iroc_fleet_manager/WaypointPlanner"
      name_space: "waypoint_planner"

    CoveragePlanner:
      address: "iroc_fleet_manager/CoveragePlanner"
      name_space: "coverage_planner"
```

| Parameter | Type | Description |
|---|---|---|
| `main_timer_rate` | `double` (Hz) | Polling rate for `timerMain` (completion detection) and `timerUpdateCommonHandlers` (diagnostics refresh) |
| `feedback_timer_rate` | `double` (Hz) | Rate at which `timerFeedback` publishes action feedback to the bridge |
| `no_message_timeout` | `double` (s) | Duration after which a subscriber with no received messages emits a warning |
| `planners.planner_names` | `string[]` | List of planner names to load at startup |
