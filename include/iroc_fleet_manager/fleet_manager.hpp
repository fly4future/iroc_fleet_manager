#pragma once

/**
 * \file fleet_manager.hpp
 * \brief Fleet-level mission coordinator with pluggable planners and a state machine.
 *
 * IROCFleetManager is a ROS 2 composable component that orchestrates multi-robot
 * missions. It receives fleet-level goals (via an action server), decomposes them
 * into per-robot goals using pluginlib-loaded planners, dispatches those goals to
 * each robot's MissionHandler (via action clients), and monitors execution until
 * all robots complete or a failure triggers an abort.
 *
 * State machine: IDLE -> STAGED -> EXECUTING <-> PAUSED -> IDLE
 *
 * Key design decisions:
 * - Mission upload is all-or-nothing with rollback on partial failure.
 * - Feedback is aggregated from per-robot reports (progress = average across fleet).
 * - Two execution paths: slow-path (goal -> plan -> send) and fast-path (pre-staged
 *   missions with auto-activation on goal acceptance).
 */

/* ROS */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <pluginlib/class_loader.hpp>

#include <std_srvs/srv/trigger.hpp>

/* Fleet manager interfaces */
#include <iroc_fleet_manager/srv/change_fleet_mission_state_srv.hpp>
#include <iroc_fleet_manager/srv/change_robot_mission_state_srv.hpp>
#include <iroc_fleet_manager/srv/get_mission_points_srv.hpp>
#include <iroc_fleet_manager/srv/get_obstacles_srv.hpp>
#include <iroc_fleet_manager/srv/get_safety_border_srv.hpp>
#include <iroc_fleet_manager/srv/get_world_origin_srv.hpp>
#include <iroc_fleet_manager/srv/upload_fleet_mission_srv.hpp>
#include <iroc_mission_handler/srv/upload_mission_srv.hpp>
#include <iroc_mission_handler/srv/unload_mission_srv.hpp>

#include <iroc_fleet_manager/planner.h>
#include <iroc_fleet_manager/action/execute_mission.hpp>
#include <iroc_fleet_manager/msg/mission_goal.hpp>
#include <iroc_mission_handler/action/mission.hpp>

#include <iroc_common/result.h>

/* Robot diagnostics */
#include <mrs_msgs/msg/collision_avoidance_info.hpp>
#include <mrs_msgs/msg/control_info.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/safety_area_manager_diagnostics.hpp>
#include <mrs_msgs/msg/state_estimation_info.hpp>
#include <mrs_msgs/msg/system_health_info.hpp>
#include <mrs_msgs/msg/uav_info.hpp>

/* MRS Lib */
#include <mrs_lib/node.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/subscriber_handler.h>

/* IROC */
#include <iroc_fleet_manager/common_handlers.h>
#include <iroc_fleet_manager/enums/fleet_mission_state.h>

/* STL */
#include <map>
#include <mutex>
#include <vector>

namespace iroc_fleet_manager
{

// Use shared result_t from iroc_common
using result_t = iroc_common::result_t;

/**
 * \brief Configuration parameters for a dynamically-loaded planner plugin.
 *
 * - `address`: The pluginlib class address used to instantiate the planner.
 * - `name_space`: The ROS parameter namespace under which the planner's config lives.
 */
class PlannerParams {
public:
  PlannerParams(const std::string &address, const std::string &name_space);

  std::string address;
  std::string name_space;
};

using Mission           = iroc_fleet_manager::action::ExecuteMission;
using RobotMission      = iroc_mission_handler::action::Mission;
using GoalHandleMission = rclcpp_action::ServerGoalHandle<Mission>;
using MissionClient     = rclcpp_action::Client<iroc_mission_handler::action::Mission>;
using MissionGoalHandle = rclcpp_action::ClientGoalHandle<iroc_mission_handler::action::Mission>;

/**
 * \brief Fleet-level mission coordinator (ROS 2 composable component).
 *
 * Manages the lifecycle of multi-robot missions: planning, upload, execution
 * monitoring, feedback aggregation, pause/resume, and cancellation. Communicates
 * with per-robot MissionHandler nodes via action clients and service clients.
 */
class IROCFleetManager : public mrs_lib::Node {
public:
  IROCFleetManager(rclcpp::NodeOptions options);

private:
  bool is_initialized_ = false;

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;   ///< Callback group for subscribers.
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;      ///< Callback group for service servers.
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;      ///< Callback group for service clients.
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;  ///< Callback group for timers.
  rclcpp::CallbackGroup::SharedPtr cbkgrp_action_;  ///< Callback group for action server/clients.

  // | ----------------------- Timers ----------------------- |

  std::shared_ptr<TimerType> timer_main_;
  /** \brief Monitors fleet mission execution; checks for robot failures and handles completion. */
  void                       timerMain();

  std::shared_ptr<TimerType> timer_update_common_handlers_;
  /** \brief Updates shared CommonHandlers_t with latest sensor data from all robot subscribers. */
  void                       timerUpdateCommonHandlers();

  std::shared_ptr<TimerType> timer_feedback_;
  /** \brief Broadcasts aggregated fleet feedback to action server clients (active in EXECUTING/PAUSED). */
  void                       timerFeedback();

  /** \brief Loads configuration, creates subscribers, service servers, timers, action server, and planners. */
  void initialize(void);
  /** \brief Graceful shutdown handler. */
  void shutdown();

  // | ----------------------- Action server ----------------------- |

  rclcpp_action::Server<Mission>::SharedPtr action_server_ptr_;   ///< ExecuteMission action server.
  std::shared_ptr<GoalHandleMission>        current_goal_handle_; ///< Currently active fleet mission goal.
  std::recursive_mutex                      action_server_mutex_;

  // | ----------------------- ROS service servers ---------------------- |

  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::ChangeFleetMissionStateSrv> ss_change_fleet_mission_state_;
  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv> ss_change_robot_mission_state_;

  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::GetSafetyBorderSrv> ss_get_safety_border_;
  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::GetWorldOriginSrv>  ss_get_world_origin_;
  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::GetObstaclesSrv>    ss_get_obstacles_;

  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::GetMissionPointsSrv>   ss_get_mission_data_;
  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::UploadFleetMissionSrv> ss_upload_fleet_mission_;

  // | ----------------------- Staged mission state ---------------------- |

  std::mutex                                          staged_mission_mtx_;     ///< Guards staged mission fields below.
  std::vector<iroc_mission_handler::msg::MissionGoal> staged_mission_robots_;  ///< Per-robot goals for the staged mission.
  std::string                                         staged_mission_uuid_;    ///< UUID of the staged mission.

  std::atomic<fleet_mission_state_t> fleet_state_{fleet_mission_state_t::IDLE}; ///< Current fleet state machine state.

  // | ----------------------- Planner plugin system ---------------------- |

  /**
   * \brief Runtime wrapper for a single planner plugin instance.
   *
   * - `name`: Human-readable planner name (e.g., "WaypointPlanner").
   * - `params`: Plugin address and parameter namespace.
   * - `instance`: The loaded planner plugin.
   * - `mutex_planner_list_`: Per-planner mutex for thread-safe access.
   */
  struct planner_t
  {
    std::string                                            name;
    PlannerParams                                          params;
    std::shared_ptr<iroc_fleet_manager::planners::Planner> instance;
    std::mutex                                             mutex_planner_list_;
  };

  /**
   * \brief Container for all loaded planner plugin instances.
   */
  struct planners_handler_t
  {
    std::vector<planner_t> planners;
  } planner_handlers_;

  // | ----------------------- Robot diagnostic subscribers ---------------------- |

  /**
   * \brief Per-robot subscriber bundle for all diagnostic topics.
   *
   * One instance per robot; each field subscribes to the corresponding
   * MRS diagnostic topic (e.g., /<robot_name>/general_robot_info).
   */
  struct robot_diagnostics_topics_t
  {
    std::string                                                             robot_name;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::GeneralRobotInfo>             sh_general_robot_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::StateEstimationInfo>          sh_state_estimation_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlInfo>                  sh_control_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::CollisionAvoidanceInfo>       sh_collision_avoidance_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::UavInfo>                      sh_uav_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::SystemHealthInfo>             sh_system_health_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::SafetyAreaManagerDiagnostics> sh_safety_area_info;
  };

  /**
   * \brief Thread-safe container for all robot diagnostic subscriber handlers.
   */
  struct robot_topic_handlers_t
  {
    std::recursive_mutex                    mtx;      ///< Guards access to the handlers vector.
    std::vector<robot_diagnostics_topics_t> handlers;
  } robot_handlers_;

  CommonRobotHandlers_t common_robot_handlers_; ///< Latest cached robot data, shared with planners.

  std::unique_ptr<pluginlib::ClassLoader<iroc_fleet_manager::planners::Planner>> planner_loader_;
  std::vector<std::string>                                                       _planner_names_;
  std::map<std::string, PlannerParams>                                           planners_;
  std::vector<std::shared_ptr<iroc_fleet_manager::planners::Planner>>            planner_list_;
  std::mutex                                                                     mutex_planner_list_;

  int _initial_planner_idx_ = 0;
  int active_planner_idx_;

  // | ----------------------- Per-robot mission action clients ---------------------- |

  /**
   * \brief Per-robot action client state for interaction with MissionHandler.
   *
   * Tracks the action client, goal handle, service clients for activation/pausing/upload/unload,
   * and caches the latest feedback and result from each robot.
   *
   * - `got_result`: Set to true when the robot's action server returns a final result.
   * - `is_upload_staged`: True if the robot has a staged (uploaded but not yet executing) mission.
   * - `auto_activate`: When true, the robot is activated via service call as soon as
   *   the action goal is accepted (used for the fast-path staged mission flow).
   */
  struct robot_mission_handler_t
  {
    std::string                                                                robot_name;
    std::shared_ptr<MissionClient>                                             action_client_ptr;
    MissionGoalHandle::SharedPtr                                               current_goal_handle;
    mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                      sc_robot_activation;
    mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                      sc_robot_pausing;
    mrs_lib::ServiceClientHandler<iroc_mission_handler::srv::UploadMissionSrv> sc_upload_mission;
    mrs_lib::ServiceClientHandler<iroc_mission_handler::srv::UnloadMissionSrv> sc_unload_mission;
    iroc_mission_handler::action::Mission::Feedback                            current_feedback;
    iroc_mission_handler::action::Mission::Result                              current_result;
    bool                                                                       got_result       = false;
    bool                                                                       is_upload_staged = false;
    bool auto_activate = false;
  };

  /**
   * \brief Thread-safe container for all per-robot mission action client handlers.
   */
  struct fleet_mission_handlers_t
  {
    std::recursive_mutex                 mtx;      ///< Guards access to the handlers vector.
    std::vector<robot_mission_handler_t> handlers;
  } fleet_mission_handlers_;

  std::vector<std::string>             lost_robot_names_;      ///< Robots that dropped out during execution.
  iroc_fleet_manager::msg::MissionGoal current_mission_goal_;  ///< The currently active or last processed mission goal.
  std::mutex                           mission_goals_mtx_;     ///< Guards current_mission_goal_.

  // | ----------------------- Action client callbacks ---------------------- |

  /** \brief Called when a robot's action server begins processing the goal. Logs only. */
  void missionActiveCallback(const std::string &robot_name) const;

  /** \brief Called when a robot returns a final result (succeeded/aborted/canceled). Caches result in handler. */
  void missionDoneCallback(const rclcpp_action::ClientGoalHandle<RobotMission>::WrappedResult &result);

  /** \brief Called on per-robot feedback updates. Caches feedback in the corresponding handler. */
  void missionFeedbackCallback(const RobotMission::Feedback::ConstSharedPtr feedback);

  /** \brief Collects per-robot feedback, aggregates it, and publishes fleet-level feedback to action clients. */
  void actionPublishFeedback(void);

  // | ----------------------- Action server callbacks ---------------------- |

  /**
   * \brief Validates an incoming fleet mission goal.
   * Accepts if initialized, no other mission is active, and goal has details or a pre-staged mission exists.
   */
  rclcpp_action::GoalResponse   handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Mission::Goal> goal);

  /**
   * \brief Executes the accepted fleet goal.
   * Fast-path: if STAGED, sends pre-staged goals with auto_activate. Slow-path: plans via processGoal(), then sends.
   */
  void                          handle_accepted(const std::shared_ptr<GoalHandleMission> goal_handle);

  /** \brief Cancels the active fleet mission, aborting all robot action clients and transitioning to IDLE. */
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMission> goal_handle);

  // | ----------------------- Service callbacks ---------------------- |

  /**
   * \brief Handles fleet-wide state changes (start/pause/stop).
   * Calls each robot's activation or pausing service and aggregates results.
   */
  bool changeFleetMissionStateCallback(const std::shared_ptr<iroc_fleet_manager::srv::ChangeFleetMissionStateSrv::Request>  &request,
                                       const std::shared_ptr<iroc_fleet_manager::srv::ChangeFleetMissionStateSrv::Response> &response);

  /**
   * \brief Handles per-robot state changes (start/pause/stop) targeting a single robot by name.
   */
  bool changeRobotMissionStateCallback(const std::shared_ptr<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv::Request>  &request,
                                       const std::shared_ptr<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv::Response> &response);

  /**
   * \brief Returns the world origin (lat/lon) after verifying consistency across all robots.
   */
  bool getWorldOriginCallback(const std::shared_ptr<iroc_fleet_manager::srv::GetWorldOriginSrv::Request>  &request,
                              const std::shared_ptr<iroc_fleet_manager::srv::GetWorldOriginSrv::Response> &response);

  /**
   * \brief Returns the shared safety border prism after verifying all robots have identical borders.
   */
  bool getSafetyBorderCallback(const std::shared_ptr<iroc_fleet_manager::srv::GetSafetyBorderSrv::Request>  &request,
                               const std::shared_ptr<iroc_fleet_manager::srv::GetSafetyBorderSrv::Response> &response);

  /**
   * \brief Returns the shared obstacle list after verifying consistency across the fleet.
   */
  bool getObstaclesCallback(const std::shared_ptr<iroc_fleet_manager::srv::GetObstaclesSrv::Request>  &request,
                            const std::shared_ptr<iroc_fleet_manager::srv::GetObstaclesSrv::Response> &response);

  /**
   * \brief Returns the current mission goal data. Only valid in EXECUTING or PAUSED states.
   */
  bool getMissionData(const std::shared_ptr<iroc_fleet_manager::srv::GetMissionPointsSrv::Request>  &request,
                      const std::shared_ptr<iroc_fleet_manager::srv::GetMissionPointsSrv::Response> &response);

  // | ----------------------- Helper methods ---------------------- |

  /** \brief Updates fleet_state_ and logs the state transition. */
  void updateFleetState(fleet_mission_state_t new_state);

  /**
   * \brief Creates action clients for each robot and sends per-robot mission goals.
   * \param robots Per-robot mission goals to dispatch.
   * \param auto_activate If true, each robot is activated via service call on goal acceptance (fast-path).
   * \return Map of robot_name -> result_t indicating per-robot send success/failure.
   */
  std::map<std::string, result_t> sendRobotGoals(const std::vector<iroc_mission_handler::msg::MissionGoal> &robots, bool auto_activate = false);

  /**
   * \brief Finds a robot mission handler by name. Caller must hold fleet_mission_handlers_.mtx.
   * \return Pointer to the handler, or nullptr if not found.
   */
  robot_mission_handler_t *findRobotHandler(const std::string &robot_name, fleet_mission_handlers_t &mission_handlers) const;

  /**
   * \brief Aggregates per-robot feedback into a single fleet-level feedback message.
   * Progress is computed as the average across all robots.
   */
  std::shared_ptr<Mission::Feedback> processAggregatedFeedbackInfo(const std::vector<iroc_mission_handler::msg::MissionFeedback> &robot_feedbacks) const;

  /**
   * \brief Determines the fleet feedback message and state by consensus across robot feedback.
   * \return Tuple of (message_string, state_enum_string).
   */
  std::tuple<std::string, std::string> processFeedbackMsg() const;

  /**
   * \brief Sends async cancel requests to all robots that haven't returned a result yet.
   * Thread-safe: acquires fleet_mission_handlers_.mtx.
   */
  void cancelRobotClients();

  /** \brief Collects final results from all robot handlers. */
  std::vector<iroc_mission_handler::msg::MissionResult> getRobotResults();

  /**
   * \brief Extracts goal fields from the action message and delegates to processGoalFromRequest().
   */
  std::tuple<result_t, std::vector<iroc_mission_handler::msg::MissionGoal>> processGoal(const std::shared_ptr<GoalHandleMission> goal_handle);

  /**
   * \brief Validates planner name, activates the planner, and calls createGoal() to produce per-robot goals.
   * \return Tuple of (result, vector of per-robot MissionGoals).
   */
  std::tuple<result_t, std::vector<iroc_mission_handler::msg::MissionGoal>> processGoalFromRequest(const std::string &type, const std::string &details,
                                                                                                   const std::string &uuid);

  /**
   * \brief All-or-nothing fleet mission upload: validates with planner, uploads to all robots, rolls back on failure.
   * On success, stores the mission as staged and transitions to STAGED state.
   */
  bool uploadFleetMissionCallback(const std::shared_ptr<iroc_fleet_manager::srv::UploadFleetMissionSrv::Request>  &request,
                                  const std::shared_ptr<iroc_fleet_manager::srv::UploadFleetMissionSrv::Response> &response);

  /**
   * \brief Uploads per-robot missions synchronously via UploadMissionSrv service calls.
   * \return Map of robot_name -> result_t indicating per-robot upload success/failure.
   */
  std::map<std::string, result_t> uploadRobotMissions(const std::vector<iroc_mission_handler::msg::MissionGoal> &robots);

  /**
   * \brief Unloads missions from robots that succeeded upload, restoring pre-upload state.
   * Called when a partial upload failure requires atomicity rollback.
   */
  void rollbackUpload(const std::vector<std::string> &succeeded_robots);

  /** \brief Convenience wrapper around iroc_common::callService(). */
  template <typename ServiceType>
  result_t callService(mrs_lib::ServiceClientHandler<ServiceType> &sc, const std::shared_ptr<typename ServiceType::Request> &request);

  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers_; ///< Shared handlers passed to planners.
};

} // namespace iroc_fleet_manager
