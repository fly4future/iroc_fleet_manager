#pragma once

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

class IROCFleetManager : public mrs_lib::Node {
public:
  IROCFleetManager(rclcpp::NodeOptions options);

private:
  bool is_initialized_ = false;

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_action_;

  std::shared_ptr<TimerType> timer_main_;
  void                       timerMain();

  std::shared_ptr<TimerType> timer_update_common_handlers_;
  void                       timerUpdateCommonHandlers();

  std::shared_ptr<TimerType> timer_feedback_;
  void                       timerFeedback();

  void initialize(void);
  void shutdown();

  // Action server
  rclcpp_action::Server<Mission>::SharedPtr action_server_ptr_;
  std::shared_ptr<GoalHandleMission>        current_goal_handle_;
  std::recursive_mutex                      action_server_mutex_;

  // | ----------------------- ROS service servers ---------------------- |

  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::ChangeFleetMissionStateSrv> ss_change_fleet_mission_state_;
  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv> ss_change_robot_mission_state_;

  // Environment getters
  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::GetSafetyBorderSrv> ss_get_safety_border_;
  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::GetWorldOriginSrv>  ss_get_world_origin_;
  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::GetObstaclesSrv>    ss_get_obstacles_;

  // Mission getters
  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::GetMissionPointsSrv> ss_get_mission_data_;

  // Upload fleet mission service server
  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::UploadFleetMissionSrv> ss_upload_fleet_mission_;

  // Staged mission state (guarded by staged_mission_mtx_)
  std::mutex                                          staged_mission_mtx_;
  std::vector<iroc_mission_handler::msg::MissionGoal> staged_mission_robots_;
  std::string                                         staged_mission_uuid_;

  std::atomic<fleet_mission_state_t> fleet_state_{fleet_mission_state_t::IDLE};

  // | --------------- dynamic loading of planners -------------- |
  struct planner_t
  {
    std::string                                            name;
    PlannerParams                                          params;
    std::shared_ptr<iroc_fleet_manager::planners::Planner> instance;
    std::mutex                                             mutex_planner_list_;
  };

  struct planners_handler_t
  {
    std::vector<planner_t> planners;
  } planner_handlers_;

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

  struct robot_topic_handlers_t
  {
    std::recursive_mutex                    mtx;
    std::vector<robot_diagnostics_topics_t> handlers;
  } robot_handlers_;

  CommonRobotHandlers_t common_robot_handlers_;

  std::unique_ptr<pluginlib::ClassLoader<iroc_fleet_manager::planners::Planner>> planner_loader_; // pluginlib loader of dynamically loaded planners
  std::vector<std::string>                                                       _planner_names_; // list of planner names
  std::map<std::string, PlannerParams>                                           planners_;       // map between planner names and planner params
  std::vector<std::shared_ptr<iroc_fleet_manager::planners::Planner>>            planner_list_;   // list of planners, routines are callable from this
  std::mutex                                                                     mutex_planner_list_;

  int _initial_planner_idx_ = 0;
  int active_planner_idx_;

  // | ----------------- mission handler action client stuff ---------------- |

  // Handlers for the interaction with the robot's action clients with
  // MissionHandler
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
    bool auto_activate = false; // activate via service as soon as goal is accepted (staged path)
  };

  struct fleet_mission_handlers_t
  {
    std::recursive_mutex                 mtx;
    std::vector<robot_mission_handler_t> handlers;
  } fleet_mission_handlers_;

  std::vector<std::string>             lost_robot_names_;
  iroc_fleet_manager::msg::MissionGoal current_mission_goal_;
  std::mutex                           mission_goals_mtx_;

  // action client callbacks
  void missionActiveCallback(const std::string &robot_name) const;
  void missionDoneCallback(const rclcpp_action::ClientGoalHandle<RobotMission>::WrappedResult &result);
  void missionFeedbackCallback(const RobotMission::Feedback::ConstSharedPtr feedback);
  void actionPublishFeedback(void);

  // Action server callbacks
  rclcpp_action::GoalResponse   handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Mission::Goal> goal);
  void                          handle_accepted(const std::shared_ptr<GoalHandleMission> goal_handle);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMission> goal_handle);

  bool changeFleetMissionStateCallback(const std::shared_ptr<iroc_fleet_manager::srv::ChangeFleetMissionStateSrv::Request>  &request,
                                       const std::shared_ptr<iroc_fleet_manager::srv::ChangeFleetMissionStateSrv::Response> &response);
  bool changeRobotMissionStateCallback(const std::shared_ptr<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv::Request>  &request,
                                       const std::shared_ptr<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv::Response> &response);
  bool getWorldOriginCallback(const std::shared_ptr<iroc_fleet_manager::srv::GetWorldOriginSrv::Request>  &request,
                              const std::shared_ptr<iroc_fleet_manager::srv::GetWorldOriginSrv::Response> &response);
  bool getSafetyBorderCallback(const std::shared_ptr<iroc_fleet_manager::srv::GetSafetyBorderSrv::Request>  &request,
                               const std::shared_ptr<iroc_fleet_manager::srv::GetSafetyBorderSrv::Response> &response);
  bool getObstaclesCallback(const std::shared_ptr<iroc_fleet_manager::srv::GetObstaclesSrv::Request>  &request,
                            const std::shared_ptr<iroc_fleet_manager::srv::GetObstaclesSrv::Response> &response);
  bool getMissionData(const std::shared_ptr<iroc_fleet_manager::srv::GetMissionPointsSrv::Request>  &request,
                      const std::shared_ptr<iroc_fleet_manager::srv::GetMissionPointsSrv::Response> &response);

  // helper methods
  void                                 updateFleetState(fleet_mission_state_t new_state);
  std::map<std::string, result_t>      sendRobotGoals(const std::vector<iroc_mission_handler::msg::MissionGoal> &robots, bool auto_activate = false);
  robot_mission_handler_t             *findRobotHandler(const std::string &robot_name, fleet_mission_handlers_t &mission_handlers) const;
  std::shared_ptr<Mission::Feedback>   processAggregatedFeedbackInfo(const std::vector<iroc_mission_handler::msg::MissionFeedback> &robot_feedbacks) const;
  std::tuple<std::string, std::string> processFeedbackMsg() const;
  void                                 cancelRobotClients();
  std::vector<iroc_mission_handler::msg::MissionResult>                     getRobotResults();
  std::tuple<result_t, std::vector<iroc_mission_handler::msg::MissionGoal>> processGoal(const std::shared_ptr<GoalHandleMission> goal_handle);
  std::tuple<result_t, std::vector<iroc_mission_handler::msg::MissionGoal>> processGoalFromRequest(const std::string &type, const std::string &details,
                                                                                                   const std::string &uuid);
  bool                            uploadFleetMissionCallback(const std::shared_ptr<iroc_fleet_manager::srv::UploadFleetMissionSrv::Request>  &request,
                                                             const std::shared_ptr<iroc_fleet_manager::srv::UploadFleetMissionSrv::Response> &response);
  std::map<std::string, result_t> uploadRobotMissions(const std::vector<iroc_mission_handler::msg::MissionGoal> &robots);
  void                            rollbackUpload(const std::vector<std::string> &succeeded_robots);
  template <typename ServiceType>
  result_t callService(mrs_lib::ServiceClientHandler<ServiceType> &sc, const std::shared_ptr<typename ServiceType::Request> &request);

  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers_;
};

} // namespace iroc_fleet_manager
