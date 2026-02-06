// ROS
#include <pluginlib/class_loader.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

// Fleet manager
#include <iroc_fleet_manager/srv/change_robot_mission_state_srv.hpp>
#include <iroc_fleet_manager/srv/get_mission_points_srv.hpp>
#include <iroc_fleet_manager/srv/get_obstacles_srv.hpp>
#include <iroc_fleet_manager/srv/get_safety_border_srv.hpp>
#include <iroc_fleet_manager/srv/get_world_origin_srv.hpp>
// TODO
#include <iroc_fleet_manager/planner.h>
#include <iroc_fleet_manager/action/execute_mission.hpp>
#include <iroc_fleet_manager/msg/mission_goal.hpp>

#include <iroc_mission_handler/action/mission.hpp>
#include <iroc_fleet_manager/utils/types.h>

// Robot diagnostics
#include <mrs_msgs/msg/collision_avoidance_info.hpp>
#include <mrs_msgs/msg/control_info.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/safety_area_manager_diagnostics.hpp>
#include <mrs_msgs/msg/state_estimation_info.hpp>
#include <mrs_msgs/msg/system_health_info.hpp>
#include <mrs_msgs/msg/uav_info.hpp>
#include <mrs_robot_diagnostics/enums/robot_type.h>

// MRS Lib
#include <mrs_lib/mutex.h>
#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_msgs/srv/string.hpp>

// Third party
#include <boost/smart_ptr/shared_ptr.hpp>
#include <numeric>
#include <tuple>

#include <iroc_fleet_manager/common_handlers.h>
//
#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

namespace iroc_fleet_manager
{

// Forward declaration of result struct
struct result_t
{
  bool success;
  std::string message;
};

class PlannerParams {

public:
  PlannerParams(const std::string &address, const std::string &name_space);

public:
  std::string address;
  std::string name_space;
};

PlannerParams::PlannerParams(const std::string &address, const std::string &name_space) {

  this->address    = address;
  this->name_space = name_space;
}

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

  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_action_;

  std::shared_ptr<TimerType> timer_main_;
  void timerMain();

  std::shared_ptr<TimerType> timer_update_common_handlers_;
  void timerUpdateCommonHandlers();

  std::shared_ptr<TimerType> timer_feedback_;
  void timerFeedback();

  void initialize(void);
  void shutdown();

  // Action server
  rclcpp_action::Server<Mission>::SharedPtr action_server_ptr_;
  std::shared_ptr<GoalHandleMission> current_goal_handle_;
  std::recursive_mutex action_server_mutex_;

  // | ----------------------- ROS service servers ---------------------- |

  mrs_lib::ServiceServerHandler<mrs_msgs::srv::String> ss_change_fleet_mission_state_;
  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv> ss_change_robot_mission_state_;

  // Environment getters
  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::GetSafetyBorderSrv> ss_get_safety_border_;
  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::GetWorldOriginSrv> ss_get_world_origin_;
  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::GetObstaclesSrv> ss_get_obstacles_;

  // Mission getters
  mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::GetMissionPointsSrv> ss_get_mission_data_;

  std::atomic_bool active_mission_        = false;
  std::atomic_bool active_mission_change_ = false;

  // | --------------- dynamic loading of planners -------------- |
  struct planner_t
  {
    std::string name;
    PlannerParams params;
    std::shared_ptr<iroc_fleet_manager::planners::Planner> instance;
    std::mutex mutex_planner_list_;
  };

  struct planners_handler_t
  {
    std::vector<planner_t> planners;
  } planner_handlers_;

  struct robot_diagnostics_topics_t
  {
    std::string robot_name;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::GeneralRobotInfo> sh_general_robot_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::StateEstimationInfo> sh_state_estimation_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlInfo> sh_control_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::CollisionAvoidanceInfo> sh_collision_avoidance_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::UavInfo> sh_uav_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::SystemHealthInfo> sh_system_health_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::SafetyAreaManagerDiagnostics> sh_safety_area_info;
  };

  struct robot_topic_handlers_t
  {
    std::recursive_mutex mtx;
    std::vector<robot_diagnostics_topics_t> handlers;
  } robot_handlers_;

  CommonRobotHandlers_t common_robot_handlers_;

  std::unique_ptr<pluginlib::ClassLoader<iroc_fleet_manager::planners::Planner>> planner_loader_; // pluginlib loader of dynamically loaded planners
  std::vector<std::string> _planner_names_;                                                       // list of planner names
  std::map<std::string, PlannerParams> planners_;                                                 // map between planner names and planner params
  std::vector<std::shared_ptr<iroc_fleet_manager::planners::Planner>> planner_list_;              // list of planners, routines are callable from this
  std::mutex mutex_planner_list_;

  int _initial_planner_idx_ = 0;
  int active_planner_idx_;

  // | ----------------- mission handler action client stuff ---------------- |

  // Handlers for the interaction with the robot's action clients with
  // MissionHandler
  struct robot_mission_handler_t
  {
    std::string robot_name;
    std::shared_ptr<MissionClient> action_client_ptr;
    MissionGoalHandle::SharedPtr current_goal_handle;
    mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sc_robot_activation;
    mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sc_robot_pausing;
    iroc_mission_handler::action::Mission::Feedback current_feedback;
    iroc_mission_handler::action::Mission::Result current_result;
    bool got_result = false;
  };

  struct fleet_mission_handlers_t
  {
    std::recursive_mutex mtx;
    std::vector<robot_mission_handler_t> handlers;
  } fleet_mission_handlers_;

  std::vector<std::string> lost_robot_names_;
  iroc_fleet_manager::msg::MissionGoal current_mission_goal_;
  std::mutex mission_goals_mtx_;

  // action client callbacks
  void missionActiveCallback(const std::string &robot_name) const;
  void missionDoneCallback(const rclcpp_action::ClientGoalHandle<RobotMission>::WrappedResult &result);
  void missionFeedbackCallback(const RobotMission::Feedback::ConstSharedPtr feedback);
  void actionPublishFeedback(void);

  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Mission::Goal> goal);
  void handle_accepted(const std::shared_ptr<GoalHandleMission> goal_handle);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMission> goal_handle);

  bool changeFleetMissionStateCallback(const std::shared_ptr<mrs_msgs::srv::String::Request> &request,
                                       const std::shared_ptr<mrs_msgs::srv::String::Response> &response);
  bool changeRobotMissionStateCallback(const std::shared_ptr<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv::Request> &request,
                                       const std::shared_ptr<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv::Response> &response);
  bool getWorldOriginCallback(const std::shared_ptr<iroc_fleet_manager::srv::GetWorldOriginSrv::Request> &request,
                              const std::shared_ptr<iroc_fleet_manager::srv::GetWorldOriginSrv::Response> &response);
  bool getSafetyBorderCallback(const std::shared_ptr<iroc_fleet_manager::srv::GetSafetyBorderSrv::Request> &request,
                               const std::shared_ptr<iroc_fleet_manager::srv::GetSafetyBorderSrv::Response> &response);
  bool getObstaclesCallback(const std::shared_ptr<iroc_fleet_manager::srv::GetObstaclesSrv::Request> &request,
                            const std::shared_ptr<iroc_fleet_manager::srv::GetObstaclesSrv::Response> &response);
  bool getMissionData(const std::shared_ptr<iroc_fleet_manager::srv::GetMissionPointsSrv::Request> &request,
                      const std::shared_ptr<iroc_fleet_manager::srv::GetMissionPointsSrv::Response> &response);

  // helper methods
  std::map<std::string, result_t> sendRobotGoals(const std::vector<iroc_mission_handler::msg::MissionGoal> &robots);
  robot_mission_handler_t *findRobotHandler(const std::string &robot_name, fleet_mission_handlers_t &mission_handlers) const;
  Mission::Feedback processAggregatedFeedbackInfo(const std::vector<iroc_mission_handler::msg::MissionFeedback> &robot_feedbacks) const;
  std::tuple<std::string, std::string> processFeedbackMsg() const;
  void cancelRobotClients();
  std::vector<iroc_mission_handler::msg::MissionResult> getRobotResults();
  std::tuple<result_t, std::vector<iroc_mission_handler::msg::MissionGoal>> processGoal(const std::shared_ptr<GoalHandleMission> goal_handle);
  template <typename ServiceType>
  result_t callService(mrs_lib::ServiceClientHandler<ServiceType> &sc, const std::shared_ptr<typename ServiceType::Request> &request);

  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers_;
};

IROCFleetManager::IROCFleetManager(rclcpp::NodeOptions options) : mrs_lib::Node("IROCFleetManager", options) {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_action_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  initialize();
}

void IROCFleetManager::initialize() {

  // --------------------------------------------------------------
  // |         common handler for fleet manager and planners      |
  // --------------------------------------------------------------

  common_handlers_ = std::make_shared<iroc_fleet_manager::CommonHandlers_t>();

  mrs_lib::ParamLoader param_loader(node_, "IROCFleetManager");

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);

  // Custom config loaded first to have the priority, if not given it loads from
  // the default config file
  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  std::string network_config_path;
  param_loader.loadParam("network_config", network_config_path);

  if (network_config_path != "") {
    param_loader.addYamlFile(network_config_path);
  }

  // Default config file
  param_loader.addYamlFileFromParam("config");

  const auto robot_names = param_loader.loadParam2<std::vector<std::string>>("network/robot_names");

  // param_loader.setPrefix("fleet_manager/");
  const auto main_timer_rate     = param_loader.loadParam2<double>("fleet_manager/main_timer_rate");
  const auto feedback_timer_rate = param_loader.loadParam2<double>("fleet_manager/feedback_timer_rate");
  const auto no_message_timeout  = param_loader.loadParam2<rclcpp::Duration>("fleet_manager/no_message_timeout");

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node               = node_;
  shopts.node_name          = "IROCFleetManager";
  shopts.no_message_timeout = no_message_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;

  // populate the robot handlers vector
  {
    std::scoped_lock lck(robot_handlers_.mtx);

    robot_handlers_.handlers.reserve(robot_names.size());
    for (const auto &robot_name : robot_names) {
      robot_diagnostics_topics_t robot_topics;
      robot_topics.robot_name = robot_name;

      const std::string general_robot_info_topic_name = "/" + robot_name + "/general_robot_info_in";

      robot_topics.sh_general_robot_info = mrs_lib::SubscriberHandler<mrs_msgs::msg::GeneralRobotInfo>(shopts, general_robot_info_topic_name);

      const std::string state_estimation_info_topic_name = "/" + robot_name + "/state_estimation_info_in";
      robot_topics.sh_state_estimation_info = mrs_lib::SubscriberHandler<mrs_msgs::msg::StateEstimationInfo>(shopts, state_estimation_info_topic_name);

      const std::string control_info_topic_name = "/" + robot_name + "/control_info_in";
      robot_topics.sh_control_info              = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlInfo>(shopts, control_info_topic_name);

      const std::string collision_avoidance_info_topic_name = "/" + robot_name + "/collision_avoidance_info_in";
      robot_topics.sh_collision_avoidance_info = mrs_lib::SubscriberHandler<mrs_msgs::msg::CollisionAvoidanceInfo>(shopts, collision_avoidance_info_topic_name);

      const std::string uav_info_topic_name = "/" + robot_name + "/uav_info_in";
      robot_topics.sh_uav_info              = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavInfo>(shopts, uav_info_topic_name);

      const std::string system_health_info_topic_name = "/" + robot_name + "/system_health_info_in";
      robot_topics.sh_system_health_info              = mrs_lib::SubscriberHandler<mrs_msgs::msg::SystemHealthInfo>(shopts, system_health_info_topic_name);

      const std::string safety_area_info_topic_name = "/" + robot_name + "/safety_area_info_in";
      robot_topics.sh_safety_area_info = mrs_lib::SubscriberHandler<mrs_msgs::msg::SafetyAreaManagerDiagnostics>(shopts, safety_area_info_topic_name);

      // move is necessary because copy construction of the subscribe handlers
      // is deleted due to mutexes
      robot_handlers_.handlers.emplace_back(std::move(robot_topics));
    }
  }

  // --------------------------------------------------------------
  // |                      load the plugins                      |
  // --------------------------------------------------------------

  param_loader.setPrefix("fleet_manager/planners/");
  param_loader.loadParam("planner_names", _planner_names_);

  planner_loader_ =
      std::make_unique<pluginlib::ClassLoader<iroc_fleet_manager::planners::Planner>>("iroc_fleet_manager", "iroc_fleet_manager::planners::Planner");

  // for each plugin in the list
  for (int i = 0; i < int(_planner_names_.size()); i++) {
    std::string planner_name = _planner_names_[i];

    // load the plugin parameters
    std::string address;
    std::string name_space;

    param_loader.loadParam(planner_name + "/address", address);
    param_loader.loadParam(planner_name + "/name_space", name_space);

    PlannerParams new_planner(address, name_space);
    planners_.insert(std::pair<std::string, PlannerParams>(planner_name, new_planner));

    try {
      RCLCPP_INFO(node_->get_logger(), "loading the planner '%s'", new_planner.address.c_str());
      planner_list_.push_back(planner_loader_->createSharedInstance(new_planner.address.c_str()));
    }
    catch (pluginlib::CreateClassException &ex1) {
      RCLCPP_ERROR(node_->get_logger(), "CreateClassException for the planner '%s'", new_planner.address.c_str());
      rclcpp::shutdown();
      exit(1);
    }
    catch (pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(node_->get_logger(), "PluginlibException for the planner '%s'", new_planner.address.c_str());
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex.what());
      rclcpp::shutdown();
      exit(1);
    }
  }

  RCLCPP_INFO(node_->get_logger(), "planners were loaded");
  {
    std::scoped_lock lck(robot_handlers_.mtx);

    for (int i = 0; i < int(planner_list_.size()); i++) {
      try {
        std::map<std::string, PlannerParams>::iterator it;
        it = planners_.find(_planner_names_[i]);

        RCLCPP_INFO(node_->get_logger(), "initializing the planner '%s'", it->second.address.c_str());
        planner_list_[i]->initialize(node_, _planner_names_[i], it->second.name_space, common_handlers_);
      }
      catch (std::runtime_error &ex) {
        RCLCPP_ERROR(node_->get_logger(), "Runtime error for the planner '%s'", ex.what());
      }
    }
  }

  RCLCPP_INFO(node_->get_logger(), "IROCFleetManager: --------------------");

  // | ------------------------- timers ------------------------- |
  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_timers_;

  {
    timer_main_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(main_timer_rate, clock_), [this]() { this->timerMain(); });
  }

  {
    timer_update_common_handlers_ =
        std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(main_timer_rate, clock_), [this]() { this->timerUpdateCommonHandlers(); });
  }

  {
    timer_feedback_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(feedback_timer_rate, clock_), [this]() { this->timerFeedback(); });
  }

  // | --------------------- service servers -------------------- |

  ss_change_fleet_mission_state_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::String>(
      node_, "svc/change_fleet_mission_state",
      [this](std::shared_ptr<mrs_msgs::srv::String::Request> request, std::shared_ptr<mrs_msgs::srv::String::Response> response) {
        changeFleetMissionStateCallback(request, response);
      },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_change_robot_mission_state_ = mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv>(
      node_, "svc/change_robot_mission_state",
      [this](std::shared_ptr<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv::Request> request,
             std::shared_ptr<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv::Response> response) { changeRobotMissionStateCallback(request, response); },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_get_world_origin_ = mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::GetWorldOriginSrv>(
      node_, "svc/get_world_origin",
      [this](std::shared_ptr<iroc_fleet_manager::srv::GetWorldOriginSrv::Request> request,
             std::shared_ptr<iroc_fleet_manager::srv::GetWorldOriginSrv::Response> response) { getWorldOriginCallback(request, response); },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_get_safety_border_ = mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::GetSafetyBorderSrv>(
      node_, "svc/get_safety_border",
      [this](std::shared_ptr<iroc_fleet_manager::srv::GetSafetyBorderSrv::Request> request,
             std::shared_ptr<iroc_fleet_manager::srv::GetSafetyBorderSrv::Response> response) { getSafetyBorderCallback(request, response); },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_get_obstacles_ = mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::GetObstaclesSrv>(
      node_, "svc/get_obstacles",
      [this](std::shared_ptr<iroc_fleet_manager::srv::GetObstaclesSrv::Request> request,
             std::shared_ptr<iroc_fleet_manager::srv::GetObstaclesSrv::Response> response) { getObstaclesCallback(request, response); },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_get_mission_data_ = mrs_lib::ServiceServerHandler<iroc_fleet_manager::srv::GetMissionPointsSrv>(
      node_, "svc/get_mission_data",
      [this](std::shared_ptr<iroc_fleet_manager::srv::GetMissionPointsSrv::Request> request,
             std::shared_ptr<iroc_fleet_manager::srv::GetMissionPointsSrv::Response> response) { getMissionData(request, response); },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  // // | ------------------ action server methods ----------------- |

  action_server_ptr_ = rclcpp_action::create_server<Mission>(
      node_, "iroc_fleet_manager", [this](auto uuid, auto goal) { return handle_goal(uuid, goal); },
      [this](auto goal_handle) { return handle_cancel(goal_handle); }, [this](auto goal_handle) { handle_accepted(goal_handle); },
      rcl_action_server_get_default_options(), cbkgrp_action_);

  RCLCPP_INFO(node_->get_logger(), "initialized");
  RCLCPP_INFO(node_->get_logger(), "--------------------");
  is_initialized_ = true;
}

/*!
 * Main Timer: Responsible of monitoring and validating the progress of the
 * missions for each robot within the fleet.
 *
 * Workflow:
 * 1. Provides a successful mission response if all robots finished
 * successfully.
 * 2. Aborts the mission for all robots if any robot reports a failure.
 *
 */
void IROCFleetManager::timerMain() {

  if (!is_initialized_) {
    return;
  }

  // Activate based on asynchronous service call which locks the action server
  // and fleet_mission_handler mutexes
  if (active_mission_change_ || !active_mission_) {
    return;
  }

  std::scoped_lock lock(action_server_mutex_);
  bool all_success     = false;
  bool got_all_results = false;
  bool any_failure     = false;
  {
    {
      std::scoped_lock lock(fleet_mission_handlers_.mtx);
      // Check if any missions aborted early
      any_failure = std::any_of(fleet_mission_handlers_.handlers.begin(), fleet_mission_handlers_.handlers.end(),
                                [](const auto &handler) { return handler.got_result && !handler.current_result.robot_result.success; });
    }

    if (any_failure) {
      RCLCPP_WARN(node_->get_logger(), " Early failure detected, not all robots finished "
                                       "successfully, aborting mission.");
      auto result           = std::make_shared<Mission::Result>();
      result->success       = false;
      result->message       = "Early failure detected, aborting mission.";
      result->robot_results = getRobotResults();
      current_goal_handle_->abort(result);
      active_mission_ = false;
      cancelRobotClients();
      RCLCPP_INFO(node_->get_logger(), " Mission aborted.");
      return;
    }

    {
      std::scoped_lock lock(fleet_mission_handlers_.mtx);
      got_all_results =
          std::all_of(fleet_mission_handlers_.handlers.begin(), fleet_mission_handlers_.handlers.end(), [](const auto &handler) { return handler.got_result; });

      all_success = std::all_of(fleet_mission_handlers_.handlers.begin(), fleet_mission_handlers_.handlers.end(),
                                [](const auto &handler) { return handler.current_result.robot_result.success; });
    }

    // Finish mission when we get all the robots result
    if (got_all_results) {
      if (!all_success) {
        RCLCPP_WARN(node_->get_logger(), " Not all robots finished successfully, finishing mission.");
        auto result           = std::make_shared<Mission::Result>();
        result->success       = false;
        result->message       = "Early failure detected, aborting mission.";
        result->robot_results = getRobotResults();
        current_goal_handle_->abort(result);
        active_mission_ = false;
        cancelRobotClients();
        RCLCPP_INFO(node_->get_logger(), " Mission finished.");
        return;
      }

      RCLCPP_INFO(node_->get_logger(), " All robots finished successfully, finishing mission.");
      auto result           = std::make_shared<Mission::Result>();
      result->success       = true;
      result->message       = "All robots finished successfully, mission finished";
      result->robot_results = getRobotResults();
      current_goal_handle_->succeed(result);
      active_mission_ = false;
      cancelRobotClients();
      RCLCPP_INFO(node_->get_logger(), " Mission finished.");
    }
  }
}

/*!
 * Continuously gathers the information from robots and wraps
 * their feedback into a general feedback message.
 *
 */
void IROCFleetManager::timerFeedback() {

  if (!is_initialized_) {
    return;
  }

  // Activate based on asynchronous service call which locks the action server
  // and fleet_mission_handler mutexes
  if (active_mission_change_ || !active_mission_) {
    return;
  }

  actionPublishFeedback();
}

void IROCFleetManager::timerUpdateCommonHandlers() {
  std::scoped_lock lck(robot_handlers_.mtx);

  // Updating the common handler with the latest messages
  for (auto &rh : robot_handlers_.handlers) {
    const auto &robot_name = rh.robot_name;

    if (rh.sh_general_robot_info.newMsg()) {
      const auto msg                                                   = rh.sh_general_robot_info.getMsg();
      common_robot_handlers_.robots_map[robot_name].general_robot_info = std::move(msg);
    }

    if (rh.sh_state_estimation_info.newMsg()) {
      const auto msg                                                      = rh.sh_state_estimation_info.getMsg();
      common_robot_handlers_.robots_map[robot_name].state_estimation_info = std::move(msg);
    }

    if (rh.sh_control_info.newMsg()) {
      const auto msg                                             = rh.sh_control_info.getMsg();
      common_robot_handlers_.robots_map[robot_name].control_info = std::move(msg);
    }

    if (rh.sh_collision_avoidance_info.newMsg()) {
      const auto msg                                                         = rh.sh_collision_avoidance_info.getMsg();
      common_robot_handlers_.robots_map[robot_name].collision_avoidance_info = std::move(msg);
    }

    if (rh.sh_uav_info.newMsg()) {
      const auto msg                                         = rh.sh_uav_info.getMsg();
      common_robot_handlers_.robots_map[robot_name].uav_info = std::move(msg);
    }

    if (rh.sh_system_health_info.newMsg()) {
      const auto msg                                                   = rh.sh_system_health_info.getMsg();
      common_robot_handlers_.robots_map[robot_name].system_health_info = std::move(msg);
    }

    if (rh.sh_safety_area_info.newMsg()) {
      const auto msg                                                 = rh.sh_safety_area_info.getMsg();
      common_robot_handlers_.robots_map[robot_name].safety_area_info = std::move(msg);
    }
  }
  // Updating the common handler with latest message
  common_handlers_->handlers = std::make_shared<CommonRobotHandlers_t>(common_robot_handlers_);
}

// | ----------------- service server callback ---------------- |
/*!

 * Handles the incoming request to change the
 * state of the mission: Start, Pause or Stop.
 *
 */
bool IROCFleetManager::changeFleetMissionStateCallback(const std::shared_ptr<mrs_msgs::srv::String::Request> &request,
                                                       const std::shared_ptr<mrs_msgs::srv::String::Response> &response) {
  std::stringstream ss;
  bool success           = true;
  active_mission_change_ = true;

  std::scoped_lock lock(action_server_mutex_, fleet_mission_handlers_.mtx);

  RCLCPP_INFO(node_->get_logger(), " Received a %s request for the fleet", request->value.c_str());

  if (current_goal_handle_->is_active()) {
    if (request->value == "start") {
      RCLCPP_INFO(node_->get_logger(), "Activating the mission for all robots.");
      for (auto &rh : fleet_mission_handlers_.handlers) {
        auto request    = std::make_shared<std_srvs::srv::Trigger::Request>();
        const auto resp = callService<std_srvs::srv::Trigger>(rh.sc_robot_activation, request);
        if (!resp.success) {
          success = false;
          RCLCPP_WARN(node_->get_logger(), " Call for robot %s was not successful with message: %s", rh.robot_name.c_str(), resp.message.c_str());
        }
      }
    } else if (request->value == "pause") {
      RCLCPP_INFO(node_->get_logger(), "Pausing the mission for all robots.");
      for (auto &rh : fleet_mission_handlers_.handlers) {
        auto request    = std::make_shared<std_srvs::srv::Trigger::Request>();
        const auto resp = callService<std_srvs::srv::Trigger>(rh.sc_robot_pausing, request);
        if (!resp.success) {
          success = false;
          RCLCPP_WARN(node_->get_logger(), " Call for robot %s was not successful with message: %s", rh.robot_name.c_str(), resp.message.c_str());
        }
      }
    } else if (request->value == "stop") {
      RCLCPP_INFO(node_->get_logger(), "Cancelling the mission for all robots.");
      cancelRobotClients();
    } else {
      success = false;
      ss << "Unsupported type\n";
    }

  } else {
    success = false;
    ss << "No active mission.\n";
  }

  if (success) {
    RCLCPP_INFO(node_->get_logger(), " Successfully processed the %s request.", request->value.c_str());
  } else {
    RCLCPP_WARN(node_->get_logger(), " Failure: %s", ss.str().c_str());
  };

  response->success      = success;
  response->message      = ss.str();
  active_mission_change_ = false;
  return true;
}

/*!
 * Handles the incoming request to change the
 * state of an individual robot mission: Start, Pause or Stop.
 *
 */
bool IROCFleetManager::changeRobotMissionStateCallback(const std::shared_ptr<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv::Request> &request,
                                                       const std::shared_ptr<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv::Response> &response) {

  std::stringstream ss;
  active_mission_change_ = true;

  std::scoped_lock lock(action_server_mutex_, fleet_mission_handlers_.mtx);

  RCLCPP_INFO(node_->get_logger(), " Received a %s request for %s", request->type.c_str(), request->robot_name.c_str());

  auto *rh_ptr = findRobotHandler(request->robot_name, fleet_mission_handlers_);

  if (rh_ptr == nullptr) {
    ss << "robot \"" << request->robot_name << "\" not found as a part of the mission, skipping\n";
    RCLCPP_WARN(node_->get_logger(), " Robot %s not found as a part of the mission. Skipping.", request->robot_name.c_str());
    response->message = ss.str();
    response->success = false;
    return true;
  }

  bool success = true;
  if (current_goal_handle_->is_active()) {
    if (request->type == "start") {
      RCLCPP_INFO(node_->get_logger(), " Calling mission activation for robot: %s.", request->robot_name.c_str());
      auto service_request = std::make_shared<std_srvs::srv::Trigger::Request>();
      const auto resp      = callService<std_srvs::srv::Trigger>(rh_ptr->sc_robot_activation, service_request);
      if (!resp.success) {
        success = false;
        RCLCPP_WARN(node_->get_logger(), " Call for robot %s was not successful with message: %s", request->robot_name.c_str(), resp.message.c_str());
        ss << "Call for robot \"" << request->robot_name << "\" was not successful with message: " << resp.message << "\n";
      } else {
        ss << "Call successful.\n";
      }
    } else if (request->type == "pause") {
      RCLCPP_INFO(node_->get_logger(), " Calling mission pausing for robot: %s.", request->robot_name.c_str());
      auto service_request = std::make_shared<std_srvs::srv::Trigger::Request>();
      const auto resp      = callService<std_srvs::srv::Trigger>(rh_ptr->sc_robot_pausing, service_request);
      if (!resp.success) {
        success = false;
        RCLCPP_WARN(node_->get_logger(), " Call for robot %s was not successful with message: %s", request->robot_name.c_str(), resp.message.c_str());
        ss << "Call for robot \"" << request->robot_name << "\" was not successful with message: " << resp.message << "\n";
      } else {
        ss << "Call successful.\n";
      }
    } else if (request->type == "stop") {
      RCLCPP_INFO(node_->get_logger(), " Calling mission stopping for robot: %s.", request->robot_name.c_str());
      // TODO when mission handler is updated to ROS2
      // const auto action_client_state = rh_ptr->action_client_ptr->getState();
      // if (action_client_state.isDone()) {
      //   ss << "robot \"" << rh_ptr->robot_name << "\" mission done,
      //   skipping\n"; success = false; ROS_WARN_STREAM("[IROCFleetManager]:
      //   Robot \"" << rh_ptr->robot_name << "\" mission done. Skipping.");
      // } else {
      //   ss << "Call successful.\n";
      //   ROS_INFO_STREAM("[IROCFleetManager]: Cancelling \"" <<
      //   rh_ptr->robot_name << "\" mission.");
      //   rh_ptr->action_client_ptr->cancelGoal();
      //   rh_ptr->action_client_ptr->waitForResult(ros::Duration(1.0));
      // }
    } else {
      success = false;
      ss << "Unsupported type\n";
    }

  } else {
    success = false;
    ss << "No active mission\n";
  }

  if (success) {
    RCLCPP_INFO(node_->get_logger(), " Successfully processed the %s request for %s.", request->type.c_str(), request->robot_name.c_str());
    ss << "Successfully processed the " << request->type << " request for " << request->robot_name << ".\n";
  } else {
    RCLCPP_WARN(node_->get_logger(), " Failure: %s", ss.str().c_str());
  };

  response->success      = success;
  response->message      = ss.str();
  active_mission_change_ = false;
  return true;
}

//}

bool IROCFleetManager::getWorldOriginCallback(const std::shared_ptr<iroc_fleet_manager::srv::GetWorldOriginSrv::Request> &request,
                                              const std::shared_ptr<iroc_fleet_manager::srv::GetWorldOriginSrv::Response> &response) {

  std::scoped_lock lck(robot_handlers_.mtx);
  RCLCPP_INFO(node_->get_logger(), " Processing a getWorldOriginCallback.");
  std::set<std::string> origin_hashes;

  double origin_x;
  double origin_y;
  bool first_robot = true;

  for (const auto &rh : robot_handlers_.handlers) {

    if (!rh.sh_safety_area_info.hasMsg()) {
      response->message = "No safety area info received, check if the Safety "
                          "Area Manager is running!";
      response->success = false;
      return true;
    }

    const auto msg = rh.sh_safety_area_info.peekMsg();
    std::ostringstream hash_stream;

    bool is_latlon = (msg->world_origin.units == "LATLON");
    if (is_latlon) {
      hash_stream << std::fixed << std::setprecision(7);
    } else {
      hash_stream << std::fixed << std::setprecision(4);
    }

    hash_stream << msg->world_origin.x << "|" << msg->world_origin.y << "|";
    origin_hashes.insert(hash_stream.str());

    if (first_robot) {
      origin_x    = msg->world_origin.x;
      origin_y    = msg->world_origin.y;
      first_robot = false;
    }
  }

  bool has_discrepancies = origin_hashes.size() > 1;

  if (has_discrepancies) {
    RCLCPP_WARN(node_->get_logger(), " Discrepancy with the robot origins!!");
    response->message = "Discrepancy in the origins between the fleet, please set the origin!";
    response->success = false;
    return true;
  } else {
    response->message  = "All robots in the fleet with same origin";
    response->success  = true;
    response->origin_x = origin_x;
    response->origin_y = origin_y;
    return true;
  }
}

// bool
// IROCFleetManager::getSafetyBorderCallback(iroc_fleet_manager::GetSafetyBorderSrv::Request
// &req, iroc_fleet_manager::GetSafetyBorderSrv::Response &res) {
bool IROCFleetManager::getSafetyBorderCallback(const std::shared_ptr<iroc_fleet_manager::srv::GetSafetyBorderSrv::Request> &request,
                                               const std::shared_ptr<iroc_fleet_manager::srv::GetSafetyBorderSrv::Response> &response) {
  std::scoped_lock lck(robot_handlers_.mtx);
  RCLCPP_INFO(node_->get_logger(), " Processing a getSafetyBorderCallback.");
  std::set<std::string> config_hashes;
  mrs_msgs::msg::Prism reference_border;
  bool first_robot = true;

  for (const auto &rh : robot_handlers_.handlers) {

    if (!rh.sh_safety_area_info.hasMsg()) {
      response->message = "No safety area info received, check if the Safety "
                          "Area Manager is running!";
      response->success = false;
      return true;
    }

    const auto msg = rh.sh_safety_area_info.peekMsg();
    std::ostringstream hash_stream;

    bool is_latlon = (msg->border.horizontal_frame == "latlon_origin");
    if (is_latlon) {
      hash_stream << std::fixed << std::setprecision(7);
    } else {
      hash_stream << std::fixed << std::setprecision(4);
    }

    hash_stream << msg->border.max_z << "|" << msg->border.min_z << "|" << msg->border.horizontal_frame << "|" << msg->border.vertical_frame << "|"
                << msg->safety_area_enabled << "|";

    for (const auto &point : msg->border.points) {
      hash_stream << point.x << "," << point.y << ";";
    }

    config_hashes.insert(hash_stream.str());

    if (first_robot) {
      reference_border = msg->border;
      first_robot      = false;
    }
  }

  bool has_discrepancies = config_hashes.size() > 1;

  if (has_discrepancies) {
    RCLCPP_WARN(node_->get_logger(), " Discrepancy with the robot borders!!");
    response->success = false;
    response->message = "Discrepancy in the borders between the fleet, please "
                        "set the safety borders!";
    return true;
  } else {
    response->success = true;
    response->message = "All robots in the fleet with the same safety border";
    response->border  = reference_border;
    return true;
  }
}

bool IROCFleetManager::getObstaclesCallback(const std::shared_ptr<iroc_fleet_manager::srv::GetObstaclesSrv::Request> &request,
                                            const std::shared_ptr<iroc_fleet_manager::srv::GetObstaclesSrv::Response> &response) {

  std::scoped_lock lck(robot_handlers_.mtx);
  RCLCPP_INFO(node_->get_logger(), " Processing a getObstaclesCallback.");

  std::set<std::string> config_hashes;
  std::vector<mrs_msgs::msg::Prism> reference_obstacles;
  bool first_robot = true;

  for (const auto &rh : robot_handlers_.handlers) {

    if (!rh.sh_safety_area_info.hasMsg()) {
      response->message = "No safety area info received, check if the Safety "
                          "Area Manager is running!";
      response->success = false;
      return true;
    }

    const auto msg = rh.sh_safety_area_info.peekMsg();
    std::ostringstream hash_stream;

    bool is_latlon = (msg->border.horizontal_frame == "latlon_origin");
    if (is_latlon) {
      hash_stream << std::fixed << std::setprecision(7);
    } else {
      hash_stream << std::fixed << std::setprecision(4);
    }

    // TODO update with new obstacle message structure
    //  for (const auto &point : msg->safety_area.obstacles.data)
    //    hash_stream << point.x << "," << point.y << ";";
    //
    //  for (const auto &row : msg->safety_area.obstacles.rows)
    //    hash_stream << row << "|";
    //
    //  for (const auto &max_z : msg->safety_area.obstacles.max_z)
    //    hash_stream << max_z << "|";
    //
    //  for (const auto &min_z : msg->safety_area.obstacles.min_z)
    //    hash_stream << min_z << "|";
    //
    //  config_hashes.insert(hash_stream.str());
    //
    //  if (first_robot) {
    //    reference_obstacles                  = msg->safety_area.obstacles;
    //    reference_obstacles.horizontal_frame =
    //    msg->safety_area.border.horizontal_frame; // Obstacles follow same
    //    frame as the border reference_obstacles.vertical_frame   =
    //    msg->safety_area.border.vertical_frame; first_robot = false;
    //  }
  }

  bool has_discrepancies = config_hashes.size() > 1;

  if (has_discrepancies) {

    for (const auto &hash : config_hashes) {
      RCLCPP_WARN_STREAM(node_->get_logger(), hash);
    }
    RCLCPP_WARN(node_->get_logger(), " Discrepancy with the robot obstacles!!");
    response->success = false;
    response->message = "Discrepancy in the obstacles between the fleet, "
                        "please set the obstacles!";
    return true;
  } else {
    response->success   = true;
    response->message   = "All robots in the fleet with the same safety border";
    response->obstacles = reference_obstacles;
    return true;
  }
}

bool IROCFleetManager::getMissionData(const std::shared_ptr<iroc_fleet_manager::srv::GetMissionPointsSrv::Request> &request,
                                      const std::shared_ptr<iroc_fleet_manager::srv::GetMissionPointsSrv::Response> &response) {

  std::scoped_lock lck(mission_goals_mtx_);

  if (!active_mission_) {
    response->success = false;
    response->message = "No active mission.";
    return true;
  }

  response->success      = true;
  response->message      = "Successfully got the mission goals";
  response->mission_goal = current_mission_goal_;

  return true;
}

// | ---------------------- action server callbacks --------------------- |

void IROCFleetManager::missionActiveCallback(const std::string &robot_name) const {
  RCLCPP_INFO(node_->get_logger(), " Action server on robot %s is processing the goal.", robot_name.c_str());
}

/*!
 * Handles and store the result from the action client in
 * the  struct fleet_mission_handlers_.
 *
 * Workflow:
 *
 * 1. In case any disconnection from a robot (Mission Handler node dies), the
 * connection with that robot is lost, this is saved in a vector
 * lost_robot_names_ for the proper handling.
 *
 */

void IROCFleetManager::missionDoneCallback(const rclcpp_action::ClientGoalHandle<RobotMission>::WrappedResult &result) {

  if (!active_mission_) {
    return;
  }
  const auto robot_name = result.result->robot_result.name;

  RCLCPP_INFO_STREAM(node_->get_logger(), "Action server on robot " << result.result->robot_result.name << " finished successfully.");
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO_STREAM(node_->get_logger(), "Action server on robot " << robot_name << " reported success.");
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_WARN_STREAM(node_->get_logger(), "Action server on robot " << robot_name << " was aborted");
    break;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_WARN_STREAM(node_->get_logger(), "Action server on robot " << robot_name << " was canceled");
    break;
  default:
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Action server on robot " << robot_name << " returned unknown result code");
    break;
  }

  {
    std::scoped_lock lck(fleet_mission_handlers_.mtx);
    auto *rh_ptr           = findRobotHandler(robot_name, fleet_mission_handlers_);
    rh_ptr->current_result = *result.result;
    rh_ptr->got_result     = true;
  }
}

/*!
 * Handles and store the feedback from the action
 * client in the  struct fleet_mission_handlers_.
 *
 */

void IROCFleetManager::missionFeedbackCallback(const RobotMission::Feedback::ConstSharedPtr feedback) {

  if (!active_mission_) {
    return;
  }

  auto robot_name = feedback->robot_feedback.name;

  {
    std::scoped_lock lck(fleet_mission_handlers_.mtx);
    auto *rh_ptr             = findRobotHandler(robot_name, fleet_mission_handlers_);
    rh_ptr->current_feedback = *feedback;
  }
}

/*!
 * Handles and processes goals from action clients.
 *
 * Workflow:
 * 1. Calls virtual methods defined in child classes to get goals for each robot
 * 2. Ensures all missions follow the MissionRobotGoal message structure
 *    required by Mission Handler
 *
 * @param goal The incoming goal from the action client
 */
rclcpp_action::GoalResponse IROCFleetManager::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Mission::Goal> goal) {
  RCLCPP_INFO(node_->get_logger(), "Received goal request with ID %s", rclcpp_action::to_string(uuid).c_str());

  if (!is_initialized_) {
    RCLCPP_WARN(node_->get_logger(), "Not initialized yet, rejecting goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void IROCFleetManager::handle_accepted(const std::shared_ptr<GoalHandleMission> goal_handle) {

  std::scoped_lock lock(action_server_mutex_);

  const auto goal = goal_handle->get_goal();

  const auto [result, mission_robots] = processGoal(goal_handle);

  if (!result.success) {
    auto result_ptr           = std::make_shared<Mission::Result>();
    result_ptr->success       = false;
    result_ptr->message       = result.message;
    result_ptr->robot_results = getRobotResults();
    RCLCPP_WARN(node_->get_logger(), " Goal creation process failed");
    goal_handle->abort(result_ptr);
    return;
  }

  // Start each robot action/service clients with mission_handler
  const auto results = sendRobotGoals(mission_robots);

  bool all_success = std::all_of(results.begin(), results.end(), [](const auto &pair) { return pair.second.success; });

  // TODO: check maybe not necessary anymore (or different way of doing it), as if the goal is rejected we will handle with the callbacks
  if (!all_success) {
    auto result_ptr = std::make_shared<Mission::Result>();
    iroc_mission_handler::msg::MissionResult robot_result;
    for (const auto &result : results) {
      std::stringstream ss;
      robot_result.name    = result.first;
      robot_result.message = result.second.message;
      robot_result.success = result.second.success;
      result_ptr->robot_results.emplace_back(robot_result);
      if (!result.second.success) {
        ss << result.first << " failed with response: " << result.second.message;
        RCLCPP_WARN(node_->get_logger(), " Failure starting robot clients: %s", ss.str().c_str());
      }
    }
    result_ptr->success = false;
    result_ptr->message = "Failure starting robot clients.";
    goal_handle->abort(result_ptr);
    RCLCPP_INFO(node_->get_logger(), " Mission Aborted.");
    return;
  }
  RCLCPP_INFO(node_->get_logger(), " Successfully sent the goal to robots in mission.");
  current_goal_handle_ = goal_handle;
  active_mission_      = true;
}

rclcpp_action::CancelResponse IROCFleetManager::handle_cancel(const std::shared_ptr<GoalHandleMission> goal_handle) {
  RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");

  auto result     = std::make_shared<Mission::Result>();
  result->success = false;
  result->message = "Mission cancelled by client request.";
  current_goal_handle_->abort(result);

  cancelRobotClients();
  active_mission_ = false;
  RCLCPP_INFO(node_->get_logger(), "Mission stopped by cancel request.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

/*!
 * Collects the feedback from the active robots in mission and aggregates them
 * to provide a general mission feedback.
 */
void IROCFleetManager::actionPublishFeedback() {
  std::scoped_lock lock(action_server_mutex_);

  // Collect the feedback from active robots in the mission
  std::vector<iroc_mission_handler::msg::MissionFeedback> robot_feedbacks;
  {
    std::scoped_lock lck(fleet_mission_handlers_.mtx);

    // Fill the robots feedback vector
    for (const auto &rh : fleet_mission_handlers_.handlers)
      robot_feedbacks.emplace_back(rh.current_feedback.robot_feedback);

    if (current_goal_handle_->is_active()) {
      auto action_server_feedback = processAggregatedFeedbackInfo(robot_feedbacks);
      auto feedback               = std::make_shared<Mission::Feedback>();
      feedback->info.state        = iroc_fleet_manager::msg::WaypointMissionInfo::STATE_TRAJECTORIES_LOADED;
      feedback->info.message      = "Feedback not implemented yet.";
      feedback->info.progress     = 0.0;
      current_goal_handle_->publish_feedback(feedback);
    }
  }
}

/*!
 * Obtains the results from each robot.
 *
 * Workflow
 * 1. While obtaining the results in case there is not a result yet --because an
 * ongoing mission -- the mission for that robot is aborted.
 *
 */


std::vector<iroc_mission_handler::msg::MissionResult> IROCFleetManager::getRobotResults() {
  // Get the robot results
  std::vector<iroc_mission_handler::msg::MissionResult> robot_results;
  {
    std::scoped_lock lock(fleet_mission_handlers_.mtx);
    for (auto &handler : fleet_mission_handlers_.handlers) {
      iroc_mission_handler::msg::MissionResult robot_result;
      if (handler.got_result && !handler.current_result.robot_result.success) {
        robot_result.name    = handler.robot_name;
        robot_result.message = handler.current_result.robot_result.message;
        robot_result.success = handler.current_result.robot_result.success;
      }

      if (handler.got_result) {
        robot_result.name    = handler.robot_name;
        robot_result.message = handler.current_result.robot_result.message;
        robot_result.success = handler.current_result.robot_result.success;
      } else {
        robot_result.name    = handler.robot_name;
        robot_result.message = "Robot did not finish its mission, mission was aborted.";
        robot_result.success = false;
      }
      robot_results.emplace_back(robot_result);
    }
  }
  // Print the robots result
  for (auto &robot_result : robot_results) {
    RCLCPP_INFO(node_->get_logger(), " Robot: %s, result: %s success: %d", robot_result.name.c_str(), robot_result.message.c_str(), robot_result.success);
  }

  return robot_results;
}

/*!
 * Initializes the missions for each robot through Mission Handler.
 *
 * Workflow:
 * 1. Connects to the robots server.
 * 2. Creates the action clients for each robot topics, initializes the service
 * clients for the mission features and sends the goals for each robot.
 * 3. Provides a successful response if all the requested robots for the mission
 * where successfully initialized.
 */

std::map<std::string, result_t> IROCFleetManager::sendRobotGoals(const std::vector<iroc_mission_handler::msg::MissionGoal> &robots) {
  std::scoped_lock lck(fleet_mission_handlers_.mtx);
  std::map<std::string, result_t> robot_results;

  // Clear the previous handlers
  fleet_mission_handlers_.handlers.clear();
  lost_robot_names_.clear();

  fleet_mission_handlers_.handlers.reserve(robots.size());
  // Initialize the robots received in the goal request
  for (const auto &robot : robots) {
    std::stringstream ss;
    const std::string action_client_topic = "/" + robot.name + "/action_client_mission_in";

    auto action_client_ptr = rclcpp_action::create_client<iroc_mission_handler::action::Mission>(node_, action_client_topic);

    RCLCPP_INFO(node_->get_logger(), "Creating action client on topic '%s'", action_client_topic.c_str());

    // Wait for server
    if (!action_client_ptr->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_WARN(node_->get_logger(), " Action server from robot: %s is not connected. Check the iroc_mission_handler node.", robot.name.c_str());
      robot_results[robot.name].message = ss.str();
      robot_results[robot.name].success = false;
      continue;
    }

    RCLCPP_INFO(node_->get_logger(), " Created action client on topic '%s'", action_client_topic.c_str());

    // Goal message creation
    auto mission_goal                       = RobotMission::Goal();
    mission_goal.robot_goal.frame_id        = robot.frame_id;
    mission_goal.robot_goal.height_id       = robot.height_id;
    mission_goal.robot_goal.terminal_action = robot.terminal_action;
    mission_goal.robot_goal.points          = robot.points;

    // Robot handler creation
    robot_mission_handler_t robot_handler;
    robot_handler.robot_name        = robot.name;
    robot_handler.action_client_ptr = action_client_ptr;
    robot_handler.got_result        = false;


    if (!action_client_ptr->action_server_is_ready()) {
      ss << "Action server from robot: " + robot.name + " is not ready. Check the iroc_mission_handler node.\n ";
      RCLCPP_WARN(node_->get_logger(), " Action server from robot: %s is not ready. Check the iroc_mission_handler node.", robot.name.c_str());
      robot_results[robot.name].message = ss.str();
      robot_results[robot.name].success = false;
    }

    auto send_goal_options = rclcpp_action::Client<RobotMission>::SendGoalOptions();

    // TODO: handle properly with a function callback
    send_goal_options.goal_response_callback = [this, robot_name = robot.name](MissionGoalHandle::SharedPtr goal_handle) {
      std::scoped_lock lock(fleet_mission_handlers_.mtx);
      auto *handler = findRobotHandler(robot_name, fleet_mission_handlers_);
      if (!goal_handle) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Goal was rejected by robot");
        if (handler) {
          handler->got_result                          = true;
          handler->current_result.robot_result.success = false;
          handler->current_result.robot_result.message = "Goal rejected by mission handler";
        }
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by robot '%s'", robot_name.c_str());
      // Save the goal handle
      if (handler) {
        handler->current_goal_handle = goal_handle;
      }
    };

    // Result callback
    send_goal_options.result_callback = [this](const MissionGoalHandle::WrappedResult &result) { this->missionDoneCallback(result); };

    // Feedback callback
    send_goal_options.feedback_callback = [this](MissionGoalHandle::SharedPtr, const RobotMission::Feedback::ConstSharedPtr feedback) {
      this->missionFeedbackCallback(feedback);
    };

    action_client_ptr->async_send_goal(mission_goal, send_goal_options);

    // Save the ros service clients from mission_manager
    const std::string mission_activation_client_topic = "/" + robot.name + "/mission_activation_svc_in";
    robot_handler.sc_robot_activation = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, mission_activation_client_topic, cbkgrp_sc_);

    const std::string mission_pausing_client_topic = "/" + robot.name + "/mission_pausing_svc_in";
    robot_handler.sc_robot_pausing                 = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, mission_pausing_client_topic, cbkgrp_sc_);


    // Save the handler
    fleet_mission_handlers_.handlers.emplace_back(std::move(robot_handler));
    ss << "Mission goal sent successfully to robot '" << robot.name << "'";
    robot_results[robot.name].success = true;
    robot_results[robot.name].message = ss.str();

    RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
  }

  return robot_results;
}


std::tuple<result_t, std::vector<iroc_mission_handler::msg::MissionGoal>> IROCFleetManager::processGoal(const std::shared_ptr<GoalHandleMission> goal_handle) {
  // Here we make the validation with the planners
  // First we check if the type matches the loaded planners
  // check the existance of the requested planner
  result_t result;
  bool check = false;
  int planner_idx;

  // Get the goal
  auto goal = goal_handle->get_goal();

  for (int i = 0; i < int(_planner_names_.size()); i++) {
    std::string planner_name = _planner_names_[i];
    if (planner_name == goal->type) {
      check       = true;
      planner_idx = i;
      break;
    }
  }

  if (!check) {
    RCLCPP_WARN(node_->get_logger(), " The requested planner (%s) is not within the loaded planners", goal->type.c_str());
    result.success = false;
    result.message = "Requested planner is not within the loaded planners";
    return std::make_tuple(result, std::vector<iroc_mission_handler::msg::MissionGoal>());
  }

  RCLCPP_INFO(node_->get_logger(), " Received a request for %s, activating the planner...", goal->type.c_str());

  // activate the planner
  planner_list_[planner_idx]->activate();
  active_planner_idx_ = planner_idx;

  std::scoped_lock lck(robot_handlers_.mtx, mission_goals_mtx_);

  // Validate the goal with the planner
  const auto [goal_creation_result, mission_robots] = planner_list_[planner_idx]->createGoal(goal->details);

  auto goal_type = std::make_shared<iroc_mission_handler::action::Mission::Goal>();


  // Check if the goal was created successfully
  if (!goal_creation_result.success) {
    result.success = false;
    result.message = goal_creation_result.message;
    return std::make_tuple(result, std::vector<iroc_mission_handler::msg::MissionGoal>());
  }

  result.success                    = true;
  result.message                    = "Goal created successfully";
  current_mission_goal_.type        = goal->type;
  current_mission_goal_.uuid        = goal->uuid;
  current_mission_goal_.robot_goals = mission_robots;

  RCLCPP_INFO(node_->get_logger(), " Mission goal created successfully for %zu robots.", mission_robots.size());

  return std::make_tuple(result, mission_robots);
}

/*!
 * Aggregates the feedback status message from the robots feedback.
 *
 */

Mission::Feedback IROCFleetManager::processAggregatedFeedbackInfo(const std::vector<iroc_mission_handler::msg::MissionFeedback> &robot_feedbacks) const {

  Mission::Feedback feedback;
  // FeedbackType action_server_feedback;
  std::vector<std::string> robots_msg;
  std::vector<double> robots_progress;

  if (robot_feedbacks.empty()) {
    feedback.info.progress = 0.0;
    feedback.info.message  = "No robot feedback available";
    feedback.info.state    = iroc_fleet_manager::msg::WaypointMissionInfo::STATE_INVALID;
    return feedback;
  }

  for (const auto &rbf : robot_feedbacks) {
    robots_msg.emplace_back(rbf.message);
    robots_progress.emplace_back(rbf.mission_progress);
  }

  // Get average of active robots progress for the general progress
  auto mission_progress = std::accumulate(robots_progress.begin(), robots_progress.end(), 0.0) / robots_progress.size();

  // Checks if ALL messages are exactly "MISSION_LOADED"
  auto [message, state] = processFeedbackMsg();

  feedback.info.progress        = mission_progress;
  feedback.info.message         = message;
  feedback.info.state           = state;
  feedback.info.robot_feedbacks = robot_feedbacks;

  return feedback;
}

/*!
 * Obtains the feedback from each robot to provide a status for the general
 * mission.
 *
 */

std::tuple<std::string, std::string> IROCFleetManager::processFeedbackMsg() const {

  auto all_loaded = std::all_of(fleet_mission_handlers_.handlers.begin(), fleet_mission_handlers_.handlers.end(),
                                [](const auto &handler) { return handler.current_feedback.robot_feedback.message == "MISSION_LOADED"; });

  if (all_loaded)
    return std::make_tuple("All missions loaded", iroc_fleet_manager::msg::WaypointMissionInfo::STATE_TRAJECTORIES_LOADED);

  auto all_executing = std::all_of(fleet_mission_handlers_.handlers.begin(), fleet_mission_handlers_.handlers.end(),
                                   [](const auto &handler) { return handler.current_feedback.robot_feedback.message == "EXECUTING"; });

  if (all_executing)
    return std::make_tuple("Robots executing mission", iroc_fleet_manager::msg::WaypointMissionInfo::STATE_EXECUTING);

  auto all_paused = std::all_of(fleet_mission_handlers_.handlers.begin(), fleet_mission_handlers_.handlers.end(),
                                [](const auto &handler) { return handler.current_feedback.robot_feedback.message == "PAUSED"; });

  if (all_paused)
    return std::make_tuple("All robots paused", iroc_fleet_manager::msg::WaypointMissionInfo::STATE_PAUSED);

  auto any_idle = std::any_of(fleet_mission_handlers_.handlers.begin(), fleet_mission_handlers_.handlers.end(),
                              [](const auto &handler) { return handler.current_feedback.robot_feedback.message == "IDLE"; });

  if (any_idle)
    return std::make_tuple("One robot is idle, check mission_manager", iroc_fleet_manager::msg::WaypointMissionInfo::STATE_ERROR);

  return std::make_tuple("Not defined message", iroc_fleet_manager::msg::WaypointMissionInfo::STATE_INVALID);
}

/*!
 * Helper method to obtain a robot handler
 *
 */
IROCFleetManager::robot_mission_handler_t *IROCFleetManager::findRobotHandler(const std::string &robot_name,
                                                                              fleet_mission_handlers_t &fleet_mission_handlers) const {
  for (auto &rh : fleet_mission_handlers.handlers) {
    if (rh.robot_name == robot_name)
      return &rh;
  }
  RCLCPP_WARN(node_->get_logger(), " Robot handler not found!");
  return nullptr;
}

/*!
 * Cancel the mission for all the robots in the current active mission.
 *
 * Workflow:
 * 1. Aborts the active mission for all robots.
 * 2. Cancelling is skipped if the mission robot was already done (got a result
 * already).
 *
 */
void IROCFleetManager::cancelRobotClients() {

  RCLCPP_INFO(node_->get_logger(), " Canceling robot clients...");
  std::scoped_lock lock(fleet_mission_handlers_.mtx);

  for (auto &handler : fleet_mission_handlers_.handlers) {
    if (handler.current_goal_handle && !handler.got_result) {
      RCLCPP_INFO(node_->get_logger(), "Canceling goal for robot '%s'", handler.robot_name.c_str());

      auto cancel_callback = [this, robot_name = handler.robot_name](std::shared_ptr<rclcpp_action::Client<RobotMission>::CancelResponse> response) {
        if (response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
          RCLCPP_INFO(node_->get_logger(), "Cancel accepted for robot '%s'", robot_name.c_str());
        } else {
          RCLCPP_WARN(node_->get_logger(), "Cancel rejected for robot '%s'", robot_name.c_str());
        }
      };

      // Send the cancel request
      handler.action_client_ptr->async_cancel_goal(handler.current_goal_handle, cancel_callback);
    }
  }
}

template <typename ServiceType>
result_t IROCFleetManager::callService(mrs_lib::ServiceClientHandler<ServiceType> &sc, const std::shared_ptr<typename ServiceType::Request> &request) {
  // typename ServiceType::Response res;

  auto response = sc.callSync(request);

  if (response) {
    if (response.value()->success) {
      // TODO add getService() to mrs_lib ServiceClientHandler
      RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000,
                                  "Called service  << sc.getService() <<  with response \"" << response.value()->message << "\".");
      return {true, response.value()->message};
    } else {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000,
                                  "Called service << sc.getService() <<  with response \"" << response.value()->message << "\".");
      return {false, response.value()->message};
    }
  } else {
    const std::string msg = "Failed to call service  + sc.getService() .";
    // ROS_WARN_STREAM(msg);
    return {false, msg};
  }
}

/*
template <typename Svc_T>
result_t IROCFleetManager::callService(ros::ServiceClient &sc, typename
Svc_T::Request req) const { typename Svc_T::Response res; if (sc.call(req, res))
{ if (response->success) { ROS_INFO_STREAM("Called service \"" <<
sc.getService() << "\" with response \"" << response->message << "\"."); return
{true, res.message}; } else { ROS_WARN_STREAM("Called service \"" <<
sc.getService() << "\" with response \"" << response->message << "\"."); return
{false, res.message};
    }
  } else {
    const std::string msg = "Failed to call service \"" + sc.getService() +
"\"."; ROS_WARN_STREAM(msg); return {false, msg};
  }
}

template <typename Svc_T>
result_t IROCFleetManager::callService(ros::ServiceClient &sc) const {
  return callService<Svc_T>(sc, {});
}

result_t IROCFleetManager::callService(ros::ServiceClient &sc, const bool val)
const { using svc_t = std_srvs::SetBool; svc_t::Request req; request->data =
val; return callService<svc_t>(sc, req);
}
*/

} // namespace iroc_fleet_manager
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(iroc_fleet_manager::IROCFleetManager)
