/* includes //{ */

#include <string>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

/* custom msgs of MRS group */
#include <iroc_fleet_manager/ChangeRobotMissionStateSrv.h>
#include <mrs_msgs/String.h>
#include <iroc_mission_handler/waypointMissionAction.h>
#include <iroc_fleet_manager/CoverageMissionAction.h>
#include <iroc_fleet_manager/WaypointMissionRobotFeedback.h>
#include <iroc_fleet_manager/WaypointMissionRobot.h>
#include <unistd.h>
#include <iostream>
#include <numeric>

// Coverage planner library includes
#include <CoveragePlannerLib/coverage_planner.hpp>
#include <CoveragePlannerLib/MapPolygon.hpp>
#include <CoveragePlannerLib/EnergyCalculator.h>
#include <CoveragePlannerLib/algorithms.hpp>
#include <CoveragePlannerLib/ShortestPathCalculator.hpp>
#include <CoveragePlannerLib/mstsp_solver/SolverConfig.h>
#include <CoveragePlannerLib/mstsp_solver/MstspSolver.h>
#include <CoveragePlannerLib/SimpleLogger.h>
#include <CoveragePlannerLib/utils.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

//}

namespace iroc_fleet_manager
{

using namespace actionlib;

typedef SimpleActionClient<iroc_mission_handler::waypointMissionAction> MissionManagerClient;
typedef iroc_mission_handler::waypointMissionGoal                       MissionManagerActionServerGoal;

/* class IROC_CoverageManager //{ */

class IROC_CoverageManager: public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool is_initialized_                      = false;
  std::atomic_bool active_mission_          = false;
  std::atomic_bool active_mission_change_   = false;
  struct result_t
  {
    bool        success;
    std::string message;
  };

  // | ----------------------- ROS servers ---------------------- |
  ros::ServiceServer ss_change_fleet_mission_state_;
  ros::ServiceServer ss_change_robot_mission_state_;
  bool changeFleetMissionStateCallback(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);
  bool changeRobotMissionStateCallback(iroc_fleet_manager::ChangeRobotMissionStateSrv::Request& req, iroc_fleet_manager::ChangeRobotMissionStateSrv::Response& res);

  // | ----------------------- main timer ----------------------- |
  ros::Timer timer_main_;
  ros::Timer timer_feedback_;
  void       timerMain(const ros::TimerEvent& event);
  void       timerFeedback(const ros::TimerEvent& event);

  // | ----------------- mission management action server stuff  ---------------- |

  typedef actionlib::SimpleActionServer<iroc_fleet_manager::CoverageMissionAction> CoverageMissionServer;

  void                                                           actionCallbackGoal();
  void                                                           actionCallbackPreempt();
  void                                                           actionPublishFeedback(void);
  std::unique_ptr<CoverageMissionServer>                         coverage_mission_server_ptr_;
  typedef iroc_fleet_manager::CoverageMissionGoal ActionServerGoal;
  typedef iroc_fleet_manager::CoverageMissionFeedback ActionServerFeedback;
  ActionServerGoal                                               action_server_goal_;
  std::recursive_mutex                                           action_server_mutex_;


  // | ----------------- mission manager action client stuff ---------------- |
  struct robot_mission_handler_t
  {
    std::string                                  robot_name;
    std::unique_ptr<MissionManagerClient>        action_client_ptr;
    ros::ServiceClient                           sc_robot_activation;
    ros::ServiceClient                           sc_robot_pausing;
    iroc_mission_handler::waypointMissionFeedback feedback;
    iroc_mission_handler::waypointMissionResult   result;
    bool                                         got_result = false;
  };

  struct fleet_mission_handlers_t
  {
    std::recursive_mutex                                                mtx;
    std::vector<robot_mission_handler_t>                                handlers;
  } fleet_mission_handlers_;

  std::vector<std::string> lost_robot_names_;

  void waypointMissionActiveCallback(const std::string& robot_name);
  void waypointMissionDoneCallback(const SimpleClientGoalState& state, const iroc_mission_handler::waypointMissionResultConstPtr& result,
                                   const std::string& robot_name);
  void waypointMissionFeedbackCallback(const iroc_mission_handler::waypointMissionFeedbackConstPtr& result, const std::string& robot_name);

  // | ------------------ Additional functions ------------------ |
  algorithm_config_t parse_algorithm_config(mrs_lib::ParamLoader& param_loader);
  std::map<std::string, IROC_CoverageManager::result_t> startRobotClients(const ActionServerGoal& goal);
  ActionServerFeedback processAggregatedFeedbackInfo(const std::vector<iroc_fleet_manager::WaypointMissionRobotFeedback>& robots_feedback);
  std::tuple<std::string, std::string> processFeedbackMsg();
  robot_mission_handler_t* findRobotHandler(const std::string& robot_name, fleet_mission_handlers_t& mission_handlers); 
  void cancelRobotClients();
  std::vector<iroc_fleet_manager::WaypointMissionRobotResult> getRobotResults();

  // some helper method overloads
  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc, typename Svc_T::Request req);

  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc);

  result_t callService(ros::ServiceClient& sc, const bool val);

};
//}

/* onInit() //{ */

void IROC_CoverageManager::onInit() {

  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  /* load parameters */
  mrs_lib::ParamLoader param_loader(nh_, "IROC_CoverageManager");

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");

  const auto main_timer_rate    = param_loader.loadParam2<double>("main_timer_rate");
  const auto feedback_timer_rate    = param_loader.loadParam2<double>("feedback_timer_rate");
  const auto no_message_timeout = param_loader.loadParam2<ros::Duration>("no_message_timeout");

  const auto planner_config = parse_algorithm_config(param_loader); 



  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[IROC_CoverageManager]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "IROC_CoverageManager";
  shopts.no_message_timeout = no_message_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  // | ------------------------- timers ------------------------- |

  timer_main_     = nh_.createTimer(ros::Rate(main_timer_rate), &IROC_CoverageManager::timerMain, this);
  timer_feedback_ = nh_.createTimer(ros::Rate(feedback_timer_rate), &IROC_CoverageManager::timerFeedback, this);


  // | --------------------- service clients -------------------- |
  
  // | --------------------- service servers -------------------- |

  ss_change_fleet_mission_state_ = nh_.advertiseService(nh_.resolveName("svc/change_fleet_mission_state"), &IROC_CoverageManager::changeFleetMissionStateCallback, this);
  ROS_INFO("[IROC_CoverageManager]: Created ServiceServer on service \'svc_server/change_mission_state\' -> \'%s\'", ss_change_fleet_mission_state_.getService().c_str());
  ss_change_robot_mission_state_ = nh_.advertiseService(nh_.resolveName("svc/change_robot_mission_state"), &IROC_CoverageManager::changeRobotMissionStateCallback, this);
  ROS_INFO("[IROC_CoverageManager]: Created ServiceServer on service \'svc_server/change_robot_mission_state\' -> \'%s\'", ss_change_robot_mission_state_.getService().c_str());

  // | ------------------ action server methods ----------------- |
  coverage_mission_server_ptr_ = std::make_unique<CoverageMissionServer>(nh_, ros::this_node::getName(), false);
  coverage_mission_server_ptr_->registerGoalCallback(boost::bind(&IROC_CoverageManager::actionCallbackGoal, this));
  coverage_mission_server_ptr_->registerPreemptCallback(boost::bind(&IROC_CoverageManager::actionCallbackPreempt, this));
  coverage_mission_server_ptr_->start();

  // | --------------------- finish the init -------------------- |

  ROS_INFO("[IROC_CoverageManager]: initialized");
  ROS_INFO("[IROC_CoverageManager]: --------------------");

  is_initialized_ = true;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void IROC_CoverageManager::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    ROS_WARN_THROTTLE(1, "[iroc_fleet_manager]: Waiting for nodelet initialization");
    return;
  }

  //Activate based on asynchronous service call which locks the action server and fleet_mission_handler mutexes
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
      //Check if any missions aborted early
      any_failure = std::any_of(fleet_mission_handlers_.handlers.begin(),
          fleet_mission_handlers_.handlers.end(), [](const auto& handler) {
          return handler.got_result && !handler.result.success;});
    }

    if (any_failure) {
      ROS_WARN("[IROC_CoverageManager]: Early failure detected, aborting mission.");
      iroc_fleet_manager::CoverageMissionResult action_server_result;
      action_server_result.success = false;
      action_server_result.message = "Early failure detected, aborting mission.";
      action_server_result.robots_results = getRobotResults();
      active_mission_ = false;
      coverage_mission_server_ptr_->setAborted(action_server_result);
      cancelRobotClients(); 
      ROS_INFO("[IROC_CoverageManager]: Mission aborted.");
      return;
    }

    {
      std::scoped_lock lock(fleet_mission_handlers_.mtx);
      got_all_results = std::all_of(fleet_mission_handlers_.handlers.begin(),
          fleet_mission_handlers_.handlers.end(), [](const auto& handler) {
          return handler.got_result;});

      all_success = std::all_of(fleet_mission_handlers_.handlers.begin(),
          fleet_mission_handlers_.handlers.end(), [](const auto& handler) {
          return handler.result.success;});
    }

    //Finish mission when we get all the robots result
    if (got_all_results) {
      if (!all_success) {
        ROS_WARN("[IROC_CoverageManager]: Not all robots finished successfully, finishing mission. ");
        iroc_fleet_manager::CoverageMissionResult action_server_result;
        action_server_result.success = false;
        action_server_result.message = "Not all robots finished successfully, finishing mission";
        action_server_result.robots_results = getRobotResults();
        active_mission_ = false;
        coverage_mission_server_ptr_->setAborted(action_server_result);
        cancelRobotClients();
        ROS_INFO("[IROC_CoverageManager]: Mission finished.");
        return;
      }

      ROS_INFO("[IROC_CoverageManager]: All robots finished successfully, finishing mission."); 
      iroc_fleet_manager::CoverageMissionResult action_server_result;
      action_server_result.success = true;
      action_server_result.message = "All robots finished successfully, mission finished";
      action_server_result.robots_results = getRobotResults();

      active_mission_ = false;
      coverage_mission_server_ptr_->setSucceeded(action_server_result);
      cancelRobotClients();
      ROS_INFO("[IROC_CoverageManager]: Mission finished.");
    } 
  }
}

//}

/* timerFeedback() //{ */
void IROC_CoverageManager::timerFeedback([[maybe_unused]] const ros::TimerEvent& event) {
  
  if (!is_initialized_) {
    ROS_WARN_THROTTLE(1, "[MissionManager]: Waiting for nodelet initialization");
    return;
  }
  
  //Activate based on asynchronous service call which locks the action server and fleet_mission_handler mutexes
  if (active_mission_change_ || !active_mission_) {
    return;
  }

  actionPublishFeedback();
 
}
//}

// | ----------------- service server callback ---------------- |

/*  changeFleetMissionStateCallback()//{ */

bool IROC_CoverageManager::changeFleetMissionStateCallback(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res) {

  active_mission_change_ = true;
  std::scoped_lock lock(action_server_mutex_,fleet_mission_handlers_.mtx);

  ROS_INFO_STREAM("[IROC_CoverageManager]: Received a " << req.value<< " request for the fleet");
  std::stringstream ss;
  bool success = true;
  if (coverage_mission_server_ptr_->isActive()) {
    if (req.value == "start") {
      ROS_INFO_STREAM("[IROC_CoverageManager]: Calling mission activation.");
      for (auto& rh : fleet_mission_handlers_.handlers) {
        const auto resp = callService<std_srvs::Trigger>(rh.sc_robot_activation);
        if (!resp.success) { 
          success = false; 
          ROS_WARN_STREAM("[IROC_CoverageManager]: "<< "Call for robot \"" << rh.robot_name << "\" was not successful with message: " << resp.message << "\n");
          ss << "Call for robot \"" << rh.robot_name << "\" was not successful with message: " << resp.message << "\n";
        }
      }
    } else if (req.value == "pause") {
      ROS_INFO_STREAM("[IROC_CoverageManager]: Calling mission pausing.");
      for (auto& rh : fleet_mission_handlers_.handlers) {
        const auto resp = callService<std_srvs::Trigger>(rh.sc_robot_pausing);
        if (!resp.success) { 
          success = false; 
          ROS_WARN_STREAM("[IROC_CoverageManager]: "<< "Call for robot \"" << rh.robot_name << "\" was not successful with message: " << resp.message << "\n");
          ss << "Call for robot \"" << rh.robot_name << "\" was not successful with message: " << resp.message << "\n";
        }
      }
    } else if (req.value == "stop") {
      ROS_INFO_STREAM("[IROC_CoverageManager]: Calling mission stopping.");
      for (auto& rh : fleet_mission_handlers_.handlers) {
        const auto action_client_state = rh.action_client_ptr->getState(); 
        if (action_client_state.isDone()) {
          ss << "robot \"" << rh.robot_name << "\" mission done, skipping\n";
          ROS_WARN_STREAM("[IROC_CoverageManager]: Robot \"" << rh.robot_name << "\" mission done. Skipping.");
        } else {
          ROS_INFO_STREAM("[IROC_CoverageManager]: Cancelling \"" << rh.robot_name << "\" mission.");
          rh.action_client_ptr->cancelGoal();
          rh.action_client_ptr->waitForResult(ros::Duration(1.0));
        }
      }
    } else {
      success = false;
      ss << "Unsupported type\n";
    }

  } else {
    success = false;
    ss << "No active mission.\n";
  }

  if (success) {
    ROS_INFO_STREAM("[IROCAutonomyTestManager]: Succesfully processed the  "<< req.value <<" request.");
    ss << "Succesfully processed the  "<< req.value <<" request.\n";
  } else {
    ROS_WARN("[IROCAutonomyTestManager]: Failure: %s", res.message.c_str());
  };

  res.success = success;
  res.message = ss.str();
  active_mission_change_ = false;
  return true;
}
//}

/*  changeRobotMissionStateCallback()//{ */

bool IROC_CoverageManager::changeRobotMissionStateCallback(iroc_fleet_manager::ChangeRobotMissionStateSrv::Request& req, iroc_fleet_manager::ChangeRobotMissionStateSrv::Response& res) {

  active_mission_change_ = true;
  std::scoped_lock lock(action_server_mutex_,fleet_mission_handlers_.mtx);
  ROS_INFO_STREAM("[IROC_CoverageManager]: Received a " << req.type << " request for " << req.robot_name);
  std::stringstream ss;
  auto* rh_ptr = findRobotHandler(req.robot_name, fleet_mission_handlers_);
  if (rh_ptr == nullptr) {
    ss << "robot \"" << req.robot_name << "\" not found as a part of the mission, skipping\n";
    ROS_WARN_STREAM("[IROC_CoverageManager]: Robot \"" << req.robot_name << "\" not found as a part of the mission. Skipping.");
    res.message = ss.str();
    res.success = false;
    return true;
  }

  bool success = true;
  if (coverage_mission_server_ptr_->isActive()) {
    if (req.type == "start") {
      ROS_INFO_STREAM("[IROC_CoverageManager]: Calling mission pausing for robot: " << req.robot_name << ".");
      const auto resp = callService<std_srvs::Trigger>(rh_ptr->sc_robot_activation);
      if (!resp.success) { 
        success = false; 
        ROS_WARN_STREAM("[IROC_CoverageManager]: "<< "Call for robot \"" << req.robot_name << "\" was not successful with message: " << resp.message << "\n");
        ss << "Call for robot \"" << req.robot_name << "\" was not successful with message: " << resp.message << "\n";
      } else {
        ss << "Call successful.\n";
      }
    } else if (req.type == "pause") {
      ROS_INFO_STREAM("[IROC_CoverageManager]: Calling mission pausing for robot: " << req.robot_name << ".");
      const auto resp = callService<std_srvs::Trigger>(rh_ptr->sc_robot_pausing);
      if (!resp.success) { 
        success = false; 
        ROS_WARN_STREAM("[IROC_CoverageManager]: "<< "Call for robot \"" << req.robot_name << "\" was not successful with message: " << resp.message << "\n");
        ss << "Call for robot \"" << req.robot_name << "\" was not successful with message: " << resp.message << "\n";
      } else {
        ss << "Call successful.\n";
      }
    } else if (req.type == "stop") {
      ROS_INFO_STREAM("[IROC_CoverageManager]: Calling mission stopping.");
      const auto action_client_state = rh_ptr->action_client_ptr->getState(); 
      if (action_client_state.isDone()) {
        ss << "robot \"" << rh_ptr->robot_name << "\" mission done, skipping\n";
        success = false;
        ROS_WARN_STREAM("[IROC_CoverageManager]: Robot \"" << rh_ptr->robot_name << "\" mission done. Skipping.");
      } else {
        ss << "Call successful.\n";
        ROS_INFO_STREAM("[IROC_CoverageManager]: Cancelling \"" << rh_ptr->robot_name << "\" mission.");
        rh_ptr->action_client_ptr->cancelGoal();
        rh_ptr->action_client_ptr->waitForResult(ros::Duration(1.0));
      }
    } else {
      success = false;
      ss << "Unsupported type\n";
    }

  } else {
    success = false;
    ss << "No active mission\n";
  }

  if (success) {
    ROS_INFO_STREAM("[IROCAutonomyTestManager]: Succesfully processed the  "<< req.type<<" request for "<< req.robot_name <<".");
    ss << "Succesfully processed the  "<< req.type <<" request for "<< req.robot_name << " <.\n";
  } else {
    ROS_WARN("[IROCAutonomyTestManager]: Failure: %s", res.message.c_str());
  };

  res.success = success;
  res.message = ss.str();
  active_mission_change_ = false;
  return true;
}
//}

// --------------------------------------------------------------
// |                  acition client callbacks                  |
// --------------------------------------------------------------

/* waypointMissionActiveCallback //{ */

void IROC_CoverageManager::waypointMissionActiveCallback(const std::string& robot_name) {
  ROS_INFO_STREAM("[IROC_CoverageManager]: Action server on robot " << robot_name << " is processing the goal.");
}

//}

/* waypointMissionDoneCallback //{ */

void IROC_CoverageManager::waypointMissionDoneCallback(const SimpleClientGoalState& state, const iroc_mission_handler::waypointMissionResultConstPtr& result,
    const std::string& robot_name) {

  if (!active_mission_) {
    return;
  }

  if (result == NULL){ 
    active_mission_ = false;
    
    lost_robot_names_.push_back(robot_name);
    ROS_WARN_STREAM("[IROC_CoverageManager]: Robot " << robot_name << " mission_handler died/ or restarted while mission was active, and action server connection was lost!, reconnection is not currently handled, if mission handler was restarted need to upload a new mission!");
    iroc_fleet_manager::CoverageMissionResult action_server_result;
    action_server_result.success = false;
    action_server_result.message = "Probably mission_handler died, and action server connection was lost!, reconnection is not currently handled, if mission handler was restarted need to upload a new mission!";
    action_server_result.robots_results = getRobotResults();


    coverage_mission_server_ptr_->setAborted(action_server_result);
    cancelRobotClients();
    ROS_INFO("[IROC_CoverageManager]: Mission aborted.");
    return;
  }

  if (result->success) {
    ROS_INFO_STREAM("[IROC_CoverageManager]: Action server on robot " << robot_name << " finished with state: \"" << state.toString() << "\". Result message is: \""
          << result->message << "\"");
  } else {
    ROS_WARN_STREAM("[IROC_CoverageManager]: Action server on robot " << robot_name << " finished with state: \"" << state.toString() << "\". Result message is: \""
          << result->message << "\"");
  }

  {
    std::scoped_lock lck(fleet_mission_handlers_.mtx);
    auto* rh_ptr = findRobotHandler(robot_name, fleet_mission_handlers_);
    rh_ptr->result = *result;
    rh_ptr->got_result = true;
  }
}

//}

/* waypointMissionFeedbackCallback //{ */

void IROC_CoverageManager::waypointMissionFeedbackCallback(const iroc_mission_handler::waypointMissionFeedbackConstPtr& feedback, const std::string& robot_name) {
  
  if (!active_mission_) {
    return;
  }

  {
    std::scoped_lock lck(fleet_mission_handlers_.mtx);
    auto* rh_ptr = findRobotHandler(robot_name, fleet_mission_handlers_);
    rh_ptr->feedback = *feedback;
  }
}

//}

// | ---------------------- action server callbacks --------------------- |

/*  actionCallbackGoal()//{ */

void IROC_CoverageManager::actionCallbackGoal() {
  std::scoped_lock  lock(action_server_mutex_);
  boost::shared_ptr<const iroc_fleet_manager::CoverageMissionGoal> new_action_server_goal = coverage_mission_server_ptr_->acceptNewGoal();
  ROS_INFO_STREAM("[IROC_CoverageManager]: Action server received a new goal: \n" << *new_action_server_goal);

  if (!is_initialized_) {
    iroc_fleet_manager::CoverageMissionResult action_server_result;
    action_server_result.success = false;
    action_server_result.message = "Not  initialized yet";
    action_server_result.robots_results = getRobotResults();
    ROS_WARN("[IROC_CoverageManager]: not initialized yet");
    coverage_mission_server_ptr_->setAborted(action_server_result);
    return;
  }

  //Start each robot action/service clients with mission_manager 
  const auto results = startRobotClients(*new_action_server_goal);

  bool all_success = std::all_of(results.begin(), results.end(),[](const auto& pair){
      return pair.second.success;
      }
  );

  if (!all_success) {
      iroc_fleet_manager::CoverageMissionResult action_server_result;
      iroc_fleet_manager::WaypointMissionRobotResult robot_result;
      for (const auto& result : results) {
        std::stringstream ss;
        robot_result.name = result.first;
        robot_result.message = result.second.message;
        robot_result.success = result.second.success;
        action_server_result.robots_results.emplace_back(robot_result);
        if (!result.second.success) {
          ss << result.first << " failed with response: " << result.second.message;
          ROS_WARN_STREAM("[IROC_CoverageManager]: Failure starting robot clients: " << ss.str());
        }
      }
      action_server_result.success = false;
      action_server_result.message = "Failure starting robot clients.";
      coverage_mission_server_ptr_->setAborted(action_server_result);
      cancelRobotClients(); 
      ROS_INFO("[IROC_CoverageManager]: Mission Aborted.");
      return;
  }
  ROS_INFO("[IROC_CoverageManager]: Succesfully sent the goal to robots in mission.");

  active_mission_ = true;
  action_server_goal_ = *new_action_server_goal;
}

//}

/*  actionCallbackPreempt()//{ */

void IROC_CoverageManager::actionCallbackPreempt() {
  std::scoped_lock lock(action_server_mutex_);
  if (coverage_mission_server_ptr_->isActive()) {
    if (coverage_mission_server_ptr_->isNewGoalAvailable()) {
      ROS_INFO("[IROC_CoverageManager]: Preemption toggled for ActionServer.");
      iroc_fleet_manager::CoverageMissionResult action_server_result;
      action_server_result.success = false;
      action_server_result.message = "Preempted by client";
      ROS_WARN_STREAM("[IROC_CoverageManager]: Preempted by the client");
      coverage_mission_server_ptr_->setPreempted(action_server_result);
      cancelRobotClients();
      ROS_INFO("[IROC_CoverageManager]: Mission stopped by preemtption.");
    } else {
      ROS_INFO("[IROC_CoverageManager]: Cancel toggled for ActionServer.");

      iroc_fleet_manager::CoverageMissionResult action_server_result;
      action_server_result.success = false;
      action_server_result.message = "Mission stopped.";
      active_mission_ = false;
      coverage_mission_server_ptr_->setAborted(action_server_result);
      cancelRobotClients();
      ROS_INFO("[IROC_CoverageManager]: Mission stopped.");
    }
  }
}

//}

/* actionPublishFeedback()//{ */

void IROC_CoverageManager::actionPublishFeedback() {
  std::scoped_lock lock(action_server_mutex_);

  //Collect the feedback from active robots in the mission 
  std::vector<iroc_fleet_manager::WaypointMissionRobotFeedback> robots_feedback;
  {
    std::scoped_lock lck(fleet_mission_handlers_.mtx);
    iroc_fleet_manager::WaypointMissionRobotFeedback robot_feedback;
    //Fill the robots feedback vector
    for (const auto& rh : fleet_mission_handlers_.handlers) {
      robot_feedback.name = rh.robot_name;
      robot_feedback.message = rh.feedback.message;  
      robot_feedback.goal_idx = rh.feedback.goal_idx;
      robot_feedback.distance_to_finish =rh.feedback.distance_to_finish;
      robot_feedback.goal_estimated_arrival_time = rh.feedback.goal_estimated_arrival_time;
      robot_feedback.mission_progress = rh.feedback.mission_progress;
      robot_feedback.distance_to_closest_goal  = rh.feedback.distance_to_closest_goal;
      robot_feedback.finish_estimated_arrival_time = rh.feedback.finish_estimated_arrival_time;
      robot_feedback.goal_progress = rh.feedback.goal_progress;
      robots_feedback.emplace_back(robot_feedback);
    }
  
    if (coverage_mission_server_ptr_->isActive()) {
      auto action_server_feedback = processAggregatedFeedbackInfo(robots_feedback);
      coverage_mission_server_ptr_->publishFeedback(action_server_feedback);
    }
  }
}

//}

// | -------------------- support functions ------------------- |


/* parse_algorithm_config() //{ */

algorithm_config_t IROC_CoverageManager::parse_algorithm_config(mrs_lib::ParamLoader& param_loader )
{
  const std::string yaml_prefix = "coverage_planner/";
  algorithm_config_t algorithm_config;

  // Load basic drone parameters
  param_loader.loadParam(yaml_prefix + "drone_mass", algorithm_config.energy_calculator_config.drone_mass);
  param_loader.loadParam(yaml_prefix + "drone_area", algorithm_config.energy_calculator_config.drone_area);
  param_loader.loadParam(yaml_prefix + "average_acceleration", algorithm_config.energy_calculator_config.average_acceleration);
  param_loader.loadParam(yaml_prefix + "propeller_radius", algorithm_config.energy_calculator_config.propeller_radius);
  param_loader.loadParam(yaml_prefix + "number_of_propellers", algorithm_config.energy_calculator_config.number_of_propellers);
  param_loader.loadParam(yaml_prefix + "allowed_path_deviation", algorithm_config.energy_calculator_config.allowed_path_deviation);
  param_loader.loadParam(yaml_prefix + "number_of_rotations", algorithm_config.number_of_rotations);

  // Load battery model parameters
  const std::string battery_prefix = yaml_prefix + "battery_model/";
  param_loader.loadParam(battery_prefix + "cell_capacity", algorithm_config.energy_calculator_config.battery_model.cell_capacity);
  param_loader.loadParam(battery_prefix + "number_of_cells", algorithm_config.energy_calculator_config.battery_model.number_of_cells);
  param_loader.loadParam(battery_prefix + "d0", algorithm_config.energy_calculator_config.battery_model.d0);
  param_loader.loadParam(battery_prefix + "d1", algorithm_config.energy_calculator_config.battery_model.d1);
  param_loader.loadParam(battery_prefix + "d2", algorithm_config.energy_calculator_config.battery_model.d2);
  param_loader.loadParam(battery_prefix + "d3", algorithm_config.energy_calculator_config.battery_model.d3);

  // Load speed model parameters
  const std::string speed_prefix = yaml_prefix + "best_speed_model/";
  param_loader.loadParam(speed_prefix + "c0", algorithm_config.energy_calculator_config.best_speed_model.c0);
  param_loader.loadParam(speed_prefix + "c1", algorithm_config.energy_calculator_config.best_speed_model.c1);
  param_loader.loadParam(speed_prefix + "c2", algorithm_config.energy_calculator_config.best_speed_model.c2);

  // Load coordinate system parameters
  param_loader.loadParam(yaml_prefix + "points_in_lat_lon", algorithm_config.points_in_lat_lon);
  if (algorithm_config.points_in_lat_lon) {
    param_loader.loadParam(yaml_prefix + "latitude_origin", algorithm_config.lat_lon_origin.first);
    param_loader.loadParam(yaml_prefix + "longitude_origin", algorithm_config.lat_lon_origin.second);
  }

  // Load mission parameters
  param_loader.loadParam(yaml_prefix + "number_of_drones", algorithm_config.number_of_drones);
  param_loader.loadParam(yaml_prefix + "sweeping_step", algorithm_config.sweeping_step);

  int decomposition_method;
  param_loader.loadParam(yaml_prefix + "decomposition_method", decomposition_method);
  algorithm_config.decomposition_type = static_cast<decomposition_type_t>(decomposition_method);

  param_loader.loadParam(yaml_prefix + "min_sub_polygons_per_uav", algorithm_config.min_sub_polygons_per_uav);

  // Load starting position
  param_loader.loadParam(yaml_prefix + "start_x", algorithm_config.start_pos.first);
  param_loader.loadParam(yaml_prefix + "start_y", algorithm_config.start_pos.second);

  // Load optimization parameters
  param_loader.loadParam(yaml_prefix + "rotations_per_cell", algorithm_config.rotations_per_cell);
  param_loader.loadParam(yaml_prefix + "no_improvement_cycles_before_stop", algorithm_config.no_improvement_cycles_before_stop);
  param_loader.loadParam(yaml_prefix + "max_single_path_energy", algorithm_config.max_single_path_energy);

  return algorithm_config;
  double drone_mass;
}
//}

/* startRobotClients() //{ */

std::map<std::string,IROC_CoverageManager::result_t> IROC_CoverageManager::startRobotClients(const ActionServerGoal& goal){
  std::scoped_lock lck(fleet_mission_handlers_.mtx);
  std::map<std::string,IROC_CoverageManager::result_t> robot_results;

  // Clear the previous handlers
  fleet_mission_handlers_.handlers.clear();
  lost_robot_names_.clear();

  std::vector<iroc_fleet_manager::WaypointMissionRobot> mission_robots; 

  // TODO Here we get the coverage path from the planner 
  // TODO Here we need to fill the mission_robots vector and assign the path to each robot  
  {
    fleet_mission_handlers_.handlers.reserve(mission_robots.size());
    // Initialize the robots received in the goal request
    for (const auto& robot : mission_robots) {
      bool success = true;
      std::stringstream ss;
      const std::string waypoint_action_client_topic = "/" + robot.name + nh_.resolveName("ac/waypoint_mission");
      auto action_client_ptr                = std::make_unique<MissionManagerClient>(waypoint_action_client_topic, false);

      // Need to wait for server
      if (!action_client_ptr->waitForServer(ros::Duration(5.0))) {
        ROS_WARN("[IROC_CoverageManager]: Server connection failed for robot %s ", robot.name.c_str());
        ss << "Action server from robot: " + robot.name + " failed to connect. Check the iroc_mission_handler node.\n";
        robot_results[robot.name].message = ss.str();
        robot_results[robot.name].success = false; 
        success = false;
      }

      ROS_INFO("[IROC_CoverageManager]: Created action client on topic \'ac/waypoint_mission\' -> \'%s\'", waypoint_action_client_topic.c_str());
      MissionManagerActionServerGoal action_goal;
      action_goal.frame_id           = robot.frame_id;
      action_goal.height_id          = robot.height_id; 
      action_goal.terminal_action    = robot.terminal_action; 
      action_goal.points             = robot.points;

      if (!action_client_ptr->isServerConnected()) {
        ss << "Action server from robot: " + robot.name + " is not connected. Check the iroc_mission_handler node.\n";
        ROS_WARN_STREAM("[IROC_CoverageManager]: Action server from robot :" + robot.name + " is not connected. Check the iroc_mission_handler node.");
        robot_results[robot.name].message = ss.str();
        robot_results[robot.name].success = false; 
        success = false;
      }

      if (!action_client_ptr->getState().isDone()) {
        ss << "Mission on robot: " + robot.name + " already running. Terminate the previous one, or wait until it is finished.\n";
        ROS_WARN_STREAM("[IROC_CoverageManager]: Mission on robot: " + robot.name + " already running. Terminate the previous one, or wait until it is finished.\n");
        robot_results[robot.name].message = ss.str();
        robot_results[robot.name].success = false; 
        success = false;
      }

      // Send the goal to robot in mission_manager
      action_client_ptr->sendGoal(
        action_goal, std::bind(&IROC_CoverageManager::waypointMissionDoneCallback, this, std::placeholders::_1, std::placeholders::_2, robot.name),
        std::bind(&IROC_CoverageManager::waypointMissionActiveCallback, this, robot.name),
        std::bind(&IROC_CoverageManager::waypointMissionFeedbackCallback, this, std::placeholders::_1, robot.name));

      // This is important to wait for some time in case the goal was rejected
      // We can replace it to wait while the sate is pending
      ros::Duration(0.5).sleep();

      if (action_client_ptr->getState().isDone()) {
        auto result =  action_client_ptr->getResult();
        ss << result->message;
        ROS_INFO_STREAM("[IROC_CoverageManagerDebug]: result: " << ss.str());
        robot_results[robot.name].message = ss.str();
        robot_results[robot.name].success = false; 
        success = false;
      }

      if (!success) {
        continue;
      }
      // Save the ros service clients from mission_manager
      const std::string mission_activation_client_topic = "/" + robot.name + nh_.resolveName("svc/mission_activation");
      ros::ServiceClient sc_robot_activation = nh_.serviceClient<std_srvs::Trigger>(mission_activation_client_topic);
      ROS_INFO("[IROC_CoverageManager]: Created ServiceClient on service \'svc/mission_activation\' -> \'%s\'", sc_robot_activation.getService().c_str());

      const std::string mission_pausing_client_topic = "/" + robot.name + nh_.resolveName("svc/mission_pausing");
      ros::ServiceClient sc_robot_pausing = nh_.serviceClient<std_srvs::Trigger>(mission_pausing_client_topic);
      ROS_INFO("[IROC_CoverageManager]: Created ServiceClient on service \'svc/mission_pausing\' -> \'%s\'", sc_robot_pausing.getService().c_str());

      robot_mission_handler_t robot_handler;
      robot_handler.robot_name          = robot.name;
      robot_handler.action_client_ptr   = std::move(action_client_ptr); 
      robot_handler.sc_robot_activation = sc_robot_activation;
      robot_handler.sc_robot_pausing    = sc_robot_pausing;
      // Save robot clients in mission handler
      fleet_mission_handlers_.handlers.emplace_back(std::move(robot_handler));
      ss << "Mission on robot: " + robot.name + " was successfully processed"; 
      robot_results[robot.name].message = ss.str();
      robot_results[robot.name].success = true; 
    }
  }

  return robot_results;
}

//}

/* processAggregatedFeedbackInfo() //{ */

IROC_CoverageManager::ActionServerFeedback IROC_CoverageManager::processAggregatedFeedbackInfo(const std::vector<iroc_fleet_manager::WaypointMissionRobotFeedback>& robots_feedback){

  IROC_CoverageManager::ActionServerFeedback action_server_feedback;
  std::vector<std::string> robots_msg;
  std::vector<double> robots_progress;

  for (const auto& rbf : robots_feedback) {
    robots_msg.emplace_back(rbf.message);
    robots_progress.emplace_back(rbf.mission_progress);
  }

  //Get average of active robots progress for the general progress
  auto mission_progress = std::accumulate(robots_progress.begin(), robots_progress.end(), 0.0) / robots_progress.size();

  // Checks if ALL messages are exactly "MISSION_LOADED"
  auto [message , state] = processFeedbackMsg();

  action_server_feedback.info.progress = mission_progress;
  action_server_feedback.info.message = message; 
  action_server_feedback.info.state = state; 
  action_server_feedback.info.robots_feedback = robots_feedback;

  return action_server_feedback; 
}

//}

/* getRobotResults() //{ */
std::vector<iroc_fleet_manager::WaypointMissionRobotResult> IROC_CoverageManager::getRobotResults(){ 
  // Get the robot results
  std::vector<iroc_fleet_manager::WaypointMissionRobotResult> robots_results;

  {
    std::scoped_lock lock(fleet_mission_handlers_.mtx);
    for (auto& handler : fleet_mission_handlers_.handlers) {
      iroc_fleet_manager::WaypointMissionRobotResult robot_result;
      if (handler.got_result && !handler.result.success) {
        robot_result.name    = handler.robot_name;
        robot_result.message = handler.result.message;
        robot_result.success = handler.result.success;
      } 

      if (handler.got_result) {
        robot_result.name    = handler.robot_name;
        robot_result.message = handler.result.message; 
        robot_result.success = handler.result.success; 
      } else {
        robot_result.name    = handler.robot_name;
        robot_result.message = "Robot did not finished it's mission, mission was aborted.";
        robot_result.success = false; 
      }
      robots_results.emplace_back(robot_result);
    }
  }

  // Print the robots result
  for (auto& robot_result : robots_results) {
    ROS_INFO("[IROC_CoverageManager]: Robot: %s, result: %s success: %d", robot_result.name.c_str(), robot_result.message.c_str(), robot_result.success);
  }

  return robots_results;
}
//}

/* processFeedbackMsg() //{ */

std::tuple<std::string, std::string> IROC_CoverageManager::processFeedbackMsg(){

  auto all_loaded = std::all_of(
    fleet_mission_handlers_.handlers.begin(),
    fleet_mission_handlers_.handlers.end(),
    [](const auto& handler) { 
        return handler.feedback.message == "MISSION_LOADED";  
    }
  );

  if (all_loaded) 
    return std::make_tuple("All missions loaded",iroc_fleet_manager::WaypointMissionInfo::STATE_TRAJECTORIES_LOADED);

  auto all_executing = std::all_of(
    fleet_mission_handlers_.handlers.begin(),
    fleet_mission_handlers_.handlers.end(),
    [](const auto& handler) { 
        return handler.feedback.message == "EXECUTING";  
    }
  );

  if (all_executing) 
    return std::make_tuple("Robots executing mission", iroc_fleet_manager::WaypointMissionInfo::STATE_EXECUTING);

  auto all_paused = std::all_of(
    fleet_mission_handlers_.handlers.begin(),
    fleet_mission_handlers_.handlers.end(),
    [](const auto& handler) { 
        return handler.feedback.message == "PAUSED";  
    }
  );

  if (all_paused) 
    return std::make_tuple("All robots paused", iroc_fleet_manager::WaypointMissionInfo::STATE_PAUSED);

  auto any_idle = std::any_of(
    fleet_mission_handlers_.handlers.begin(),
    fleet_mission_handlers_.handlers.end(),
    [](const auto& handler) { 
        return handler.feedback.message == "IDLE";  
    }
  );

  if (any_idle) 
    return std::make_tuple("One robot is idle, check mission_manager", iroc_fleet_manager::WaypointMissionInfo::STATE_ERROR);
    

  return std::make_tuple("Not defined message", iroc_fleet_manager::WaypointMissionInfo::STATE_INVALID);


}

//}

/* findRobotHandler() method //{ */
IROC_CoverageManager::robot_mission_handler_t* IROC_CoverageManager::findRobotHandler(const std::string& robot_name, fleet_mission_handlers_t& fleet_mission_handlers) {
  for (auto& rh : fleet_mission_handlers.handlers) {
    if (rh.robot_name == robot_name)
      return &rh;
  }
  ROS_WARN("[IROC_CoverageManager]: Robot handler not found!");
  return nullptr;
}
//}

/* cancelRobotClients() //{ */

void IROC_CoverageManager::cancelRobotClients(){

  {
    ROS_INFO("[IROC_CoverageManager]: Canceling robot clients...");
    std::scoped_lock lock(fleet_mission_handlers_.mtx);
    for (auto& rh : fleet_mission_handlers_.handlers) {

      //Check if robot is in list of lost robots
      bool lost_robot = std::any_of(lost_robot_names_.begin(), lost_robot_names_.end(), [&rh](const auto& lost_robot) {
          return rh.robot_name == lost_robot;});

      bool robot_got_result = std::any_of(fleet_mission_handlers_.handlers.begin(), fleet_mission_handlers_.handlers.end(),
          [&rh](const auto& handler) {
        return rh.robot_name == handler.robot_name && handler.got_result;});

      if (lost_robot || robot_got_result) {
        ROS_INFO_STREAM("[IROC_CoverageManager]: Robot \"" << rh.robot_name << "\" with no active mission, skipping cancel...");
        continue;
      }

      if (rh.action_client_ptr == NULL) {
        //This is possible if mission handler was restarted with an active mission
        ROS_WARN_STREAM("[IROC_CoverageManager]: Action client for robot \"" << rh.robot_name << "\" is null!");
        continue;
      }
      const auto action_client_state = rh.action_client_ptr->getState(); 
      if (!action_client_state.isDone()) {
        ROS_INFO_STREAM("[IROC_CoverageManager]: Robot \"" << rh.robot_name << "\" has an active mission, cancelling...");
        rh.action_client_ptr->cancelGoal();
        rh.action_client_ptr->waitForResult(ros::Duration(1.0));
      }
    }
  }
}

//}

/* callService() //{ */

template <typename Svc_T>
IROC_CoverageManager::result_t IROC_CoverageManager::callService(ros::ServiceClient& sc, typename Svc_T::Request req) {
  typename Svc_T::Response res;
  if (sc.call(req, res)) {
    if (res.success) {
      ROS_INFO_STREAM("Called service \"" << sc.getService() << "\" with response \"" << res.message << "\".");
      return {true, res.message};
    } else {
      ROS_WARN_STREAM("Called service \"" << sc.getService() << "\" with response \"" << res.message << "\".");
      return {false, res.message};
    }
  } else {
    const std::string msg = "Failed to call service \"" + sc.getService() + "\".";
    ROS_WARN_STREAM(msg);
    return {false, msg};
  }
}

template <typename Svc_T>
IROC_CoverageManager::result_t IROC_CoverageManager::callService(ros::ServiceClient& sc) {
  return callService<Svc_T>(sc, {});
}

IROC_CoverageManager::result_t IROC_CoverageManager::callService(ros::ServiceClient& sc, const bool val) {
  using svc_t = std_srvs::SetBool;
  svc_t::Request req;
  req.data = val;
  return callService<svc_t>(sc, req);
}
//}
}  // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::IROC_CoverageManager, nodelet::Nodelet);
