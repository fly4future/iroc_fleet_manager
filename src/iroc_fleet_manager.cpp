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
#include <mrs_mission_manager/waypointMissionAction.h>
#include <iroc_fleet_manager/WaypointFleetManagerAction.h>
#include <iroc_fleet_manager/WaypointMissionRobotFeedback.h>
#include <unistd.h>
#include <iostream>
#include <numeric>

//}

namespace iroc_fleet_manager
{

using namespace actionlib;

typedef SimpleActionClient<mrs_mission_manager::waypointMissionAction> MissionManagerClient;
typedef mrs_mission_manager::waypointMissionGoal                       MissionManagerActionServerGoal;

/* class IROCFleetManager //{ */

class IROCFleetManager : public nodelet::Nodelet {
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

  typedef actionlib::SimpleActionServer<iroc_fleet_manager::WaypointFleetManagerAction> MissionManagementServer;

  void                                                           actionCallbackGoal();
  void                                                           actionCallbackPreempt();
  void                                                           actionPublishFeedback(void);
  std::unique_ptr<MissionManagementServer>                       mission_management_server_ptr_;
  typedef iroc_fleet_manager::WaypointFleetManagerGoal ActionServerGoal;
  typedef iroc_fleet_manager::WaypointFleetManagerFeedback ActionServerFeedback;
  ActionServerGoal                                               action_server_goal_;
  std::recursive_mutex                                           action_server_mutex_;


  // | ----------------- mission manager action client stuff ---------------- |
  struct robot_mission_handler_t
  {
    std::string                                  robot_name;
    std::unique_ptr<MissionManagerClient>        action_client_ptr;
    ros::ServiceClient                           sc_robot_activation;
    ros::ServiceClient                           sc_robot_pausing;
    mrs_mission_manager::waypointMissionFeedback feedback;
    mrs_mission_manager::waypointMissionResult   result;
    bool                                         got_result = false;
  };

  struct fleet_mission_handlers_t
  {
    std::recursive_mutex                                                mtx;
    std::vector<robot_mission_handler_t>                                handlers;
  } fleet_mission_handlers_;

  void waypointMissionActiveCallback(const std::string& robot_name);
  void waypointMissionDoneCallback(const SimpleClientGoalState& state, const mrs_mission_manager::waypointMissionResultConstPtr& result,
                                   const std::string& robot_name);
  void waypointMissionFeedbackCallback(const mrs_mission_manager::waypointMissionFeedbackConstPtr& result, const std::string& robot_name);

  // | ------------------ Additional functions ------------------ |
  std::map<std::string, IROCFleetManager::result_t> startRobotClients(const ActionServerGoal& goal);
  ActionServerFeedback processAggregatedFeedbackInfo(const std::vector<iroc_fleet_manager::WaypointMissionRobotFeedback>& robots_feedback);
  std::tuple<std::string, std::string> processFeedbackMsg();
  robot_mission_handler_t* findRobotHandler(const std::string& robot_name, fleet_mission_handlers_t& mission_handlers); 
  void cancelRobotClients();

  // some helper method overloads
  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc, typename Svc_T::Request req);

  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc);

  result_t callService(ros::ServiceClient& sc, const bool val);

};
//}

/* onInit() //{ */

void IROCFleetManager::onInit() {

  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  /* load parameters */
  mrs_lib::ParamLoader param_loader(nh_, "IROCFleetManager");

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");

  const auto main_timer_rate    = param_loader.loadParam2<double>("main_timer_rate");
  const auto feedback_timer_rate    = param_loader.loadParam2<double>("feedback_timer_rate");
  const auto no_message_timeout = param_loader.loadParam2<ros::Duration>("no_message_timeout");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[IROCFleetManager]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "IROCFleetManager";
  shopts.no_message_timeout = no_message_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  // | ------------------------- timers ------------------------- |

  timer_main_     = nh_.createTimer(ros::Rate(main_timer_rate), &IROCFleetManager::timerMain, this);
  timer_feedback_ = nh_.createTimer(ros::Rate(feedback_timer_rate), &IROCFleetManager::timerFeedback, this);


  // | --------------------- service clients -------------------- |
  
  // | --------------------- service servers -------------------- |

  ss_change_fleet_mission_state_ = nh_.advertiseService(nh_.resolveName("svc/change_fleet_mission_state"), &IROCFleetManager::changeFleetMissionStateCallback, this);
  ROS_INFO("[IROCFleetManager]: Created ServiceServer on service \'svc_server/change_mission_state\' -> \'%s\'", ss_change_fleet_mission_state_.getService().c_str());
  ss_change_robot_mission_state_ = nh_.advertiseService(nh_.resolveName("svc/change_robot_mission_state"), &IROCFleetManager::changeRobotMissionStateCallback, this);
  ROS_INFO("[IROCFleetManager]: Created ServiceServer on service \'svc_server/change_robot_mission_state\' -> \'%s\'", ss_change_robot_mission_state_.getService().c_str());

  // | ------------------ action server methods ----------------- |
  mission_management_server_ptr_ = std::make_unique<MissionManagementServer>(nh_, ros::this_node::getName(), false);
  mission_management_server_ptr_->registerGoalCallback(boost::bind(&IROCFleetManager::actionCallbackGoal, this));
  mission_management_server_ptr_->registerPreemptCallback(boost::bind(&IROCFleetManager::actionCallbackPreempt, this));
  mission_management_server_ptr_->start();

  // | --------------------- finish the init -------------------- |

  ROS_INFO("[IROCFleetManager]: initialized");
  ROS_INFO("[IROCFleetManager]: --------------------");

  is_initialized_ = true;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void IROCFleetManager::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

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
      ROS_WARN("[IROCFleetManager]: Early failure detected, aborting mission.");
      iroc_fleet_manager::WaypointFleetManagerResult action_server_result;
      action_server_result.success = false;
      action_server_result.messages.emplace_back("Early failure detected, aborting mission");
      active_mission_ = false;
      mission_management_server_ptr_->setAborted(action_server_result);
      cancelRobotClients(); 
      ROS_INFO("[IROCFleetManager]: Mission aborted.");
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
        ROS_WARN("[IROCFleetManager]: Not all robots finished successfully, finishing mission. ");
        iroc_fleet_manager::WaypointFleetManagerResult action_server_result;
        action_server_result.success = false;
        action_server_result.messages.emplace_back("Not all robots finished successfully, finishing mission");
        active_mission_ = false;
        mission_management_server_ptr_->setAborted(action_server_result);
        cancelRobotClients();
        return;
      }

      ROS_INFO("[IROCFleetManager]: All robots finished successfully, finishing mission."); 
      iroc_fleet_manager::WaypointFleetManagerResult action_server_result;
      action_server_result.success = true;
      action_server_result.messages.emplace_back("All robots finished successfully, mission finished");
      active_mission_ = false;
      mission_management_server_ptr_->setSucceeded(action_server_result);
      cancelRobotClients();
    } 
  }
}

//}

/* timerFeedback() //{ */
void IROCFleetManager::timerFeedback([[maybe_unused]] const ros::TimerEvent& event) {
  
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

bool IROCFleetManager::changeFleetMissionStateCallback(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res) {

  active_mission_change_ = true;
  std::scoped_lock lock(action_server_mutex_,fleet_mission_handlers_.mtx);

  ROS_INFO_STREAM("[IROCFleetManager]: Received a " << req.value<< " request for the fleet");
  std::stringstream ss;
  bool success = true;
  if (mission_management_server_ptr_->isActive()) {
    if (req.value == "start") {
      ROS_INFO_STREAM("[IROCFleetManager]: Calling mission activation.");
      for (auto& rh : fleet_mission_handlers_.handlers) {
        const auto resp = callService<std_srvs::Trigger>(rh.sc_robot_activation);
        if (!resp.success) { 
          success = false; 
          ROS_WARN_STREAM("[IROCFleetManager]: "<< "Call for robot \"" << rh.robot_name << "\" was not successful with message: " << resp.message << "\n");
          ss << "Call for robot \"" << rh.robot_name << "\" was not successful with message: " << resp.message << "\n";
        }
      }
    } else if (req.value == "pause") {
      ROS_INFO_STREAM("[IROCFleetManager]: Calling mission pausing.");
      for (auto& rh : fleet_mission_handlers_.handlers) {
        const auto resp = callService<std_srvs::Trigger>(rh.sc_robot_pausing);
        if (!resp.success) { 
          success = false; 
          ROS_WARN_STREAM("[IROCFleetManager]: "<< "Call for robot \"" << rh.robot_name << "\" was not successful with message: " << resp.message << "\n");
          ss << "Call for robot \"" << rh.robot_name << "\" was not successful with message: " << resp.message << "\n";
        }
      }
    } else if (req.value == "stop") {
      ROS_INFO_STREAM("[IROCFleetManager]: Calling mission stopping.");
      for (auto& rh : fleet_mission_handlers_.handlers) {
        const auto action_client_state = rh.action_client_ptr->getState(); 
        if (action_client_state.isDone()) {
          ss << "robot \"" << rh.robot_name << "\" mission done, skipping\n";
          ROS_WARN_STREAM("[IROCFleetManager]: Robot \"" << rh.robot_name << "\" mission done. Skipping.");
        } else {
          ROS_INFO_STREAM("[IROCFleetManager]: Cancelling \"" << rh.robot_name << "\" mission.");
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
  res.success = success;
  res.message = ss.str();
  if (res.success){
    ROS_INFO_STREAM("[IROCFleetManager]: Succesfully processed the  "<< req.value <<" request.");
  }else{
    ROS_WARN("[IROCFleetManager]: Failure: %s", res.message.c_str());
  };
  active_mission_change_ = false;
  return true;
}

//}

/*  changeRobotMissionStateCallback()//{ */

bool IROCFleetManager::changeRobotMissionStateCallback(iroc_fleet_manager::ChangeRobotMissionStateSrv::Request& req, iroc_fleet_manager::ChangeRobotMissionStateSrv::Response& res) {

  active_mission_change_ = true;
  std::scoped_lock lock(action_server_mutex_,fleet_mission_handlers_.mtx);
  ROS_INFO_STREAM("[IROCFleetManager]: Received a " << req.type << " request for " << req.robot_name);
  std::stringstream ss;
  auto* rh_ptr = findRobotHandler(req.robot_name, fleet_mission_handlers_);
  if (rh_ptr == nullptr) {
    ss << "robot \"" << req.robot_name << "\" not found as a part of the mission, skipping\n";
    ROS_WARN_STREAM("[IROCFleetManager]: Robot \"" << req.robot_name << "\" not found as a part of the mission. Skipping.");
    res.message = ss.str();
    res.success = false;
    return true;
  }

  bool success = true;
  if (mission_management_server_ptr_->isActive()) {
    if (req.type == "start") {
      ROS_INFO_STREAM("[IROCFleetManager]: Calling mission pausing for robot: " << req.robot_name << ".");
      const auto resp = callService<std_srvs::Trigger>(rh_ptr->sc_robot_activation);
      if (!resp.success) { 
        success = false; 
        ROS_WARN_STREAM("[IROCFleetManager]: "<< "Call for robot \"" << req.robot_name << "\" was not successful with message: " << resp.message << "\n");
        ss << "Call for robot \"" << req.robot_name << "\" was not successful with message: " << resp.message << "\n";
      } else {
        ss << "Call successful.\n";
      }
    } else if (req.type == "pause") {
      ROS_INFO_STREAM("[IROCFleetManager]: Calling mission pausing for robot: " << req.robot_name << ".");
      const auto resp = callService<std_srvs::Trigger>(rh_ptr->sc_robot_pausing);
      if (!resp.success) { 
        success = false; 
        ROS_WARN_STREAM("[IROCFleetManager]: "<< "Call for robot \"" << req.robot_name << "\" was not successful with message: " << resp.message << "\n");
        ss << "Call for robot \"" << req.robot_name << "\" was not successful with message: " << resp.message << "\n";
      } else {
        ss << "Call successful.\n";
      }
    } else if (req.type == "stop") {
      ROS_INFO_STREAM("[IROCFleetManager]: Calling mission stopping.");
      const auto action_client_state = rh_ptr->action_client_ptr->getState(); 
      if (action_client_state.isDone()) {
        ss << "robot \"" << rh_ptr->robot_name << "\" mission done, skipping\n";
        success = false;
        ROS_WARN_STREAM("[IROCFleetManager]: Robot \"" << rh_ptr->robot_name << "\" mission done. Skipping.");
      } else {
        ss << "Call successful.\n";
        ROS_INFO_STREAM("[IROCFleetManager]: Cancelling \"" << rh_ptr->robot_name << "\" mission.");
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
  res.success = success;
  res.message = ss.str();
  if (res.success){
    ROS_INFO_STREAM("[IROCFleetManager]: Succesfully processed the  "<< req.type<<" request for "<< req.robot_name <<".");
  }else{
    ROS_WARN("[IROCFleetManager]: Failure: %s", res.message.c_str());
  };

  active_mission_change_ = false;
  return true;
}

//}

// --------------------------------------------------------------
// |                  acition client callbacks                  |
// --------------------------------------------------------------

/* waypointMissionActiveCallback //{ */

void IROCFleetManager::waypointMissionActiveCallback(const std::string& robot_name) {
  ROS_INFO_STREAM("[IROCFleetManager]: Action server on robot " << robot_name << " is processing the goal.");
}

//}

/* waypointMissionDoneCallback //{ */

void IROCFleetManager::waypointMissionDoneCallback(const SimpleClientGoalState& state, const mrs_mission_manager::waypointMissionResultConstPtr& result,
    const std::string& robot_name) {

  if (!active_mission_) {
    return;
  }

  if (result == NULL) 
    ROS_WARN("[IROCFleetManager]: Probably mission_manager died, and action server connection was lost!, reconnection is not currently handled, if mission manager was restarted need to upload a new mission!");

  if (result->success) {
    ROS_INFO_STREAM("[IROCFleetManager]: Action server on robot " << robot_name << " finished with state: \"" << state.toString() << "\". Result message is: \""
          << result->message << "\"");
  } else {
    ROS_WARN_STREAM("[IROCFleetManager]: Action server on robot " << robot_name << " finished with state: \"" << state.toString() << "\". Result message is: \""
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

void IROCFleetManager::waypointMissionFeedbackCallback(const mrs_mission_manager::waypointMissionFeedbackConstPtr& feedback, const std::string& robot_name) {
  
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

void IROCFleetManager::actionCallbackGoal() {
  std::scoped_lock  lock(action_server_mutex_);
  boost::shared_ptr<const iroc_fleet_manager::WaypointFleetManagerGoal> new_action_server_goal = mission_management_server_ptr_->acceptNewGoal();
  ROS_INFO_STREAM("[IROCFleetManager]: Action server received a new goal: \n" << *new_action_server_goal);

  if (!is_initialized_) {
    iroc_fleet_manager::WaypointFleetManagerResult action_server_result;
    action_server_result.success = false;
    action_server_result.messages.emplace_back("Not initialized yet");
    ROS_WARN("[IROCFleetManager]: not initialized yet");
    mission_management_server_ptr_->setAborted(action_server_result);
    return;
  }

  //Start each robot action/service clients with mission_manager 
  const auto results = startRobotClients(*new_action_server_goal);

  bool all_success = std::all_of(results.begin(), results.end(),[](const auto& pair){
      return pair.second.success;
      }
  );

  if (!all_success) {
      iroc_fleet_manager::WaypointFleetManagerResult action_server_result;
      for (const auto& result : results) {
        std::stringstream ss;
        if (!result.second.success) {
          ss << result.first << " failed with response: " << result.second.message;
          ROS_WARN_STREAM("[IROCFleetManager]: Failure starting robot clients: " << ss.str());
          action_server_result.messages.emplace_back(ss.str()); 
        }
      }
      action_server_result.success = false;
      mission_management_server_ptr_->setAborted(action_server_result);
      cancelRobotClients(); 
      ROS_INFO("[IROCFleetManager]: Mission Aborted.");
      return;
  }
  ROS_INFO("[IROCFleetManager]: Succesfully sent the goal to robots in mission.");

  active_mission_ = true;
  action_server_goal_ = *new_action_server_goal;
}

//}

/*  actionCallbackPreempt()//{ */

void IROCFleetManager::actionCallbackPreempt() {
  std::scoped_lock lock(action_server_mutex_);
  if (mission_management_server_ptr_->isActive()) {
    if (mission_management_server_ptr_->isNewGoalAvailable()) {
      ROS_INFO("[IROCFleetManager]: Preemption toggled for ActionServer.");
      iroc_fleet_manager::WaypointFleetManagerResult action_server_result;
      action_server_result.success = false;
      action_server_result.messages.emplace_back("Preempted by client");
      ROS_WARN_STREAM("[IROCFleetManager]: Preempted by the client");
      mission_management_server_ptr_->setPreempted(action_server_result);
      cancelRobotClients();
    } else {
      ROS_INFO("[IROCFleetManager]: Cancel toggled for ActionServer.");

      iroc_fleet_manager::WaypointFleetManagerResult action_server_result;
      action_server_result.success = false;
      action_server_result.messages.emplace_back("Mission stopped.");
      active_mission_ = false;
      mission_management_server_ptr_->setAborted(action_server_result);
      cancelRobotClients();
      ROS_INFO("[IROCFleetManager]: Mission stopped.");
    }
  }
}

//}

/* actionPublishFeedback()//{ */

void IROCFleetManager::actionPublishFeedback() {
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
  
    if (mission_management_server_ptr_->isActive()) {
      auto action_server_feedback = processAggregatedFeedbackInfo(robots_feedback);
      mission_management_server_ptr_->publishFeedback(action_server_feedback);
    }
  }
}

//}

// | -------------------- support functions ------------------- |

/* startRobotClients() //{ */

std::map<std::string,IROCFleetManager::result_t> IROCFleetManager::startRobotClients(const ActionServerGoal& goal){
  std::scoped_lock lck(fleet_mission_handlers_.mtx);
  std::map<std::string,IROCFleetManager::result_t> robot_results;
  {
    fleet_mission_handlers_.handlers.reserve(goal.robots.size());
    //Initialize the robots received in the goal request
    for (const auto& robot : goal.robots) {
      bool success = true;
      std::stringstream ss;
      const std::string waypoint_action_client_topic = "/" + robot.name + nh_.resolveName("ac/waypoint_mission");
      auto action_client_ptr                = std::make_unique<MissionManagerClient>(waypoint_action_client_topic, false);

      //Need to wait for server
      if (!action_client_ptr->waitForServer(ros::Duration(5.0))) {
        ROS_WARN("[IROCFleetManager]: Server connection failed for robot %s ", robot.name.c_str());
        ss << "Action server from robot: " + robot.name + " failed to connect. Check the mrs_mission_manager node.\n";
        robot_results[robot.name].message = ss.str();
        robot_results[robot.name].success = false; 
        success = false;
      }

      ROS_INFO("[IROCFleetManager]: Created action client on topic \'ac/waypoint_mission\' -> \'%s\'", waypoint_action_client_topic.c_str());
      MissionManagerActionServerGoal action_goal;
      action_goal.frame_id           = robot.frame_id;
      action_goal.height_id          = robot.height_id; 
      action_goal.terminal_action    = robot.terminal_action; 
      action_goal.points             = robot.points;

      if (!action_client_ptr->isServerConnected()) {
        ss << "Action server from robot: " + robot.name + " is not connected. Check the mrs_mission_manager node.\n";
        ROS_WARN_STREAM("[IROCFleetManager]: Action server from robot :" + robot.name + " is not connected. Check the mrs_mission_manager node.");
        robot_results[robot.name].message = ss.str();
        robot_results[robot.name].success = false; 
        success = false;
      }

      if (!action_client_ptr->getState().isDone()) {
        ss << "Mission on robot: " + robot.name + " already running. Terminate the previous one, or wait until it is finished.\n";
        ROS_WARN_STREAM("[IROCFleetManager]: Mission on robot: " + robot.name + " already running. Terminate the previous one, or wait until it is finished.\n");
        robot_results[robot.name].message = ss.str();
        robot_results[robot.name].success = false; 
        success = false;
      }

      //Seng the goal to robot in mission_manager
      action_client_ptr->sendGoal(
        action_goal, std::bind(&IROCFleetManager::waypointMissionDoneCallback, this, std::placeholders::_1, std::placeholders::_2, robot.name),
        std::bind(&IROCFleetManager::waypointMissionActiveCallback, this, robot.name),
        std::bind(&IROCFleetManager::waypointMissionFeedbackCallback, this, std::placeholders::_1, robot.name));

      //This is important to wait for some time in case the goal was rejected
      //We can replace it to wait while the sate is pending
      ros::Duration(0.5).sleep();

      if (action_client_ptr->getState().isDone()) {
        auto result =  action_client_ptr->getResult();
        ss << result->message;
        ROS_INFO_STREAM("[IROCFleetManagerDebug]: result: " << ss.str());
        robot_results[robot.name].message = ss.str();
        robot_results[robot.name].success = false; 
        success = false;
      }

      if (!success) {
        continue;
      }
      //Save the ros service clients from mission_manager
      const std::string mission_activation_client_topic = "/" + robot.name + nh_.resolveName("svc/mission_activation");
      ros::ServiceClient sc_robot_activation = nh_.serviceClient<std_srvs::Trigger>(mission_activation_client_topic);
      ROS_INFO("[IROCFleetManager]: Created ServiceClient on service \'svc/mission_activation\' -> \'%s\'", sc_robot_activation.getService().c_str());

      const std::string mission_pausing_client_topic = "/" + robot.name + nh_.resolveName("svc/mission_pausing");
      ros::ServiceClient sc_robot_pausing = nh_.serviceClient<std_srvs::Trigger>(mission_pausing_client_topic);
      ROS_INFO("[IROCFleetManager]: Created ServiceClient on service \'svc/mission_pausing\' -> \'%s\'", sc_robot_pausing.getService().c_str());

      robot_mission_handler_t robot_handler;
      robot_handler.robot_name          = robot.name;
      robot_handler.action_client_ptr   = std::move(action_client_ptr); 
      robot_handler.sc_robot_activation = sc_robot_activation;
      robot_handler.sc_robot_pausing    = sc_robot_pausing;
      //Save robot clients in mission handler
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

IROCFleetManager::ActionServerFeedback IROCFleetManager::processAggregatedFeedbackInfo(const std::vector<iroc_fleet_manager::WaypointMissionRobotFeedback>& robots_feedback){

  IROCFleetManager::ActionServerFeedback action_server_feedback;
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

/* processFeedbackMsg() //{ */

std::tuple<std::string, std::string> IROCFleetManager::processFeedbackMsg(){

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
IROCFleetManager::robot_mission_handler_t* IROCFleetManager::findRobotHandler(const std::string& robot_name, fleet_mission_handlers_t& fleet_mission_handlers) {
  for (auto& rh : fleet_mission_handlers.handlers) {
    if (rh.robot_name == robot_name)
      return &rh;
  }
  ROS_WARN("[IROCFleetManager]: Robot handler not found!");
  return nullptr;
}
//}

/* cancelRobotClients() //{ */

void IROCFleetManager::cancelRobotClients(){
  std::scoped_lock lock(fleet_mission_handlers_.mtx);
  for (auto& rh : fleet_mission_handlers_.handlers) {
    const auto action_client_state = rh.action_client_ptr->getState(); 
    if (!action_client_state.isDone()) {
      ROS_WARN_STREAM("[IROCFleetManager]:\"" << rh.robot_name << "\" has an active mission, canceling...");
      rh.action_client_ptr->cancelGoal();
      rh.action_client_ptr->waitForResult(ros::Duration(1.0));
    }
  }
  //Clear the robot handlers
  fleet_mission_handlers_.handlers.clear();
}

//}

/* callService() //{ */

template <typename Svc_T>
IROCFleetManager::result_t IROCFleetManager::callService(ros::ServiceClient& sc, typename Svc_T::Request req) {
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
IROCFleetManager::result_t IROCFleetManager::callService(ros::ServiceClient& sc) {
  return callService<Svc_T>(sc, {});
}

IROCFleetManager::result_t IROCFleetManager::callService(ros::ServiceClient& sc, const bool val) {
  using svc_t = std_srvs::SetBool;
  svc_t::Request req;
  req.data = val;
  return callService<svc_t>(sc, req);
}
//}
}  // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::IROCFleetManager, nodelet::Nodelet);
