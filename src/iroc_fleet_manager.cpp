/* includes //{ */

#include <string>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

/* custom msgs of MRS group */
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
  bool is_initialized_               = false;
  std::atomic_bool action_finished_  = false;
  std::atomic_bool active_mission_   = false;
  struct result_t
  {
    bool        success;
    std::string message;
  };

  // | ----------------------- ROS servers ---------------------- |
  ros::ServiceServer ss_change_mission_state_;
  bool changeMissionStateCallback(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);

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
  struct mission_handler_t
  {
    std::string                           robot_name;
    std::unique_ptr<MissionManagerClient> action_client_ptr;
    ros::ServiceClient                    sc_robot_activation;
    ros::ServiceClient                    sc_robot_pausing;
  };

  struct mission_handlers_t
  {
    std::recursive_mutex                                                mtx;
    std::recursive_mutex                                                feedback_mtx;
    std::vector<mission_handler_t>                                      handlers;
    std::map<std::string, mrs_mission_manager::waypointMissionFeedback> aggregated_feedbacks;
    std::map<std::string, mrs_mission_manager::waypointMissionResult>   aggregated_results;
  } mission_handlers_;

  std::vector<ros::ServiceClient> sc_robots_activation_;   
  std::vector<std::unique_ptr<MissionManagerClient>> action_clients_ptrs_;
  
  void waypointMissionActiveCallback(const std::string& robot_name);
  void waypointMissionDoneCallback(const SimpleClientGoalState& state, const mrs_mission_manager::waypointMissionResultConstPtr& result,
                                   const std::string& robot_name);
  void waypointMissionFeedbackCallback(const mrs_mission_manager::waypointMissionFeedbackConstPtr& result, const std::string& robot_name);

  // | ------------------ Additional functions ------------------ |
  result_t startRobotClients(const ActionServerGoal& goal);
  ActionServerFeedback processAggregatedFeedbackInfo(const std::vector<iroc_fleet_manager::WaypointMissionRobotFeedback>& robots_feedback);
  void clearMissionHandlers();
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

  ss_change_mission_state_ = nh_.advertiseService(nh_.resolveName("svc/change_mission_state"), &IROCFleetManager::changeMissionStateCallback, this);
  ROS_INFO("[IROCFleetManager]: Created ServiceServer on service \'svc_server/change_mission_state\' -> \'%s\'", ss_change_mission_state_.getService().c_str());

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
  
  std::scoped_lock lock(action_server_mutex_);
  if (!is_initialized_) {
    ROS_WARN_THROTTLE(1, "[iroc_fleet_manager]: Waiting for nodelet initialization");
    return;
  }

  bool all_success = false;
  {
    if (active_mission_) {
      std::scoped_lock lock(mission_handlers_.mtx);

      //Check if any missions aborted early
      bool any_failure = std::any_of(mission_handlers_.aggregated_results.begin(), mission_handlers_.aggregated_results.end(), [](const auto& pair) {return !pair.second.success;});
      if (any_failure) {
        ROS_WARN("[IROCFleetManager]: Early failure detected, aborting mission.");
        iroc_fleet_manager::WaypointFleetManagerResult action_server_result;
        action_server_result.success = false;
        action_server_result.message = "Early failure detected, aborting mission";
        mission_management_server_ptr_->setAborted(action_server_result);
        cancelRobotClients(); 
        clearMissionHandlers();
        active_mission_ = false;
        return;
      }

      //Finish mission when we get all the robots result
      if (mission_handlers_.aggregated_results.size() == mission_handlers_.handlers.size()) {
        all_success = std::all_of(mission_handlers_.aggregated_results.begin(), mission_handlers_.aggregated_results.end(), [](const auto& pair) { return pair.second.success;});
        if (all_success) {
          ROS_INFO("[IROCFleetManager]: All robots finished successfully, finishing mission."); 
          iroc_fleet_manager::WaypointFleetManagerResult action_server_result;
          action_server_result.success = true;
          action_server_result.message = "All robots finished successfully, mission finished";
          mission_management_server_ptr_->setSucceeded(action_server_result);
        } else {
          ROS_INFO("[IROCFleetManager]: Not all robots finished successfully, finishing mission. ");
          iroc_fleet_manager::WaypointFleetManagerResult action_server_result;
          action_server_result.success = false;
          action_server_result.message = "Not all robots finished successfully, finishing mission";
          mission_management_server_ptr_->setAborted(action_server_result);
        }
          cancelRobotClients(); 
          clearMissionHandlers();
          active_mission_ = false;
      } 
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

  if (active_mission_) {
    actionPublishFeedback();
  }
}
//}

// | ----------------- service server callback ---------------- |

/*  changeMissionStateCallback()//{ */

bool IROCFleetManager::changeMissionStateCallback(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res) {
  std::scoped_lock lock(action_server_mutex_, mission_handlers_.mtx);
  ROS_INFO_STREAM("[IROCFleetManager]: Received mission activation request");
  
  std::stringstream ss;
  bool success = true;
  if (mission_management_server_ptr_->isActive()) {
    if (req.value == "start") {
      ROS_INFO_STREAM_THROTTLE(1.0, "Calling mission activation.");
        for (auto& rh : mission_handlers_.handlers) {
          const auto resp = callService<std_srvs::Trigger>(rh.sc_robot_activation);
          if (!resp.success) { 
              success = false; 
              ss << "Call for robot \"" << rh.robot_name << "\" was not successful with message: " << resp.message << "\n";
          }
        }
    } else if (req.value == "pause") {
       ROS_INFO_STREAM_THROTTLE(1.0, "Calling mission pausing.");
        for (auto& rh : mission_handlers_.handlers) {
          const auto resp = callService<std_srvs::Trigger>(rh.sc_robot_pausing);
          if (!resp.success) { 
              success = false; 
              ss << "Call for robot \"" << rh.robot_name << "\" was not successful with message: " << resp.message << "\n";
          }
        }
    } else if (req.value == "stop") {
      ROS_INFO_STREAM_THROTTLE(1.0, "Calling mission pausing.");
        for (auto& rh : mission_handlers_.handlers) {
          const auto action_client_state = rh.action_client_ptr->getState(); 
          if (action_client_state.isDone()) {
            ss << "robot \"" << rh.robot_name << "\" mission done, skipping\n";
            success = false;
            ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot \"" << rh.robot_name << "\" mission done. Skipping.");
          } else {
            ROS_INFO_STREAM_THROTTLE(1.0, "[IROCBridge]: Cancelling \"" << rh.robot_name << "\" mission.");
            rh.action_client_ptr->cancelGoal();
          }
        }
    } else {
      ROS_WARN_STREAM_THROTTLE(1.0, "Bad type.");
      res.success = false;
      res.message = "Unsupported type";
    }
  
  } else {
   res.success = false;
   res.message = "No active mission.";
   ROS_WARN_THROTTLE(1.0, "[IROCFleetManager]: %s", res.message.c_str());
  }
  res.success = success;
  res.message = ss.str();
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

  std::scoped_lock lock(action_server_mutex_);
  if (result == NULL) 
    ROS_WARN("[IROCFleetManager]: Probably mission_manager died, and action server connection was lost!, reconnection is not currently handled, if mission manager was restarted need to upload a new mission!");

  if (result->success) {
    ROS_INFO_STREAM("[IROCFleetManager]: Action server on robot " << robot_name << " finished with state: \"" << state.toString() << "\". Result message is: \""
          << result->message << "\"");
  } else {
    ROS_ERROR_STREAM("[IROCFleetManager]: Action server on robot " << robot_name << " finished with state: \"" << state.toString() << "\". Result message is: \""
          << result->message << "\"");
  }
  mission_handlers_.aggregated_results[robot_name] = *result;
}

//}

/* waypointMissionFeedbackCallback //{ */

void IROCFleetManager::waypointMissionFeedbackCallback(const mrs_mission_manager::waypointMissionFeedbackConstPtr& feedback, const std::string& robot_name) {
  
  if (!active_mission_) {
    return;
  }

  {
    std::scoped_lock lck(mission_handlers_.feedback_mtx);
    mission_handlers_.aggregated_feedbacks[robot_name] = *feedback; 
  }
}

//}

// | ---------------------- action server callbacks --------------------- |

/*  actionCallbackGoal()//{ */

void IROCFleetManager::actionCallbackGoal() {
  std::scoped_lock                                                  lock(action_server_mutex_);
  boost::shared_ptr<const iroc_fleet_manager::WaypointFleetManagerGoal> new_action_server_goal = mission_management_server_ptr_->acceptNewGoal();
  ROS_INFO_STREAM("[IROCFleetManager]: Action server received a new goal: \n" << *new_action_server_goal);
 
  if (!is_initialized_) {
    iroc_fleet_manager::WaypointFleetManagerResult action_server_result;
    action_server_result.success = false;
    action_server_result.message = "Not initialized yet";
    ROS_ERROR("[IROCFleetManager]: not initialized yet");
    mission_management_server_ptr_->setAborted(action_server_result);
    return;
  }

  //Start each robot action/service clients with mission_manager 
  const auto result = startRobotClients(*new_action_server_goal);

  if (!result.success) {
      iroc_fleet_manager::WaypointFleetManagerResult action_server_result;
      action_server_result.success = false;
      action_server_result.message = result.message;
      ROS_ERROR("[IROCFleetManager]: Failed to start all robot clients,  mission aborted");
      mission_management_server_ptr_->setAborted(action_server_result);
      cancelRobotClients(); 
      clearMissionHandlers();
      return;
  }

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
      action_server_result.message = "Preempted by client";
      ROS_WARN_STREAM("[IROCFleetManager]: " << action_server_result.message);
      mission_management_server_ptr_->setPreempted(action_server_result);
      cancelRobotClients();
      clearMissionHandlers();
    } else {
      ROS_INFO("[IROCFleetManager]: Cancel toggled for ActionServer.");

      iroc_fleet_manager::WaypointFleetManagerResult action_server_result;
      action_server_result.success = false;
      action_server_result.message = "Mission stopped.";
      mission_management_server_ptr_->setAborted(action_server_result);
      cancelRobotClients();
      clearMissionHandlers();
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
    std::scoped_lock lck(mission_handlers_.feedback_mtx);
    iroc_fleet_manager::WaypointMissionRobotFeedback robot_feedback;
    //Fill the robots feedback vector
    for (const auto& [robot_name, fb] : mission_handlers_.aggregated_feedbacks) {
      robot_feedback.name = robot_name;
      robot_feedback.message = fb.message;  
      robot_feedback.goal_idx = fb.goal_idx;
      robot_feedback.distance_to_finish =fb.distance_to_finish;
      robot_feedback.goal_estimated_arrival_time = fb.goal_estimated_arrival_time;
      robot_feedback.mission_progress = fb.mission_progress;
      robot_feedback.distance_to_closest_goal  = fb.distance_to_closest_goal;
      robot_feedback.finish_estimated_arrival_time = fb.finish_estimated_arrival_time;
      robot_feedback.goal_progress = fb.goal_progress;
      robots_feedback.emplace_back(robot_feedback);
    }
  }

  if (mission_management_server_ptr_->isActive()) {
    auto action_server_feedback = processAggregatedFeedbackInfo(robots_feedback);
    mission_management_server_ptr_->publishFeedback(action_server_feedback);
  }
}

//}

// | -------------------- support functions ------------------- |

/* startRobotClients() //{ */

IROCFleetManager::result_t IROCFleetManager::startRobotClients(const ActionServerGoal& goal){

  std::stringstream ss;
  bool success = true;

  std::scoped_lock lck(mission_handlers_.mtx);
  {
    mission_handlers_.handlers.reserve(goal.robots.size());
    //Initialize the robots received in the goal request
    for (const auto& robot : goal.robots) {
      const std::string waypoint_action_client_topic = "/" + robot.name + nh_.resolveName("ac/waypoint_mission");
      auto action_client_ptr                = std::make_unique<MissionManagerClient>(waypoint_action_client_topic, false);

      //Need to wait for server
      if (!action_client_ptr->waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("[IROCFleetManager]: Server connection failed for robot %s ", robot.name.c_str());
        ss << "Action server from robot: " + robot.name + " failed to connect. Check the mrs_mission_manager node.\n";
        success = false;
        return {success, ss.str()};
      }

      ROS_INFO("[IROCFleetManager]: Created action client on topic \'ac/waypoint_mission\' -> \'%s\'", waypoint_action_client_topic.c_str());

      MissionManagerActionServerGoal action_goal;
      action_goal.frame_id           = robot.frame_id;
      action_goal.height_id          = robot.height_id; 
      action_goal.terminal_action    = robot.terminal_action; 
      action_goal.points             = robot.points;

      if (!action_client_ptr->isServerConnected()) {
        ss << "Action server from robot: " + robot.name + " is not connected. Check the mrs_mission_manager node.\n";
        ROS_ERROR_STREAM("[IROCFleetManager]: Action server from robot :" + robot.name + "is not connected. Check the mrs_mission_manager node.");
        success = false;
        return {success, ss.str()};
      }

      if (!action_client_ptr->getState().isDone()) {
        ss << "Mission on robot: " + robot.name + " already running. Terminate the previous one, or wait until it is finished.\n";
        ROS_ERROR_STREAM("[IROCFleetManager]: Mission on robot: " + robot.name + " already running. Terminate the previous one, or wait until it is finished.\n");
        success = false;
        return {success, ss.str()};
      }

      //Seng the goal to robot in mission_manager
      action_client_ptr->sendGoal(
        action_goal, std::bind(&IROCFleetManager::waypointMissionDoneCallback, this, std::placeholders::_1, std::placeholders::_2, robot.name),
        std::bind(&IROCFleetManager::waypointMissionActiveCallback, this, robot.name),
        std::bind(&IROCFleetManager::waypointMissionFeedbackCallback, this, std::placeholders::_1, robot.name));

      //Save the ros service clients from mission_manager
      const std::string mission_activation_client_topic = "/" + robot.name + nh_.resolveName("svc/mission_activation");
      ros::ServiceClient sc_robot_activation = nh_.serviceClient<std_srvs::Trigger>(mission_activation_client_topic);
      ROS_INFO("[IROCFleetManager]: Created ServiceClient on service \'svc/mission_activation\' -> \'%s\'", sc_robot_activation.getService().c_str());

      const std::string mission_pausing_client_topic = "/" + robot.name + nh_.resolveName("svc/mission_pausing");
      ros::ServiceClient sc_robot_pausing = nh_.serviceClient<std_srvs::Trigger>(mission_pausing_client_topic);
      ROS_INFO("[IROCFleetManager]: Created ServiceClient on service \'svc/mission_pausing\' -> \'%s\'", sc_robot_pausing.getService().c_str());

      mission_handler_t robot_handler;
      robot_handler.robot_name          = robot.name;
      robot_handler.action_client_ptr   = std::move(action_client_ptr); 
      robot_handler.sc_robot_activation = sc_robot_activation;
      robot_handler.sc_robot_pausing    = sc_robot_pausing;

      //Save robot clients in mission handler
      mission_handlers_.handlers.emplace_back(std::move(robot_handler));
    }
  }
 ss << "Successfully started robot all robot clients";
 return {success, ss.str()};
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

  //TODO mission message based on the messages from all robots? 
  //TODO manage the mission based on the message received from each robot, validate if all loaded, executing, etc..
  // Checks if ALL messages are exactly "MISSION_LOADED"
  auto all_loaded = std::all_of(
    mission_handlers_.aggregated_feedbacks.begin(),
    mission_handlers_.aggregated_feedbacks.end(),
    [](const auto& pair) { 
        return pair.second.message == "MISSION_LOADED";  
    }
  );

  action_server_feedback.info.progress = mission_progress;
  action_server_feedback.info.message = all_loaded ? "MISSION_LOADED" : "ERROR";
  action_server_feedback.info.state = all_loaded ? iroc_fleet_manager::WaypointMissionInfo::STATE_TRAJECTORIES_LOADED: iroc_fleet_manager::WaypointMissionInfo::STATE_ERROR;
  action_server_feedback.info.robots_feedback = robots_feedback;

  return action_server_feedback; 
}

//}

/* clearMissionHandlers() //{ */

void IROCFleetManager::clearMissionHandlers(){
  std::scoped_lock lock(mission_handlers_.mtx);
  mission_handlers_.handlers.clear();
  mission_handlers_.aggregated_feedbacks.clear();
  mission_handlers_.aggregated_results.clear();
}

//}

/* cancelRobotClients() //{ */

void IROCFleetManager::cancelRobotClients(){
  std::scoped_lock lock(mission_handlers_.mtx);
  for (auto& rh : mission_handlers_.handlers) {
    const auto action_client_state = rh.action_client_ptr->getState(); 
    if (action_client_state.isDone()) {
      ROS_INFO_STREAM_THROTTLE(1.0, "[IROCFleetManager]: Robot \"" << rh.robot_name << "\" mission done. Skipping cancel command.");
    } else {
      ROS_INFO_STREAM_THROTTLE(1.0, "[IROCFleetManager]: Cancelling \"" << rh.robot_name << "\" mission.");
      rh.action_client_ptr->cancelGoal();
    }
  }
}

//}

/* callService() //{ */

template <typename Svc_T>
IROCFleetManager::result_t IROCFleetManager::callService(ros::ServiceClient& sc, typename Svc_T::Request req) {
  typename Svc_T::Response res;
  if (sc.call(req, res)) {
    if (res.success) {
      ROS_INFO_STREAM_THROTTLE(1.0, "Called service \"" << sc.getService() << "\" with response \"" << res.message << "\".");
      return {true, res.message};
    } else {
      ROS_ERROR_STREAM_THROTTLE(1.0, "Called service \"" << sc.getService() << "\" with response \"" << res.message << "\".");
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
