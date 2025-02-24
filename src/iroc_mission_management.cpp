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
#include <iroc_mission_management/WaypointMissionManagementAction.h>
#include <iroc_mission_management/WaypointMissionRobotFeedback.h>
#include <unistd.h>
#include <iostream>
#include <numeric>

//}

namespace iroc_mission_management
{

using namespace actionlib;

typedef SimpleActionClient<mrs_mission_manager::waypointMissionAction> MissionManagerClient;
typedef mrs_mission_manager::waypointMissionGoal                       MissionManagerActionServerGoal;

/* class IROCMissionManagement //{ */

class IROCMissionManagement : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool is_initialized_ = false;

  struct result_t
  {
    bool        success;
    std::string message;
  };

  // | ----------------------- ROS servers ---------------------- |
  ros::ServiceServer ss_change_mission_state;
  bool changeMissionStateCallback(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);

  // | ----------------------- main timer ----------------------- |
  ros::Timer timer_main_;
  ros::Timer timer_feedback_;
  void       timerMain(const ros::TimerEvent& event);
  void       timerFeedback(const ros::TimerEvent& event);

  // | ----------------- mission management action server stuff  ---------------- |

  typedef actionlib::SimpleActionServer<iroc_mission_management::WaypointMissionManagementAction> MissionManagementServer;

  void                                                           actionCallbackGoal();
  void                                                           actionCallbackPreempt();
  void                                                           actionPublishFeedback(void);
  std::unique_ptr<MissionManagementServer>                       mission_management_server_ptr;
  typedef iroc_mission_management::WaypointMissionManagementGoal ActionServerGoal;
  typedef iroc_mission_management::WaypointMissionManagementFeedback ActionServerFeedback;
  ActionServerGoal                                               action_server_goal_;
  std::recursive_mutex                                           action_server_mutex_;


  // | ----------------- mission manager action client stuff ---------------- |
  std::vector<ros::ServiceClient> sc_robots_activation_;   
  std::vector<std::unique_ptr<MissionManagerClient>> action_clients_ptrs_;
  
  void waypointMissionActiveCallback(const std::string& robot_name);
  void waypointMissionDoneCallback(const SimpleClientGoalState& state, const mrs_mission_manager::waypointMissionResultConstPtr& result,
                                   const std::string& robot_name);
  void waypointMissionFeedbackCallback(const mrs_mission_manager::waypointMissionFeedbackConstPtr& result, const std::string& robot_name);

  //Mission feedback
  std::map<std::string, mrs_mission_manager::waypointMissionFeedback> aggregated_feedback_;
  std::mutex feedback_mutex_;

  // | ------------------ Additional functions ------------------ |
  result_t startActionClients(const ActionServerGoal& goal);
  ActionServerFeedback processAggregatedFeedbackInfo(const std::vector<iroc_mission_management::WaypointMissionRobotFeedback>& robots_feedback);
};
//}

/* onInit() //{ */

void IROCMissionManagement::onInit() {

  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  /* load parameters */
  mrs_lib::ParamLoader param_loader(nh_, "IROCMissionManagement");

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
    ROS_ERROR("[IROCMissionManagement]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "IROCMissionManagement";
  shopts.no_message_timeout = no_message_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  // | ------------------------- timers ------------------------- |

  timer_main_     = nh_.createTimer(ros::Rate(main_timer_rate), &IROCMissionManagement::timerMain, this);
  timer_feedback_ = nh_.createTimer(ros::Rate(feedback_timer_rate), &IROCMissionManagement::timerFeedback, this);


  // | --------------------- service clients -------------------- |
  
  // | --------------------- service servers -------------------- |

  ss_change_mission_state = nh_.advertiseService("svc_server/mission_activation", &IROCMissionManagement::changeMissionStateCallback, this);
  ROS_INFO("[IROCBridge]: Created ServiceServer on service \'svc_server/mission_activation\' -> \'%s\'", ss_change_mission_state.getService().c_str());

  // | ------------------ action server methods ----------------- |
  mission_management_server_ptr = std::make_unique<MissionManagementServer>(nh_, ros::this_node::getName(), false);
  mission_management_server_ptr->registerGoalCallback(boost::bind(&IROCMissionManagement::actionCallbackGoal, this));
  mission_management_server_ptr->registerPreemptCallback(boost::bind(&IROCMissionManagement::actionCallbackPreempt, this));
  mission_management_server_ptr->start();

  // | --------------------- finish the init -------------------- |

  ROS_INFO("[IROCMissionManagement]: initialized");
  ROS_INFO("[IROCMissionManagement]: --------------------");

  is_initialized_ = true;

}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void IROCMissionManagement::timerMain([[maybe_unused]] const ros::TimerEvent& event) {
  
}

//}

/* timerFeedback() //{ */
void IROCMissionManagement::timerFeedback([[maybe_unused]] const ros::TimerEvent& event) {
  if (!is_initialized_) {
    ROS_WARN_THROTTLE(1, "[MissionManager]: Waiting for nodelet initialization");
    return;
  }
  actionPublishFeedback();
}
//}

// | ----------------- service server callback ---------------- |

/*  changeMissionStateCallback()//{ */

bool IROCMissionManagement::changeMissionStateCallback(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res) {
  std::scoped_lock lock(action_server_mutex_);
  ROS_INFO_STREAM("[IROCMissionManagement]: Received mission activation request");

  if (mission_management_server_ptr->isActive()) {
    if (req.value == "start") {
      ROS_INFO_STREAM_THROTTLE(1.0, "Calling mission activation.");
      /* for (const auto& robot_name : robot_names) { */
      /*   auto* rh_ptr = findRobotHandler(robot_name, robot_handlers_); */
      /*   if (rh_ptr != nullptr) { */
      /*     const auto resp = callService<std_srvs::Trigger>(rh_ptr->sc_mission_activation); */
      /*     if (!resp.success) { */
      /*       ss << "Call for robot \"" << robot_name << "\" was not successful with message: " << resp.message << "\n"; */
      /*     } */
      /*   } else { */
      /*     ss << "robot " << robot_name << " not found, skipping\n"; */
      /*     ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot " << robot_name << " not found. Skipping."); */
      /*   } */
      /* } */ 
    } else {
    res.success = false;
    res.message = "No active mission.";
    ROS_WARN_THROTTLE(1.0, "[MissionManager]: %s", res.message.c_str());
    }
  }
  return true;
}

//}

// --------------------------------------------------------------
// |                  acition client callbacks                  |
// --------------------------------------------------------------

/* waypointMissionActiveCallback //{ */

void IROCMissionManagement::waypointMissionActiveCallback(const std::string& robot_name) {
  ROS_INFO_STREAM("[IROCMissionManagement]: Action server on robot " << robot_name << " is processing the goal.");
}

//}

/* waypointMissionDoneCallback //{ */

void IROCMissionManagement::waypointMissionDoneCallback(const SimpleClientGoalState& state, const mrs_mission_manager::waypointMissionResultConstPtr& result,
    const std::string& robot_name) {
  if (result == NULL) {
    ROS_WARN("[IROCMissionManagement]: Probably mission_manager died, and action server connection was lost!, reconnection is not currently handled, if mission manager was restarted need to upload a new mission!");
    /* const json json_msg = { */
    /*   {"robot_name", robot_name}, */
    /*   {"mission_result", "Mission manager died in ongoing mission"}, */
    /*   {"mission_success", false}, */
    /* }; */
    /* sendJsonMessage("WaypointMissionDone", json_msg); */
  } else {
    if (result->success) {
      ROS_INFO_STREAM("[IROCMissionManagement]: Action server on robot " << robot_name << " finished with state: \"" << state.toString() << "\". Result message is: \""
          << result->message << "\"");
    } else {
      ROS_ERROR_STREAM("[IROCMissionManagement]: Action server on robot " << robot_name << " finished with state: \"" << state.toString() << "\". Result message is: \""
          << result->message << "\"");
    }

    /* const json json_msg = { */
    /*   {"robot_name", robot_name}, */
    /*   {"mission_result", result->message}, */
    /*   {"mission_success", result->success}, */
    /* }; */
    /* sendJsonMessage("WaypointMissionDone", json_msg); */
  }
}

//}

/* waypointMissionFeedbackCallback //{ */

void IROCMissionManagement::waypointMissionFeedbackCallback(const mrs_mission_manager::waypointMissionFeedbackConstPtr& feedback, const std::string& robot_name) {

  {
    std::scoped_lock lck(feedback_mutex_);
    aggregated_feedback_[robot_name] = *feedback; 
  }

  /* ROS_INFO_STREAM("[IROCMissionManagement]: Feedback from " << robot_name << " action: \"" << feedback->message << "\"" << " goal_idx: " << feedback->goal_idx */
  /*                                                << " distance_to_closest_goal: " << feedback->distance_to_closest_goal << " goal_estimated_arrival_time: " */
  /*                                                << feedback->goal_estimated_arrival_time << " goal_progress: " << feedback->goal_progress */
  /*                                                << " distance_to_finish: " << feedback->distance_to_finish << " finish_estimated_arrival_time: " */ 
  /*                                                << feedback->finish_estimated_arrival_time  << " mission_progress: " << feedback->mission_progress); */
  
  }

//}

// | ---------------------- action server callbacks --------------------- |

/*  actionCallbackGoal()//{ */

void IROCMissionManagement::actionCallbackGoal() {
  std::scoped_lock                                                  lock(action_server_mutex_);
  boost::shared_ptr<const iroc_mission_management::WaypointMissionManagementGoal> new_action_server_goal = mission_management_server_ptr->acceptNewGoal();
  ROS_INFO_STREAM("[IROCMissionManagement]: Action server received a new goal: \n" << *new_action_server_goal);
 
  ROS_INFO("[IROCMissionManagement]: ");
  if (!is_initialized_) {
    iroc_mission_management::WaypointMissionManagementResult action_server_result;
    action_server_result.success = false;
    action_server_result.message = "Not initialized yet";
    ROS_ERROR("[IROCMissionManagement]: not initialized yet");
    mission_management_server_ptr->setAborted(action_server_result);
    return;
  }

  const auto result = startActionClients(*new_action_server_goal);

  /* if (result.success) { */

  /*   for (const auto& client : action_clients_ptrs_) { */

  /*     if (!client->isServerConnected()) { */
  /*           /1* ss << "Action server from robot: " + robot.name + " is not connected. Check the mrs_mission_manager node.\n"; *1/ */
  /*           ROS_ERROR_STREAM("[IROCMissionManagement]: Action server from robot : is not connected. Check the mrs_mission_manager node."); */
  /*      } */

  /*      if (!client->getState().isDone()) { */
  /*           /1* ss << "Mission on robot: " + robot.name + " already running. Terminate the previous one, or wait until it is finished.\n"; *1/ */
  /*           ROS_ERROR_STREAM("[IROCMissionManagement]: Mission on robot is already running. Terminate the previous one, or wait until it is finished.\n"); */
  /*      } */
  /*   } */

  /*   /1* MissionManagerActionServerGoal action_goal; *1/ */
  /*   /1* action_goal.frame_id           = robot.frame_id; *1/ */
  /*   /1* action_goal.height_id          = robot.height_id; *1/ */ 
  /*   /1* action_goal.terminal_action    = robot.terminal_action; *1/ */ 
  /*   /1* action_goal.points             = robot.points; *1/ */

     
  /* } */

  /* auto robots = new_action_server_goal->robots; */

  action_server_goal_ = *new_action_server_goal;
}

//}

/*  actionCallbackPreempt()//{ */

void IROCMissionManagement::actionCallbackPreempt() {
  std::scoped_lock lock(action_server_mutex_);
  if (mission_management_server_ptr->isActive()) {

    if (mission_management_server_ptr->isNewGoalAvailable()) {
      ROS_INFO("[IROCMissionManagement]: Preemption toggled for ActionServer.");
      iroc_mission_management::WaypointMissionManagementResult action_server_result;
      action_server_result.success = false;
      action_server_result.message = "Preempted by client";
      ROS_WARN_STREAM("[IROCMissionManagement]: " << action_server_result.message);
      mission_management_server_ptr->setPreempted(action_server_result);

    } else {
      ROS_INFO("[IROCMissionManagement]: Cancel toggled for ActionServer.");

      iroc_mission_management::WaypointMissionManagementResult action_server_result;
      action_server_result.success = false;
      action_server_result.message = "Mission stopped.";
      mission_management_server_ptr->setAborted(action_server_result);
      ROS_INFO("[IROCMissionManagement]: Mission stopped.");

    }
  }
}

//}

/* actionPublishFeedback()//{ */

void IROCMissionManagement::actionPublishFeedback() {
  std::scoped_lock lock(action_server_mutex_);

  //Get the aggregated feedback from the robots in ongoing mission
  std::vector<iroc_mission_management::WaypointMissionRobotFeedback> robots_feedback;
  {
    std::scoped_lock lck(feedback_mutex_);
    
    iroc_mission_management::WaypointMissionRobotFeedback robot_feedback;
    //Fill the robots feedback vector
    for (const auto& [robot_name, fb] : aggregated_feedback_) {
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

  if (mission_management_server_ptr->isActive()) {
    auto action_server_feedback = processAggregatedFeedbackInfo(robots_feedback);
    /* iroc_mission_management::WaypointMissionManagementFeedback action_server_feedback; */
    mission_management_server_ptr->publishFeedback(action_server_feedback);
  }
}

//}

// --------------------------------------------------------------
// |                     callbacks                              |
// --------------------------------------------------------------

// | -------------------- support functions ------------------- |

/* startActionClients() //{ */

IROCMissionManagement::result_t IROCMissionManagement::startActionClients(const ActionServerGoal& goal){

  std::stringstream ss;
  bool success = true;

  for (const auto& robot : goal.robots) {
    const std::string waypoint_action_client_topic = "/" + robot.name + nh_.resolveName("ac/waypoint_mission");
    auto action_client_ptr                = std::make_unique<MissionManagerClient>(waypoint_action_client_topic, false);

    if (!action_client_ptr->waitForServer(ros::Duration(5.0))) {
      ROS_ERROR("[IROCMissionManagement]: Server connection failed for robot %s ", robot.name.c_str());
      success = false;
      continue;
    }

    ROS_INFO("[IROCMissionManagement]: Created action client on topic \'ac/waypoint_mission\' -> \'%s\'", waypoint_action_client_topic.c_str());

    MissionManagerActionServerGoal action_goal;
    action_goal.frame_id           = robot.frame_id;
    action_goal.height_id          = robot.height_id; 
    action_goal.terminal_action    = robot.terminal_action; 
    action_goal.points             = robot.points;

    if (!action_client_ptr->isServerConnected()) {
      ss << "Action server from robot: " + robot.name + " is not connected. Check the mrs_mission_manager node.\n";
      ROS_ERROR_STREAM("[IROCMissionManagement]: Action server from robot :" + robot.name + "is not connected. Check the mrs_mission_manager node.");
      success = false;
    }

    if (!action_client_ptr->getState().isDone()) {
      ss << "Mission on robot: " + robot.name + " already running. Terminate the previous one, or wait until it is finished.\n";
      ROS_ERROR_STREAM("[IROCMissionManagement]: Mission on robot: " + robot.name + " already running. Terminate the previous one, or wait until it is finished.\n");
      success = false;
    }

    action_client_ptr->sendGoal(
      action_goal, std::bind(&IROCMissionManagement::waypointMissionDoneCallback, this, std::placeholders::_1, std::placeholders::_2, robot.name),
      std::bind(&IROCMissionManagement::waypointMissionActiveCallback, this, robot.name),
      std::bind(&IROCMissionManagement::waypointMissionFeedbackCallback, this, std::placeholders::_1, robot.name));

    //Save the action client pointer
    action_clients_ptrs_.emplace_back(std::move(action_client_ptr));

    //Save the ros service client
    const std::string mission_manager_service_client_topic = "/" + robot.name + nh_.resolveName("svc/mission_activation");
    ros::ServiceClient sc_robot_activation = nh_.serviceClient<std_srvs::Trigger>(mission_manager_service_client_topic);
    ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/mission_activation\' -> \'%s\'", sc_robot_activation.getService().c_str());
    sc_robots_activation_.emplace_back(sc_robot_activation);
    
  }

 return {success, "success"};
}

//}

/* processAggregatedFeedbackInfo() //{ */

IROCMissionManagement::ActionServerFeedback IROCMissionManagement::processAggregatedFeedbackInfo(const std::vector<iroc_mission_management::WaypointMissionRobotFeedback>& robots_feedback){

  IROCMissionManagement::ActionServerFeedback action_server_feedback;
  std::vector<std::string> robots_msg;
  std::vector<double> robots_progress;

  for (const auto& rbf : robots_feedback) {
    robots_msg.emplace_back(rbf.message);
    robots_progress.emplace_back(rbf.mission_progress);
  }

  auto mission_progress = std::accumulate(robots_progress.begin(), robots_progress.end(), 0.0) / robots_progress.size();

  action_server_feedback.info.progress = mission_progress;
  action_server_feedback.info.message = "I am active";
  action_server_feedback.info.state = iroc_mission_management::WaypointMissionInfo::STATE_TRAJECTORIES_LOADED;
  action_server_feedback.info.robots_feedback = robots_feedback;

  //TODO manage the mission based on the message received from each robot, validate if all loaded, executing, etc..

  return action_server_feedback; 
}

//}

}  // namespace iroc_mission_management

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_mission_management::IROCMissionManagement, nodelet::Nodelet);
