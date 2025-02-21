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
#include <mrs_mission_manager/waypointMissionAction.h>
#include <iroc_mission_management/WaypointMissionManagementAction.h>
#include <unistd.h>
#include <iostream>

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


  // | ---------------------- ROS subscribers --------------------- |
  
  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  ros::Timer timer_feedback_;
  void       timerMain(const ros::TimerEvent& event);
  void       timerFeedback(const ros::TimerEvent& event);

  // | ----------------- action client callbacks ---------------- |

  typedef actionlib::SimpleActionServer<iroc_mission_management::WaypointMissionManagementAction> MissionManagementServer;
  void                                                                              actionCallbackGoal();
  void                                                                              actionCallbackPreempt();
  void                                                                              actionPublishFeedback(void);
  std::unique_ptr<MissionManagementServer>                                          mission_management_server_ptr;

  typedef iroc_mission_management::WaypointMissionManagementGoal ActionServerGoal;
  ActionServerGoal                                               action_server_goal_;
  std::recursive_mutex                                           action_server_mutex_;


  std::vector<std::unique_ptr<MissionManagerClient>> action_clients_ptrs_;
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

  // | ------------------ action server methods ----------------- |
  mission_management_server_ptr = std::make_unique<MissionManagementServer>(nh_, ros::this_node::getName(), false);
  mission_management_server_ptr->registerGoalCallback(boost::bind(&IROCMissionManagement::actionCallbackGoal, this));
  mission_management_server_ptr->registerPreemptCallback(boost::bind(&IROCMissionManagement::actionCallbackPreempt, this));
  mission_management_server_ptr->start();


  /* // | --------------------- action clients --------------------- | */
  //TODO this will be created when receiving a goal
  const std::string waypoint_action_client_topic = nh_.resolveName("ac/waypoint_mission");
  auto action_client_ptr_                = std::make_unique<MissionManagerClient>(waypoint_action_client_topic, false);
  action_clients_ptrs_.push_back(std::move(action_client_ptr_));
  ROS_INFO("[IROCBridge]: Created action client on topic \'ac/waypoint_mission\' -> \'%s\'", waypoint_action_client_topic.c_str());

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
// --------------------------------------------------------------
// |                  acition client callbacks                  |
// --------------------------------------------------------------

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

  if (mission_management_server_ptr->isActive()) {
    iroc_mission_management::WaypointMissionManagementFeedback action_server_feedback;
    action_server_feedback.info.message = "I am active";
    mission_management_server_ptr->publishFeedback(action_server_feedback);
  }
}

//}

// --------------------------------------------------------------
// |                     callbacks                              |
// --------------------------------------------------------------

}  // namespace iroc_mission_management

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_mission_management::IROCMissionManagement, nodelet::Nodelet);
