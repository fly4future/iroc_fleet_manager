/* includes //{ */

#include <string>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

/* custom msgs of MRS group */
#include <mrs_mission_manager/waypointMissionAction.h>
#include <unistd.h>
#include <iostream>

//}

namespace iroc_mission_management
{

using namespace actionlib;

typedef SimpleActionClient<mrs_mission_manager::waypointMissionAction> MissionManagerClient;
typedef mrs_mission_manager::waypointMissionGoal                       ActionServerGoal;

/* class IROCMissionManagement //{ */

class IROCMissionManagement : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;


  // | ---------------------- ROS subscribers --------------------- |
  
  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent& event);

  // | ----------------- action client callbacks ---------------- |


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

  // | --------------------- finish the init -------------------- |

  ROS_INFO("[IROCMissionManagement]: initialized");
  ROS_INFO("[IROCMissionManagement]: --------------------");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void IROCMissionManagement::timerMain([[maybe_unused]] const ros::TimerEvent& event) {
  
}

//}

// --------------------------------------------------------------
// |                  acition client callbacks                  |
// --------------------------------------------------------------

// --------------------------------------------------------------
// |                     callbacks                              |
// --------------------------------------------------------------

}  // namespace iroc_mission_management

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_mission_management::IROCMissionManagement, nodelet::Nodelet);
