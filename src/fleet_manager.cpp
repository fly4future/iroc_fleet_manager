#include "iroc_fleet_manager/planner.h"
#include "nodelet/nodelet.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
#include <mrs_lib/subscribe_handler.h>

#include <boost/smart_ptr/shared_ptr.hpp>
#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>

#include <pluginlib/class_loader.h>

namespace iroc_fleet_manager {

class PlannerParams {

public:
  PlannerParams(const std::string &address, const std::string &name_space,
                const std::string &action_type);

public:
  std::string address;
  std::string name_space;
  std::string action_type;
};

PlannerParams::PlannerParams(const std::string &address,
                             const std::string &name_space,
                             const std::string &action_type) {

  this->address = address;
  this->name_space = name_space;
  this->action_type = action_type;
}

// using namespace actionlib;
typedef actionlib::SimpleActionClient<iroc_mission_handler::MissionAction>
    MissionHandlerClient;
typedef iroc_mission_handler::MissionGoal MissionHandlerActionServerGoal;

class FleetManager : public nodelet::Nodelet {
public:
  virtual void onInit();

  struct result_t {
    bool success;
    std::string message;
  };

private:
  ros::NodeHandle nh_;
  bool is_initialized_ = false;
  ros::Timer timer_main_;
  ros::Timer timer_feedback_;

  // | ----------------------- ROS service servers ---------------------- |
  ros::ServiceServer ss_change_fleet_mission_state_;
  ros::ServiceServer ss_change_robot_mission_state_;

  std::atomic_bool active_mission_ = false;
  std::atomic_bool active_mission_change_ = false;

  // | --------------- dynamic loading of planners -------------- |

  struct planner_t {
    std::string name;
    PlannerParams params;
    boost::shared_ptr<iroc_fleet_manager::Planner> instance;
    std::mutex mutex_planner_list_;
  };

  struct planners_handler_t {
    std::vector<planner_t> planners;
  } planner_handlers_;

  std::unique_ptr<pluginlib::ClassLoader<iroc_fleet_manager::Planner>>
      planner_loader_; // pluginlib loader of dynamically loaded planners
  std::vector<std::string> _planner_names_; // list of planner names
  std::map<std::string, PlannerParams>
      planners_; // map between planner names and planner params
  std::vector<boost::shared_ptr<iroc_fleet_manager::Planner>>
      planner_list_; // list of planners, routines are callable from this
  std::mutex mutex_planner_list_;

  std::string _initial_planner_name_;
  int _initial_planner_idx_ = 0;
  int active_planner_idx_;

  // | ----------------- mission handler action client stuff ---------------- |

  // Handlers for the interaction with the robot's action clients with
  // MissionHandler
  struct robot_mission_handler_t {
    std::string robot_name;
    std::unique_ptr<MissionHandlerClient> action_client_ptr;
    ros::ServiceClient sc_robot_activation;
    ros::ServiceClient sc_robot_pausing;
    iroc_mission_handler::MissionFeedback current_feedback;
    iroc_mission_handler::MissionResult current_result;
    bool got_result = false;
  };

  struct fleet_mission_handlers_t {
    std::recursive_mutex mtx;
    std::vector<robot_mission_handler_t> handlers;
  } fleet_mission_handlers_;

  std::vector<std::string> lost_robot_names_;

  void timerMain(const ros::TimerEvent &event);
  void timerFeedback(const ros::TimerEvent &event);
};

void FleetManager::onInit() {
  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "FleetManager");

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");

  const auto main_timer_rate =
      param_loader.loadParam2<double>("main_timer_rate");
  const auto feedback_timer_rate =
      param_loader.loadParam2<double>("feedback_timer_rate");
  const auto no_message_timeout =
      param_loader.loadParam2<ros::Duration>("no_message_timeout");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[IROCAutonomyTestManager]: Could not load all parameters!");
    ros::shutdown();
  }

  param_loader.loadParam("initial_planner", _initial_planner_name_);

  // --------------------------------------------------------------
  // |                      load the plugins                      |
  // --------------------------------------------------------------

  param_loader.loadParam("planners", _planner_names_);
  planner_loader_ =
      std::make_unique<pluginlib::ClassLoader<iroc_fleet_manager::Planner>>(
          "iroc_fleet_manager", "iroc_fleet_manager::Planner");

  // for each plugin in the list
  for (int i = 0; i < int(_planner_names_.size()); i++) {
    std::string planner_name = _planner_names_[i];

    // load the plugin parameters
    std::string address;
    std::string name_space;
    std::string action_type;

    param_loader.loadParam(planner_name + "/address", address);
    param_loader.loadParam(planner_name + "/name_space", name_space);
    param_loader.loadParam(planner_name + "/action_type", action_type);

    PlannerParams new_planner(address, name_space, action_type);
    planners_.insert(
        std::pair<std::string, PlannerParams>(planner_name, new_planner));

    try {
      ROS_INFO("[FleetManager]: loading the planner '%s'",
               new_planner.address.c_str());
      planner_list_.push_back(
          planner_loader_->createInstance(new_planner.address.c_str()));
    } catch (pluginlib::CreateClassException &ex1) {
      ROS_ERROR("[FleetManager]: CreateClassException for the planner '%s'",
                new_planner.address.c_str());
      ROS_ERROR("[FleetManager]: Error: %s", ex1.what());
      ros::shutdown();
    } catch (pluginlib::PluginlibException &ex) {
      ROS_ERROR("[FleetManager]: PluginlibException for the planner '%s'",
                new_planner.address.c_str());
      ROS_ERROR("[FleetManager]: Error: %s", ex.what());
      ros::shutdown();
    }
  }

  ROS_INFO("[FleetManager]: planners were loaded");

  for (int i = 0; i < int(planner_list_.size()); i++) {
    try {
      std::map<std::string, PlannerParams>::iterator it;
      it = planners_.find(_planner_names_[i]);

      ROS_INFO("[FleetManager]: initializing the planner '%s'",
               it->second.address.c_str());
      planner_list_[i]->initialize(nh_, _planner_names_[i],
                                   it->second.name_space,
                                   it->second.action_type);

    } catch (std::runtime_error &ex) {
      ROS_ERROR("[FleetManager]: exception caught during planner "
                "initialization: '%s'",
                ex.what());
    }
  }

  ROS_INFO("[FleetManager]: planners were initialized");

  // | ---------- check the existance of initial planner --------- |
  {
    bool check = false;

    for (int i = 0; i < int(_planner_names_.size()); i++) {

      std::string planner_name = _planner_names_[i];

      if (planner_name == _initial_planner_name_) {
        check = true;
        _initial_planner_idx_ = i;
        break;
      }
    }
    if (!check) {
      ROS_ERROR("[FleetManager]: the initial planner (%s) is not within "
                "the loaded planners",
                _initial_planner_name_.c_str());
      ros::shutdown();
    }
  }

  // | ---------- activate the first planner on the list --------- |

  ROS_INFO("[FleetManager]: activating planner with idx %d on the list "
           "(named: %s)",
           _initial_planner_idx_,
           _planner_names_[_initial_planner_idx_].c_str());

  planner_list_[_initial_planner_idx_]->activate();
  active_planner_idx_ = _initial_planner_idx_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh = nh_;
  shopts.node_name = "FleetManager";
  shopts.no_message_timeout = no_message_timeout;
  shopts.threadsafe = true;
  shopts.autostart = true;
  shopts.queue_size = 10;
  shopts.transport_hints = ros::TransportHints().tcpNoDelay();

  // | ------------------------- timers ------------------------- |

  ROS_INFO("[FleetManager]: initialized");
  ROS_INFO("[FleetManager]: --------------------");
}

//}


} // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::FleetManager, nodelet::Nodelet);
