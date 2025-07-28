#include "iroc_fleet_manager/planner.h"
#include "nodelet/nodelet.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>

#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>

#include <pluginlib/class_loader.h>

namespace iroc_fleet_manager {

class PlannerParams {

public:
  PlannerParams(const std::string &address, const std::string &name_space);

public:
  std::string address;
  std::string name_space;
};

PlannerParams::PlannerParams(const std::string &address,
                             const std::string &name_space) {

  this->address = address;
  this->name_space = name_space;
}

// using namespace actionlib;
typedef actionlib::SimpleActionClient<iroc_mission_handler::MissionAction>
    MissionHandlerClient;
typedef iroc_mission_handler::MissionGoal MissionHandlerActionServerGoal;

class FleetManager : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

  // | --------------- dynamic loading of planners -------------- |

  std::unique_ptr<pluginlib::ClassLoader<iroc_fleet_manager::Planner>>
      planner_loader_; // pluginlib loader of dynamically loaded planners
  std::vector<std::string> _planner_names_; // list of planner names
                                            //
  std::map<std::string, PlannerParams>
      planners_; // map between planner names and planner params
  std::vector<boost::shared_ptr<iroc_fleet_manager::Planner>>
      planner_list_; // list of planners, routines are callable from this
  std::mutex mutex_planner_list_;

  std::string _initial_planner_name_;
  int _initial_planner_idx_ = 0;
  int active_planner_idx_;
};

void FleetManager::onInit() {
  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "FleetManager");

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

    param_loader.loadParam(planner_name + "/address", address);
    param_loader.loadParam(planner_name + "/name_space", name_space);

    PlannerParams new_planner(address, name_space);
    planners_.insert(
        std::pair<std::string, PlannerParams>(planner_name, new_planner));

    try {
      ROS_INFO("[FleetManager]: loading the plugin '%s'",
               new_planner.address.c_str());
      planner_list_.push_back(
          planner_loader_->createInstance(new_planner.address.c_str()));
    } catch (pluginlib::CreateClassException &ex1) {
      ROS_ERROR("[FleetManager]: CreateClassException for the plugin '%s'",
                new_planner.address.c_str());
      ROS_ERROR("[FleetManager]: Error: %s", ex1.what());
      ros::shutdown();
    } catch (pluginlib::PluginlibException &ex) {
      ROS_ERROR("[FleetManager]: PluginlibException for the plugin '%s'",
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

      ROS_INFO("[FleetManager]: initializing the plugin '%s'",
               it->second.address.c_str());
      planner_list_[i]->initialize(nh_, _planner_names_[i],
                                   it->second.name_space);
    } catch (std::runtime_error &ex) {
      ROS_ERROR("[FleetManager]: exception caught during plugin "
                "initialization: '%s'",
                ex.what());
    }
  }

  ROS_INFO("[FleetManager]: plugins were initialized");

  // | ---------- check the existance of initial plugin --------- |
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
      ROS_ERROR("[FleetManager]: the initial plugin (%s) is not within "
                "the loaded plugins",
                _initial_planner_name_.c_str());
      ros::shutdown();
    }
  }

  // | ---------- activate the first plugin on the list --------- |

  ROS_INFO("[FleetManager]: activating plugin with idx %d on the list "
           "(named: %s)",
           _initial_planner_idx_, _planner_names_[_initial_planner_idx_].c_str());

  planner_list_[_initial_planner_idx_]->activate();
  active_planner_idx_ = _initial_planner_idx_;

  ROS_INFO("[FleetManager]: initialized");
  ROS_INFO("[FleetManager]: --------------------");
}

} // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::FleetManager, nodelet::Nodelet);
