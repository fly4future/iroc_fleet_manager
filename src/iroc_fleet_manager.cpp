/* includes //{ */

#include <iroc_fleet_manager/base_fleet_manager.h>

//}

namespace iroc_fleet_manager
{

 typedef actionlib::SimpleActionServer<iroc_fleet_manager::WaypointFleetManagerAction> WaypointFleetManagerServer;

/* class IROCFleetManager //{ */

class IROCFleetManager : public iroc_fleet_manager::BaseFleetManager<WaypointFleetManagerServer, iroc_fleet_manager::WaypointFleetManagerGoal, iroc_fleet_manager::WaypointFleetManagerResult, iroc_fleet_manager::WaypointFleetManagerFeedback> {
public:

  std::vector<iroc_fleet_manager::WaypointMissionRobot> processGoal(const iroc_fleet_manager::WaypointFleetManagerGoal& goal) const override;

};
//}

}  // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::IROCFleetManager, nodelet::Nodelet);
