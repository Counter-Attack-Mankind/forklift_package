#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/rule_engine.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"
#include "forklift_planner/planner_param.h"

namespace forklift_planner {
namespace multi_vehicle {

class MarkerPublisher {
public:
    MarkerPublisher(ros::NodeHandle& nh, const MapParam& mp,
                    const PlannerParam& pp,
                    const MultiVehicleConfig& cfg);

    // 地图原点+XY轴(标定核对)。public:real_mode 启动时可经 latched 话题先发一次,
    // 不必等所有车动捕就绪(否则 tick 早退、per-tick publish 不跑 → 摆车前看不到轴)。
    void addOriginAxes(visualization_msgs::MarkerArray& arr) const;

    void publish(const std::vector<VehicleAgent>& vehicles,
                 const std::vector<ConflictMarker>& conflicts) const;

private:
    void addPathMarker(visualization_msgs::MarkerArray& arr,
                       const VehicleAgent& v) const;
    void addBodyMarker(visualization_msgs::MarkerArray& arr,
                       const VehicleAgent& v) const;
    void addArrowMarker(visualization_msgs::MarkerArray& arr,
                        const VehicleAgent& v) const;
    void addLabelMarker(visualization_msgs::MarkerArray& arr,
                        const VehicleAgent& v) const;
    void addConflictMarkers(visualization_msgs::MarkerArray& arr,
                            const std::vector<ConflictMarker>& conflicts) const;

    ros::Publisher pub_;
    const MapParam& mp_;
    const PlannerParam& pp_;
    const MultiVehicleConfig& cfg_;
    mutable int last_conflict_marker_count_ = 0;
    mutable int publish_seq_ = 0;
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
