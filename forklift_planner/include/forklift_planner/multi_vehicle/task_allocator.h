#pragma once

#include <array>
#include <memory>
#include <vector>

#include "forklift_map/forklift_map.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"
#include "forklift_planner/path_generator.h"
#include "forklift_planner/planner_param.h"

namespace forklift_planner {
namespace multi_vehicle {

enum class TaskRejectReason {
    NONE,
    SAME_SLOT,
    EMPTY_PATH,
    CURVATURE_DISCONTINUITY,
    FOOTPRINT_OUT_OF_BOUNDS,
    SHELF_COLLISION,
    KINK,
};

struct TaskPlanCache {
    bool valid = false;
    TaskRejectReason reject_reason = TaskRejectReason::EMPTY_PATH;
    RoughPath path;
    PathGenerationInfo info;
};

class TaskAllocator {
public:
    TaskAllocator(const MapParam& mp, const PlannerParam& pp,
                  const MultiVehicleConfig& cfg,
                  const ForkliftMap& map, PathGenerator& generator);

    void buildCache();
    bool assignNextTask(VehicleAgent& vehicle,
                        const std::vector<VehicleAgent>& all);

    // 前瞻仿真用:快照/恢复分配器持久计数器,使「克隆-空跑」里的再派活不污染真实分配状态。
    struct AllocSnapshot { std::vector<int> target, edge, row; };
    AllocSnapshot snapshot() const {
        return AllocSnapshot{target_visit_counts_, edge_visit_counts_, row_visit_counts_};
    }
    void restore(const AllocSnapshot& s) {
        target_visit_counts_ = s.target;
        edge_visit_counts_ = s.edge;
        row_visit_counts_ = s.row;
    }
    // 死锁恢复(C):给一辆卡死车从其「当前实际位姿」重规划到一个空库位,使其驶离争用区。
    // 不倒车;成功置新 track 并返回 true。失败(无可行目标/路径)返回 false。
    bool replanFromPose(VehicleAgent& vehicle,
                        const std::vector<VehicleAgent>& all);
    const char* rejectReasonName(TaskRejectReason reason) const;

    bool hasValidOutbound(int slot) const;

    // 简单测试版:该库位是否至少有一个「全程前进(无 REVERSE 段=无尖点)」的可达目标。
    // 用于 initAgents 起点筛选,保证每车都能一把开进某个库位。
    bool hasForwardTarget(int slot) const;
    void forwardTargets(int slot, std::vector<int>& out,
                        std::vector<double>* out_len = nullptr) const;
    // 该库位「无预约下最近的全程前进目标」+ 路径长(诊断/预设挑对用),无则返回 -1。
    int nearestForwardTargetLen(const VehicleAgent& vehicle,
                                const std::vector<VehicleAgent>& all,
                                double& out_len) const;

private:
    TaskPlanCache makeTaskPlan(int src_id, int target_id) const;
    TaskPlanCache makeTaskPlanFromPose(const VehicleAgent& vehicle, int target_id) const;
    TaskRejectReason validatePath(const RoughPath& path,
                                  const PathGenerationInfo& info,
                                  const Slot& src,
                                  const Slot& target) const;
    TaskRejectReason validatePose(const RoughWp& pose, const Slot& src,
                                  const Slot& target) const;

    bool poseInSlotSweep(const RoughWp& pose, const Slot& slot) const;
    bool footprintCollidesWithShelf(const RoughWp& pose,
                                    const Slot& src,
                                    const Slot& target) const;
    RoughWp interpolatePose(const RoughWp& a, const RoughWp& b,
                            double ratio) const;

    const TaskPlanCache& cacheAt(int src, int target) const;
    TaskPlanCache& cacheAt(int src, int target);
    bool planAvailable(const VehicleAgent& vehicle, int target,
                       bool require_no_arc) const;
    int chooseNextTarget(const VehicleAgent& vehicle,
                         const std::vector<VehicleAgent>& all,
                         bool require_no_arc) const;
    // 简单测试版选靶:返回「路径最短且全程前进(无 REVERSE 段=无尖点)」的可达目标,无则 -1。
    int chooseNearestForwardTarget(const VehicleAgent& vehicle,
                                   const std::vector<VehicleAgent>& all) const;
    bool tryPlan(VehicleAgent& vehicle, int target, bool require_no_arc);
    void rememberTask(VehicleAgent& vehicle, int target);
    bool containsRecent(const std::vector<int>& values, int value) const;
    double deterministicJitter(const VehicleAgent& vehicle, int target) const;
    int activeTargetCount(const VehicleAgent& vehicle,
                          const std::vector<VehicleAgent>& all,
                          int target) const;
    RoughWp vehicleCurrentPose(const VehicleAgent& v) const;
    bool slotReservedByOther(const VehicleAgent& vehicle,
                             const std::vector<VehicleAgent>& all,
                             int slot) const;

    const MapParam& mp_;
    const PlannerParam& pp_;
    const MultiVehicleConfig& cfg_;
    const ForkliftMap& map_;
    PathGenerator& generator_;

    std::vector<TaskPlanCache> task_cache_;
    std::vector<int> target_visit_counts_;
    std::vector<int> edge_visit_counts_;
    std::vector<int> row_visit_counts_;
    std::vector<int> outbound_valid_counts_;
    std::vector<int> outbound_cross_row_counts_;
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
