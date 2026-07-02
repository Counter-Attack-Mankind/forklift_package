#pragma once

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/traffic_resource.h"
#include "forklift_planner/multi_vehicle/traffic_resource_map.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace multi_vehicle {

struct ConflictMarker {
    double x = 0.0;
    double y = 0.0;
    double t = 0.0;
    int vehicle_a = -1;
    int vehicle_b = -1;
};

class RuleEngine {
public:
    RuleEngine(const MapParam& mp, const MultiVehicleConfig& cfg);

    // 接入资源地图(Phase 2:资源仲裁需要它把路径映射到资源占用)。
    void setResourceMap(const TrafficResourceMap* m) { resmap_ = m; }

    void decide(std::vector<VehicleAgent>& vehicles, double dt);
    double speedForAction(VehicleAction action) const;

    // 前瞻仿真用:快照/恢复跨周期持久状态,使「克隆-空跑」忠实复现真实协调(确定性⇒预测准)。
    struct SimSnapshot {
        std::map<std::pair<int, int>, int> commit_owner;
        std::set<std::pair<int, int>> following_pairs;
        ResourceTokenTable tokens;
        double now = 0.0;
    };
    SimSnapshot snapshot() const {
        return SimSnapshot{commit_owner_, following_pairs_, tokens_, now_};
    }
    void restore(const SimSnapshot& s) {
        commit_owner_ = s.commit_owner;
        following_pairs_ = s.following_pairs;
        tokens_ = s.tokens;
        now_ = s.now;
    }
    const std::vector<ConflictMarker>& conflicts() const { return conflicts_; }

    int priorityWinner(const VehicleAgent& a, const VehicleAgent& b) const;

    int unifiedPriority(const VehicleAgent& a, const VehicleAgent& b) const;

    // 诊断:打印一对车的冲突区全貌(各块 se/sx、same_dir、committed、停止线、owner 预约、
    // 是否在 following_pairs_)。供无头批处理在碰撞现场调用,定位"该不该门控/谁越线"。
    void debugDumpConflict(const VehicleAgent& a, const VehicleAgent& b) const;

private:
    struct ConflictZone {
        double s_self_enter = 0.0;   // arc entry on self's path
        double s_other_enter = 0.0;  // arc entry on other's path
        double s_self_exit = 0.0;    // arc exit on self's path
        double s_other_exit = 0.0;   // arc exit on other's path
        double x = 0.0;
        double y = 0.0;
        // 块内两路径是否「同向」(正对角带=同车道跟车;否则交叉/对向)。由静态几何在
        // 块中点测两路径行进朝向算定 → 稳定不随当前位姿闪烁(对称,与 self/other 朝向无关)。
        bool same_dir = false;
    };

    struct OccupancyInterval {
        bool occupies = false;
        double enter = 0.0;
        double exit = 0.0;
    };

    // 当前位置下、属于 (self,other) 的有效冲突块:取缓存的静态 C_ij(见
    // conflictBlocksCanonical),按 self/other 朝向取用,并裁掉任一方已完全清出的块、
    // 把入口夹到各自车尾起点——与历史"逐拍沿剩余路径扫描"的产物在同一离散精度下等价。
    std::vector<ConflictZone> findConflictZones(const VehicleAgent& self,
                                                const VehicleAgent& other) const;
    // 在两条「完全固定」路径的整段 [0,L]×[0,L] 上一次算定的静态冲突集 C_ij(与
    // 时间/速度/朝向/当前位置无关)。仅几何,重活在此。
    std::vector<ConflictZone> computeConflictZonesFull(
        const VehicleAgent& self, const VehicleAgent& other) const;
    // 取 (lo,hi)(lo.id<hi.id)的静态 C_ij,按 path_gen 缓存;任一方换路径才重算。
    // 返回的块以 self=lo、other=hi 的朝向存储(s_self_*=lo 路径弧长)。
    const std::vector<ConflictZone>& conflictBlocksCanonical(
        const VehicleAgent& lo, const VehicleAgent& hi) const;
    OccupancyInterval occupancyInterval(const VehicleAgent& v,
                                        VehicleAction action,
                                        double zone_enter_s,
                                        double zone_exit_s) const;
    bool intervalsOverlap(const OccupancyInterval& a,
                          const OccupancyInterval& b) const;
    double timeToReachS(const VehicleAgent& v, VehicleAction action,
                        double target_s) const;
    double predictedTravelDistance(const VehicleAgent& v,
                                   VehicleAction action,
                                   double t) const;
    void applyActionRequest(VehicleAgent& v, VehicleAction action,
                            const std::string& reason, int blocker_id = -1);
    void resolvePairwiseConflicts(std::vector<VehicleAgent>& vehicles, double dt);
    void resolveFollowing(std::vector<VehicleAgent>& vehicles);
    // 普适前向净空护栏:任何车若沿自身固定路径在自己刹车距离内会撞上另一辆车的当前
    // 车身,提前 STOP(留余量、干净对停)。堵死 following/crossing 分类接缝处「两套都
    // 没刹→NOMINAL 直撞停着的车→十字楔死」的漏洞。破环车豁免。比硬护栏早刹留余量。
    void enforceForwardClearance(std::vector<VehicleAgent>& vehicles);
    void resolveTargetSlotOccupancy(std::vector<VehicleAgent>& vehicles);
    void applyRequestedActions(std::vector<VehicleAgent>& vehicles, double dt);
    void breakDeadlockCycles(std::vector<VehicleAgent>& vehicles);
    // Phase 4(§9/§11.11):用上一周期的等待边(blocker_id)建等待图,检测环,
    // 按 §9 顺序选破环车并置 deadlock_breaker(资源/优先级层据此给它临时最高优先级)。
    void resolveDeadlock(std::vector<VehicleAgent>& vehicles, double dt);
    // Phase 2:track 变化时,用资源地图重算每车路径的资源占用区间(缓存到 agent)。
    void refreshResourceSpans(std::vector<VehicleAgent>& vehicles);
    // Phase 2.2:对 capacity=1 互斥资源(窄道/路口/货位口)按统一优先级发令牌,
    // winner 通过、其余在上游停止线让行;令牌持有到车身驶出才释放(§11)。
    void arbitrateResources(std::vector<VehicleAgent>& vehicles, double dt);

    const MapParam& mp_;
    const MultiVehicleConfig& cfg_;
    std::vector<ConflictMarker> conflicts_;

    // 资源-时间窗调度的预约闭锁表（跨决策周期持久）。
    // key = 两车 id 的有序对 {min,max}；value = 当前持有该共享冲突区路权(预约)
    // 的车辆 id。一旦按规则裁决出 winner 并写入此表，让行方持续让行，直到 owner
    // 车身(含后伸)完全驶出该共享区才释放——杜绝优先级随 wait_time 在过程中翻转
    // 而把让行车提前放行导致的撞车。
    std::map<std::pair<int, int>, int> commit_owner_;

    // 真·同向同车道跟车对(由 resolveFollowing 每周期重算认定;key={min,max} id)。
    // 唯一事实源:resolvePairwiseConflicts 仅当某对在此集合内才跳过(交 resolveFollowing
    // 管纵向跟距),否则一律由原子门门控。保证「被跳过 ⟺ 被 following 接管」,杜绝两层
    // 判据不一致留下的缝隙(V1↔V4 / V0↔V7 / V1↔V5 头对头/汇入对撞的根因)。
    std::set<std::pair<int, int>> following_pairs_;

    // 静态冲突集 C_ij 缓存(协调图第一步)。key={lo.id,hi.id};块以 self=lo 朝向存储。
    // gen_lo/gen_hi 记录算定时两车的 path_gen;任一方 path_gen 变(换了固定路径)即失效
    // 重算。把"逐拍沿固定路径全程精扫"摊销成"每对路径只算一次",冲突几何恒定可见、不抖动。
    struct ConflictCacheEntry {
        int gen_lo = -1;
        int gen_hi = -1;
        std::vector<ConflictZone> blocks;  // canonical: s_self_*=lo, s_other_*=hi
    };
    mutable std::map<std::pair<int, int>, ConflictCacheEntry> conflict_cache_;

    // Phase 2 资源模型:资源地图(只读)+ 资源令牌表(跨周期持久,§11.10)。
    const TrafficResourceMap* resmap_ = nullptr;
    ResourceTokenTable tokens_;
    double now_ = 0.0;  // 内部仿真时钟(每 decide 累加 dt),供令牌防抖/超时用
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
