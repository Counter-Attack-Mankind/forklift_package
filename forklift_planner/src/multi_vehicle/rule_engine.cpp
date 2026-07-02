#include "forklift_planner/multi_vehicle/rule_engine.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

#include "forklift_planner/multi_vehicle/footprint.h"

namespace forklift_planner {
namespace multi_vehicle {

RuleEngine::RuleEngine(const MapParam& mp, const MultiVehicleConfig& cfg)
    : mp_(mp), cfg_(cfg) {}

namespace {

struct OverlapSample {
    double s_self = 0.0;
    double s_other = 0.0;
    double x = 0.0;
    double y = 0.0;
};

}  // namespace

double RuleEngine::speedForAction(VehicleAction action) const {
    switch (action) {
        case VehicleAction::STOP:
            return 0.0;
        case VehicleAction::CREEP:
            return cfg_.nominal_speed * cfg_.creep_ratio;
        case VehicleAction::YIELD:
            return cfg_.nominal_speed * cfg_.yield_ratio;
        case VehicleAction::NOMINAL:
            return cfg_.nominal_speed;
        case VehicleAction::BOOST:
            return cfg_.enable_boost
                       ? std::min(cfg_.max_speed,
                                  cfg_.nominal_speed * cfg_.boost_ratio)
                       : cfg_.nominal_speed;
    }
    return 0.0;
}

int RuleEngine::priorityWinner(const VehicleAgent& a,
                               const VehicleAgent& b) const {
    if (!cfg_.enable_priority_tiebreak) return -1;

    // §9 破环车临时最高优先级:跨层一致——pairwise 也必须让破环车赢,否则它在资源层
    // 赢了令牌、却被 pairwise 摁住,环还是破不了。
    if (a.deadlock_breaker != b.deadlock_breaker) return a.deadlock_breaker ? a.id : b.id;

    // 资源前置约束(非任意优先级,§4/§6/§7):若一车要去的目标库位正被另一车占着
    // (a.target==b.current),占用者必须先清出该位、入库者让行——否则入库者抢先开到
    // 库位口却进不去(位被占),占用者又被它让停在口内,直接死锁。这是 slot 资源依赖,
    // 当任务只指向空/即将空的库位时无环;故置于严格全序之上作 override(若与全序冲突成环
    // =任务分配层的循环占位问题,由第5步环检测断言抓出)。
    {
        const bool a_wants_b_slot = (a.target_slot == b.current_slot);
        const bool b_wants_a_slot = (b.target_slot == a.current_slot);
        if (a_wants_b_slot && !b_wants_a_slot) return b.id;  // b 占用者,先清出
        if (b_wants_a_slot && !a_wants_b_slot) return a.id;  // a 占用者,先清出
    }

    auto motionHeading = [](const VehicleAgent& v) {
        constexpr double kPi = 3.14159265358979323846;
        double heading = v.track.poseAtS(v.path_s).theta;
        if (v.track.typeAtS(v.path_s) == WpType::REVERSE) heading += kPi;
        return heading;
    };

    const double ha = motionHeading(a);
    const double hb = motionHeading(b);
    const double dot =
        std::cos(ha) * std::cos(hb) + std::sin(ha) * std::sin(hb);
    if (dot <= 0.70) return unifiedPriority(a, b);

    const RoughWp pa = a.track.poseAtS(a.path_s);
    const RoughWp pb = b.track.poseAtS(b.path_s);
    const double dx = pb.x - pa.x;
    const double dy = pb.y - pa.y;
    const double lat = std::abs(-dx * std::sin(ha) + dy * std::cos(ha));
    if (lat > mp_.vehicle_width) return unifiedPriority(a, b);

    const double fwd = dx * std::cos(ha) + dy * std::sin(ha);
    if (std::abs(fwd) < 1e-6) return unifiedPriority(a, b);
    return fwd > 0.0 ? b.id : a.id;
}

int RuleEngine::unifiedPriority(const VehicleAgent& a,
                                const VehicleAgent& b) const {
    // 协调图第2步:严格全序 ≺。按可证「反对称+传递+完全」的字典序键比较,id 作唯一终裁。
    // 任意时刻"谁让谁"关系由全序导出 ⇒ 必无环 ⇒ 无死锁(见 草履虫规则_协调图统一架构
    // 设计 §2.3/§5)。
    // 键(越小越优先 / 越先走):
    //   1) 饥饿组(§17):wait_time 超阈值者整体提升一档(有界提升);组内等得越久越先,
    //      以防长饥饿。非饥饿组此分量恒 0、不参与比较。
    //   2) 载货优先;3) 已完成任务少者优先(摊平工作量);4) id 最小(确定性终裁)。
    // 注:原"占用者先清出"的成对 slot 规则会破坏传递性(成对、非全序),已移出本函数,
    //     在 priorityWinner 中作显式**资源前置约束** override(见那里)。原 both-starved 的
    //     1e-3 eps 相等判定也是非传递的,这里改为直接比较、由 id 终裁,彻底消除非传递性。
    auto key = [&](const VehicleAgent& v) {
        const bool starved = v.wait_time > cfg_.starvation_wait_time;
        return std::make_tuple(starved ? 0 : 1,             // 饥饿组优先
                               starved ? -v.wait_time : 0.0, // 组内久者优先
                               v.loaded ? 0 : 1,             // 载货优先
                               v.task_count,                 // 任务少优先
                               v.id);                        // 唯一终裁
    };
    return key(a) < key(b) ? a.id : b.id;
}

double RuleEngine::predictedTravelDistance(const VehicleAgent& v,
                                           VehicleAction action,
                                           double t) const {
    if (t <= 0.0) return 0.0;

    const double desired = speedForAction(action);
    const double current = std::max(0.0, v.current_speed);
    if (std::abs(desired - current) < 1e-9) {
        return current * t;
    }

    const bool accelerating = desired > current;
    const double limit = accelerating ? cfg_.max_accel : cfg_.max_decel;
    if (limit <= 1e-9) {
        return current * t;
    }

    const double signed_accel = accelerating ? limit : -limit;
    const double ramp_time = std::abs(desired - current) / limit;
    const double ramp_used = std::min(t, ramp_time);
    const double ramp_dist =
        current * ramp_used + 0.5 * signed_accel * ramp_used * ramp_used;
    if (t <= ramp_time) {
        return std::max(0.0, ramp_dist);
    }
    return std::max(0.0, ramp_dist + desired * (t - ramp_time));
}


std::vector<RuleEngine::ConflictZone> RuleEngine::computeConflictZonesFull(
    const VehicleAgent& self, const VehicleAgent& other) const {
    constexpr double kStep = 0.025;
    constexpr double kMergeGap = kStep * 2.25;
    std::vector<ConflictZone> zones;

    // 两条路径都是「完全固定、已知」的;它们之间的冲突集 C_ij 是静态几何量,与时间/
    // 速度/朝向/当前位置都无关 → 在整段路径 [0,L]×[0,L] 上一次算定即可(由
    // conflictBlocksCanonical 按 path_gen 缓存)。当前位置相关的裁剪(裁掉已清出的块、
    // 把入口夹到车尾起点)在 findConflictZones 里按调用时的 path_s 施加。
    // (历史教训:早先按 current_speed×prediction_horizon 裁剪扫描范围,会在让行车停在
    // 停止线 speed→0 时让冲突区凭空消失 → 原子门预约被释放 → 让行车蹭过停止线挤进区
    // → 三车旋转楔死。改扫整段固定路径根治;此处进一步缓存,几何恒定可见且不每拍重算。)
    const double s_self_end = self.track.length();
    const double s_other_end = other.track.length();
    const double s_self_begin = 0.0;
    const double s_other_begin = 0.0;

    // 广相剪枝(扫全程后的性能护栏):两车「剩余路径」的轴对齐包围盒各按「车身对角线
    // 半径 + 冲突余量」外胀;若两盒分离,则任意位姿下车身 OBB 绝无可能重叠 → 直接返回空,
    // 免去对八竿子打不着的远车做整段精扫。这只省算力,绝不改变任何冲突判定结果。
    {
        const double inf = std::numeric_limits<double>::infinity();
        const double infl =
            0.5 * std::hypot(mp_.vehicle_length, mp_.vehicle_width) +
            cfg_.conflict_margin;
        constexpr double kCoarse = 0.15;  // 粗采样建包围盒
        double ax0 = inf, ay0 = inf, ax1 = -inf, ay1 = -inf;
        for (double s = s_self_begin; s <= s_self_end + 1e-9; s += kCoarse) {
            const RoughWp p = self.track.poseAtS(std::min(s, s_self_end));
            ax0 = std::min(ax0, p.x); ay0 = std::min(ay0, p.y);
            ax1 = std::max(ax1, p.x); ay1 = std::max(ay1, p.y);
        }
        double bx0 = inf, by0 = inf, bx1 = -inf, by1 = -inf;
        for (double s = s_other_begin; s <= s_other_end + 1e-9; s += kCoarse) {
            const RoughWp p = other.track.poseAtS(std::min(s, s_other_end));
            bx0 = std::min(bx0, p.x); by0 = std::min(by0, p.y);
            bx1 = std::max(bx1, p.x); by1 = std::max(by1, p.y);
        }
        if (ax1 + infl < bx0 - infl || bx1 + infl < ax0 - infl ||
            ay1 + infl < by0 - infl || by1 + infl < ay0 - infl) {
            return zones;  // 包围盒分离 → 无冲突
        }
    }

    for (double ss = s_self_begin; ss <= s_self_end + 1e-9; ss += kStep) {
        const double ss_clamped = std::min(ss, s_self_end);
        // 冲突检测加安全余量：每车身各胀 conflict_margin/2，总净距 < conflict_margin
        // 即判为冲突 → 提前刹住、留出安全距离(真实开车的"别贴太近")。余量须 < 双车道
        // 会车净间隙(本图 0.059m)，否则会把正常对向会车也误判成冲突。硬护栏仍用 0 余量
        // 作真碰撞底线。
        const double cm = cfg_.conflict_margin * 0.5;
        const OBB obb_s = makeBody(self.track.poseAtS(ss_clamped), mp_, cm);
        std::vector<OverlapSample> row;

        for (double so = s_other_begin; so <= s_other_end + 1e-9; so += kStep) {
            const double so_clamped = std::min(so, s_other_end);
            const OBB obb_o = makeBody(other.track.poseAtS(so_clamped), mp_, cm);

            if (!overlaps(obb_s, obb_o)) continue;

            const RoughWp ps = self.track.poseAtS(ss_clamped);
            const RoughWp po = other.track.poseAtS(so_clamped);
            row.push_back(OverlapSample{ss_clamped, so_clamped,
                                        0.5 * (ps.x + po.x),
                                        0.5 * (ps.y + po.y)});
        }

        if (row.empty()) continue;

        std::vector<ConflictZone> row_zones;
        for (const OverlapSample& sample : row) {
            if (row_zones.empty() ||
                sample.s_other > row_zones.back().s_other_exit + kMergeGap) {
                ConflictZone z;
                z.s_self_enter = sample.s_self;
                z.s_self_exit = sample.s_self;
                z.s_other_enter = sample.s_other;
                z.s_other_exit = sample.s_other;
                z.x = sample.x;
                z.y = sample.y;
                row_zones.push_back(z);
            } else {
                ConflictZone& z = row_zones.back();
                z.s_other_exit = sample.s_other;
                z.x = 0.5 * (z.x + sample.x);
                z.y = 0.5 * (z.y + sample.y);
            }
        }

        for (const ConflictZone& row_zone : row_zones) {
            bool merged = false;
            for (ConflictZone& z : zones) {
                const bool self_touch =
                    row_zone.s_self_enter <= z.s_self_exit + kMergeGap;
                const bool other_touch =
                    row_zone.s_other_enter <= z.s_other_exit + kMergeGap &&
                    row_zone.s_other_exit + kMergeGap >= z.s_other_enter;
                if (!self_touch || !other_touch) continue;

                z.s_self_enter = std::min(z.s_self_enter, row_zone.s_self_enter);
                z.s_self_exit = std::max(z.s_self_exit, row_zone.s_self_exit);
                z.s_other_enter = std::min(z.s_other_enter, row_zone.s_other_enter);
                z.s_other_exit = std::max(z.s_other_exit, row_zone.s_other_exit);
                z.x = 0.5 * (z.x + row_zone.x);
                z.y = 0.5 * (z.y + row_zone.y);
                merged = true;
                break;
            }
            if (!merged) {
                zones.push_back(row_zone);
            }
        }
    }

    // 为每个块算「同向」标志(静态):在块中点测两路径行进朝向(REVERSE 段切线+π),
    // 同向(dot>0.7)=正对角带=同车道跟车;否则交叉/对向。供 resolveFollowing 与 pairwise
    // 共用作稳定判据——不再用随当前位姿闪烁的瞬时朝向(交叉/汇入处会瞬时对齐而误判)。
    constexpr double kPi = 3.14159265358979323846;
    auto pathHeadingAtS = [&](const VehicleAgent& v, double s) {
        double h = v.track.poseAtS(s).theta;
        if (v.track.typeAtS(s) == WpType::REVERSE) h += kPi;
        return h;
    };
    for (ConflictZone& z : zones) {
        const double hs =
            pathHeadingAtS(self, 0.5 * (z.s_self_enter + z.s_self_exit));
        const double ho =
            pathHeadingAtS(other, 0.5 * (z.s_other_enter + z.s_other_exit));
        z.same_dir =
            (std::cos(hs) * std::cos(ho) + std::sin(hs) * std::sin(ho)) > 0.7;
    }

    return zones;
}

const std::vector<RuleEngine::ConflictZone>& RuleEngine::conflictBlocksCanonical(
    const VehicleAgent& lo, const VehicleAgent& hi) const {
    // lo.id < hi.id(调用方保证)。按 path_gen 缓存:任一方换了固定路径才重算。
    const std::pair<int, int> key{lo.id, hi.id};
    ConflictCacheEntry& e = conflict_cache_[key];
    if (e.gen_lo != lo.path_gen || e.gen_hi != hi.path_gen) {
        e.blocks = computeConflictZonesFull(lo, hi);  // self=lo, other=hi
        e.gen_lo = lo.path_gen;
        e.gen_hi = hi.path_gen;
    }
    return e.blocks;
}

std::vector<RuleEngine::ConflictZone> RuleEngine::findConflictZones(
    const VehicleAgent& self, const VehicleAgent& other) const {
    // 取静态 C_ij(缓存),按 self/other 朝向取用,再按当前位置裁剪——产物与历史
    // 「逐拍沿剩余路径全程精扫」在同一离散精度下等价(窗口只截低端、不会拆分连通块)。
    const bool self_is_lo = self.id < other.id;
    const VehicleAgent& lo = self_is_lo ? self : other;
    const VehicleAgent& hi = self_is_lo ? other : self;
    const std::vector<ConflictZone>& canon = conflictBlocksCanonical(lo, hi);

    // 车尾参考:车身向后伸 rear_ext。某车「已完全清出某块」= 车尾(s-rear_ext)越过该块
    // 在其路径上的出口 → 该块对它不再是冲突,丢弃。
    // 注:块的入口/出口保持「静态」(不再夹到 path_s-rear_ext)——夹紧会让上报的 se 随车
    // 前移,导致让行方的停止线(se-front)随它一起漂、永远追不上、最终蹭进区(实测 V1↔V5
    // 蹭撞的一半根因)。停止线必须是固定弧长,让行方才能稳稳停在区外。committed/cleared 仍按
    // 静态 se/sx 与当前 path_s 比较,语义不变。
    const double rear_ext = mp_.body_rear_ext();
    const double self_begin = std::max(0.0, self.path_s - rear_ext);
    const double other_begin = std::max(0.0, other.path_s - rear_ext);

    std::vector<ConflictZone> out;
    out.reserve(canon.size());
    for (const ConflictZone& cz : canon) {
        ConflictZone z = cz;
        if (!self_is_lo) {  // 朝向反转:canonical 以 lo 为 self,这里 self 是 hi
            std::swap(z.s_self_enter, z.s_other_enter);
            std::swap(z.s_self_exit, z.s_other_exit);
        }
        // 任一方已完全清出该块 → 不再冲突,丢弃。(入口不夹紧,保持静态)
        if (z.s_self_exit < self_begin || z.s_other_exit < other_begin) continue;
        out.push_back(z);
    }
    return out;
}

void RuleEngine::debugDumpConflict(const VehicleAgent& a,
                                   const VehicleAgent& b) const {
    const std::vector<ConflictZone> zones = findConflictZones(a, b);
    const std::pair<int, int> pkey{std::min(a.id, b.id), std::max(a.id, b.id)};
    int owner = -1;
    auto it = commit_owner_.find(pkey);
    if (it != commit_owner_.end()) owner = it->second;
    const bool following = following_pairs_.count(pkey) > 0;
    const double f = mp_.body_front_ext();
    bool all_same = true;
    for (const ConflictZone& z : zones)
        if (!z.same_dir) { all_same = false; break; }
    ROS_WARN("[CONFLICT] V%d(s=%.3f rem=%.3f act=%d blk=%d) vs "
             "V%d(s=%.3f rem=%.3f act=%d blk=%d) | owner=V%d following=%d "
             "nzones=%zu all_same_dir=%d",
             a.id, a.path_s, a.remainingS(), (int)a.action, a.blocker_id,
             b.id, b.path_s, b.remainingS(), (int)b.action, b.blocker_id,
             owner, (int)following, zones.size(), (int)all_same);
    for (size_t i = 0; i < zones.size(); ++i) {
        const ConflictZone& z = zones[i];
        ROS_WARN("[CONFLICT]   zone%zu same_dir=%d | A[%.3f,%.3f] stopA=%.3f "
                 "committedA=%d | B[%.3f,%.3f] stopB=%.3f committedB=%d | @(%.2f,%.2f)",
                 i, (int)z.same_dir, z.s_self_enter, z.s_self_exit,
                 z.s_self_enter - f, (int)(a.path_s > z.s_self_enter),
                 z.s_other_enter, z.s_other_exit, z.s_other_enter - f,
                 (int)(b.path_s > z.s_other_enter), z.x, z.y);
    }
}

RuleEngine::OccupancyInterval RuleEngine::occupancyInterval(
    const VehicleAgent& v, VehicleAction action,
    double zone_enter_s, double zone_exit_s) const {
    OccupancyInterval out;
    constexpr double kEps = 1e-6;

    if (v.mode == VehicleMode::NEED_TASK || v.track.empty()) return out;

    // 车身沿弧长向参考点(后轴)前方伸 front_ext、后方伸 rear_ext。判断「占用某
    // 区段」必须用整车身范围 [s - rear_ext, s + front_ext]，而非仅参考点 s ——
    // 否则车头已插进区段、后轴还在区外的车会被判为「未占用」，对向车以为路空就
    // 撞上去（墨绿车身挡住路口、橙色照冲的根因）。
    const double front_ext = mp_.body_front_ext();
    const double rear_ext = mp_.body_rear_ext();

    const double current_s = (v.mode == VehicleMode::DWELL)
        ? v.track.length()
        : v.path_s;
    // 车尾(s - rear_ext)已越过区段出口 → 整车确实驶离，不再占用。
    if (current_s - rear_ext > zone_exit_s + kEps) return out;

    // 车身(含前后伸)与区段相交即视为「此刻就压在区内」。
    const bool currently_inside =
        current_s + front_ext >= zone_enter_s - kEps &&
        current_s - rear_ext <= zone_exit_s + kEps;

    if (v.mode == VehicleMode::DWELL || action == VehicleAction::STOP) {
        if (!currently_inside) return out;
        out.occupies = true;
        out.enter = 0.0;
        out.exit = std::numeric_limits<double>::infinity();
        return out;
    }

    if (currently_inside) {
        out.enter = 0.0;
        // 车尾清出区段：参考点须前进到 exit + rear_ext。
        out.exit = timeToReachS(v, action, zone_exit_s + rear_ext);
    } else {
        // 车头抵达入口 = 参考点到 (enter - front_ext)；车尾清出 = 参考点到
        // (exit + rear_ext)。全部用车身边界，不用裸参考点。
        out.enter = timeToReachS(v, action, zone_enter_s - front_ext);
        out.exit = timeToReachS(v, action, zone_exit_s + rear_ext);
    }

    if (!std::isfinite(out.enter) || !std::isfinite(out.exit)) return {};
    if (out.exit < out.enter) out.exit = out.enter;
    out.occupies = true;
    return out;
}

bool RuleEngine::intervalsOverlap(const OccupancyInterval& a,
                                  const OccupancyInterval& b) const {
    if (!a.occupies || !b.occupies) return false;
    constexpr double kTimeEps = 1e-4;
    return a.enter < b.exit - kTimeEps && b.enter < a.exit - kTimeEps;
}

double RuleEngine::timeToReachS(const VehicleAgent& v, VehicleAction action,
                                 double target_s) const {
    if (target_s <= v.path_s + 1e-9) return 0.0;

    const double dist    = target_s - v.path_s;
    const double v_des   = speedForAction(action);
    const double v_cur   = std::max(0.0, v.current_speed);

    if (v_des < 1e-9) return std::numeric_limits<double>::infinity();

    // Binary search: invert predictedTravelDistance
    double lo = 0.0;
    double hi = dist / v_des * 3.0 + 2.0;
    for (int i = 0; i < 64; ++i) {
        const double mid = (lo + hi) * 0.5;
        (predictedTravelDistance(v, action, mid) < dist ? lo : hi) = mid;
    }
    return hi;
}

void RuleEngine::applyActionRequest(VehicleAgent& v, VehicleAction action,
                                    const std::string& reason,
                                    int blocker_id) {
    if (moreRestrictive(action, v.requested_action)) {
        v.requested_action = action;
        v.reason = reason;
        v.blocker_id = blocker_id;
    }
}

void RuleEngine::resolvePairwiseConflicts(std::vector<VehicleAgent>& vehicles,
                                          double dt) {
    // 资源-时间窗调度。对每一对车：沿各自固定路径找出车身 OBB 会重叠的弧长冲突区
    // 段(findConflictZones，已含后轴车尾后伸)，用解析时间窗(occupancyInterval)判断
    // 二者占用该区段的时间是否重叠；若重叠则按规则裁决谁先走(priorityWinner)，并把
    // 这一裁决「预约闭锁」(commit_owner_) —— owner 保持路权直到车身完全驶出共享区，
    // 期间不因 wait_time/饥饿等状态变化而翻转，让行方持续让行。硬护栏(patrol node)仍
    // 是不可关闭的安全底线。

    // 后轴参考下，车身相对参考点(后轴)向前伸 body_front_ext、向后伸 body_rear_ext。
    // 「占用某区段」= 参考点 s ∈ [enter - 前伸, exit + 后伸]（车身任一点压在区段上）。
    const double front_ext = mp_.body_front_ext();
    const double rear_ext = mp_.body_rear_ext();
    auto insideZone = [&](const VehicleAgent& v, double enter_s, double exit_s) {
        const double s =
            (v.mode == VehicleMode::DWELL) ? v.track.length() : v.path_s;
        return s >= enter_s - front_ext && s <= exit_s + rear_ext;
    };

    auto terminalDocking = [&](const VehicleAgent& v) {
        if (!v.active()) return false;
        const double terminal_distance =
            std::max(cfg_.target_request_distance, cfg_.target_stop_distance);
        return v.remainingS() <= terminal_distance;
    };

    auto brakeIfNeeded = [&](VehicleAgent& v, double zone_enter_s,
                             int other_id) {
        // §9:破环车在所有层都是最高优先级,pairwise 不得给它刹车(否则它拿了
        // 最高优先级却被旧层摁停 → 蹭行,环破不了)。真碰撞仍由硬护栏 0 余量兜底。
        if (v.deadlock_breaker) return;
        // 让行车停在「车头(参考点+前伸)正好抵到区段入口」处，即参考点停在
        // zone_enter - 前伸，车身完全停在共享区外，owner 可无阻通过。
        const double stop_line_s = zone_enter_s - front_ext;
        const double dist_to_zone = std::max(0.0, stop_line_s - v.path_s);
        const double v_cur = std::max(0.0, v.current_speed);
        // 精确制动距离：v²/(2a) (制动段) + v·dt (本帧决策延迟 — STOP 命令在
        // 下一帧才生效，这帧车辆还在以当前速度行驶)。全部基于当前状态，
        // 不依赖任何固定经验参数。
        const double braking_dist =
            (v_cur * v_cur) / (2.0 * std::max(1e-6, cfg_.max_decel));
        const double reaction_dist = v_cur * dt;
        if (dist_to_zone <= braking_dist + reaction_dist) {
            applyActionRequest(v, VehicleAction::STOP,
                               "brake_V" + std::to_string(other_id), other_id);
        }
    };

    // 给定一个冲突区段，取属于 agent `is_a`(true=self/a, false=other/b) 一侧的弧长
    // 入口/出口。findConflictZones(a,b) 中 self=a，故 a 用 s_self_*，b 用 s_other_*。
    auto sideEnter = [](const ConflictZone& z, bool is_a) {
        return is_a ? z.s_self_enter : z.s_other_enter;
    };
    auto sideExit = [](const ConflictZone& z, bool is_a) {
        return is_a ? z.s_self_exit : z.s_other_exit;
    };

    // 预约表剪枝：owner 已不在 ACTIVE(任务结束/重分配/进入 DWELL) 的条目作废。
    auto agentById = [&](int id) -> VehicleAgent* {
        for (VehicleAgent& v : vehicles) {
            if (v.id == id) return &v;
        }
        return nullptr;
    };
    for (auto it = commit_owner_.begin(); it != commit_owner_.end();) {
        VehicleAgent* owner = agentById(it->second);
        if (owner == nullptr || owner->mode != VehicleMode::ACTIVE) {
            it = commit_owner_.erase(it);
        } else {
            ++it;
        }
    }

    for (size_t i = 0; i < vehicles.size(); ++i) {
        VehicleAgent& a = vehicles[i];
        if (!a.active()) continue;

        for (size_t j = i + 1; j < vehicles.size(); ++j) {
            VehicleAgent& b = vehicles[j];
            // DWELL(泊在货位)的车是「静态障碍」,不参与会动的车之间的预约式交叉协调——
            // 那套带 conflict_margin 提前量的门控,会把仅从它旁边经过、本可贴安全余量通过的
            // 车干等到它出库(实测 V3 被泊车 V4 干等 30s;紫/蓝车同样)。静态障碍由三处既有且
            // 正确的机制接管:resolveTargetSlotOccupancy(想进它占用的库位→口外等)、
            // enforceForwardClearance(沿前向 0 余量净空、真要贴上才停)、硬护栏(0 余量兜底)。
            // 故此处 DWELL/inactive 一律跳过;b 通过后必为 ACTIVE。
            if (!b.active()) continue;

            // 先取缓存的冲突块(静态 C_ij + 当前位置裁剪),后续跟车判定/门控都基于它。
            const std::vector<ConflictZone> zones = findConflictZones(a, b);
            const std::pair<int, int> pkey{std::min(a.id, b.id),
                                           std::max(a.id, b.id)};
            if (zones.empty()) {
                commit_owner_.erase(pkey);  // 无共享冲突区:释放可能残留的预约。
                continue;
            }

            // 协调图第4步(改):同向同车道跟车交 resolveFollowing 管纵向跟距。是否跳过
            // 完全以 resolveFollowing 本周期认定的 following_pairs_ 为唯一事实源——
            // 「被跳过 ⟺ 被 following 接管」,杜绝两层判据不一致留下的缝隙(旧版 pairwise
            // 用块处/瞬时朝向自行判定,与 following 的判据不符 → 某些对被 pairwise 跳过却
            // 无人接管 → 头对头/汇入对撞,实测 V1↔V4 / V0↔V7 / V1↔V5)。不在该集合中的对
            // 一律由下方原子门门控,绝不漏。
            if (following_pairs_.count(pkey)) continue;

            // 协调图第3步:时间窗用「两车都按 NOMINAL」预测占用——回答的是「若双方都
            // 正常行驶,会不会在同一冲突块里相遇」这一与当前谁停谁走无关的客观问题。
            //  · 治 TODO-1:让行方不再因为「此刻为对方停住」就被钉成永久占用([0,∞])而
            //    时窗恒真、白等对方走完全程;按 NOMINAL 它若能在对方到达前清出本块,则时窗
            //    不相交 → 不需让行,直接放行(实测 V4 出库 1.3s 清出口,远早于 V1 到达)。
            //  · 同时根除原 ghost-clearance 对撞:两车都按 NOMINAL 评估,不存在「一方因慢/
            //    停而显得不在场」的前提 → B 不会误判 A 已消失而冲进共享块。
            //  · 稳定性不靠此处:真冲突一旦裁出 holder 并预约,由 step1(持有到清出)+
            //    committed 逻辑保证不翻转,与时间窗无关。故去掉旧「自锁(blocker==partner→
            //    STOP→[0,∞])」不会让已设预约抖动。
            //  · DWELL(停驻)车是真正静止的静态障碍 → 仍按 STOP(占据其当前位置)。
            auto predAction = [](bool is_dwell) -> VehicleAction {
                return is_dwell ? VehicleAction::STOP : VehicleAction::NOMINAL;
            };
            const VehicleAction a_action = predAction(false);
            const VehicleAction b_action = predAction(false);

            // ── §11.3 原子通行段:整片共享冲突区 = 单一互斥资源 ────────────────
            // 整片区域 = 所有冲突区在各自路径上的弧长外包 [se, sx]。同一时刻只允许
            // 一辆持有整片区域;持有车走到车身完全驶出整片区域才释放;另一辆在整片区域
            // 上游停止线(车身完全在区外)等待。从源头杜绝两车同时挤进互相挡的区域。
            const double kInf = std::numeric_limits<double>::infinity();
            double se_a = kInf, sx_a = -kInf, se_b = kInf, sx_b = -kInf;
            for (const ConflictZone& z : zones) {
                se_a = std::min(se_a, sideEnter(z, true));
                sx_a = std::max(sx_a, sideExit(z, true));
                se_b = std::min(se_b, sideEnter(z, false));
                sx_b = std::max(sx_b, sideExit(z, false));
            }
            // 「已驶入区内」= 参考点越过区域入口(车头刚停在停止线 path_s=se-front_ext
            // < se 不算)。「已完全清出」= 车尾越过区域出口。
            const bool a_committed = a.path_s > se_a;
            const bool b_committed = b.path_s > se_b;
            auto cleared = [&](const VehicleAgent& v, double sx) {
                return v.path_s - rear_ext > sx;
            };

            // [DIAG wedge] 某对车卡很久(wait>8s)→ 打印整片区域几何 + 各冲突区明细,
            // 定位根因。每对只打一次,只读。
            {
                static std::map<std::pair<int, int>, double> wedge_logged;
                const bool stuck = (a.wait_time > 8.0 || b.wait_time > 8.0);
                auto wit = wedge_logged.find(pkey);
                const bool due = (wit == wedge_logged.end()) ||
                                 (now_ - wit->second > 3.0);
                if (stuck && due) {
                    wedge_logged[pkey] = now_;
                    int owner_id = -1;
                    auto it = commit_owner_.find(pkey);
                    if (it != commit_owner_.end()) owner_id = it->second;
                    const double fe = mp_.body_front_ext();
                    ROS_WARN("[DIAG wedge] V%d(s=%.3f,rem=%.3f,wait=%.1f,act=%d) vs "
                             "V%d(s=%.3f,rem=%.3f,wait=%.1f,act=%d) owner=V%d nzones=%zu | "
                             "A region[se=%.3f sx=%.3f] stopline=%.3f committed=%d | "
                             "B region[se=%.3f sx=%.3f] stopline=%.3f committed=%d | front_ext=%.3f",
                             a.id, a.path_s, a.remainingS(), a.wait_time, (int)a.action,
                             b.id, b.path_s, b.remainingS(), b.wait_time, (int)b.action,
                             owner_id, zones.size(),
                             se_a, sx_a, se_a - fe, (int)a_committed,
                             se_b, sx_b, se_b - fe, (int)b_committed, fe);
                    for (size_t zi = 0; zi < zones.size(); ++zi) {
                        ROS_WARN("[DIAG wedge]   zone%zu A[%.3f,%.3f] B[%.3f,%.3f] @(%.2f,%.2f)",
                                 zi, zones[zi].s_self_enter, zones[zi].s_self_exit,
                                 zones[zi].s_other_enter, zones[zi].s_other_exit,
                                 zones[zi].x, zones[zi].y);
                    }
                }
            }

            // 1) 已有预约:持有车保持路权直到完全清出整片区域;另一辆在上游停止线等。
            {
                auto it = commit_owner_.find(pkey);
                if (it != commit_owner_.end()) {
                    const bool owner_is_a = (it->second == a.id);
                    VehicleAgent& owner = owner_is_a ? a : b;
                    VehicleAgent& other = owner_is_a ? b : a;
                    const double owner_sx = owner_is_a ? sx_a : sx_b;
                    const double other_se = owner_is_a ? se_b : se_a;
                    const bool owner_committed = owner_is_a ? a_committed : b_committed;
                    const bool other_committed = owner_is_a ? b_committed : a_committed;
                    if (cleared(owner, owner_sx)) {
                        commit_owner_.erase(it);  // 持有车已清出 → 释放,落下方重裁
                    } else if (other_committed && !owner_committed) {
                        // 互斥铁律:让行方已陷在共享区内、而持有方还在区外进不来 →
                        // 持有方此刻无法穿越(会撞区内的让行方),应改由「已在区内者」持有
                        // 先清出,持有方退为区外等待。释放预约,落下方按 committed 重裁。
                        // (修 V6↔V7:owner=V6 在区外硬往里开、撞 V7 困在区内的死锁)
                        commit_owner_.erase(it);
                    } else {
                        brakeIfNeeded(other, other_se, owner.id);
                        conflicts_.push_back(ConflictMarker{
                            zones.front().x, zones.front().y, 0.0, a.id, b.id});
                        continue;  // 持有到清出,绝不翻转
                    }
                }
            }

            // 2) 时间窗闸:仅当两车「都按 NOMINAL 行驶」时占用同一块的时间确有重叠才协调
            //    (§15 不提前等)。这是客观的"会不会相遇",不随谁此刻停走而变 → 时窗错开
            //    的远车(如 TODO-1 的 V1)不会再把已在区口、能先清出的车(V4)钉住空等。
            bool time_conflict = false;
            for (const ConflictZone& z : zones) {
                const OccupancyInterval oa = occupancyInterval(
                    a, a_action, z.s_self_enter, z.s_self_exit);
                const OccupancyInterval ob = occupancyInterval(
                    b, b_action, z.s_other_enter, z.s_other_exit);
                if (intervalsOverlap(oa, ob)) { time_conflict = true; break; }
            }
            if (!time_conflict && !a_committed && !b_committed) {
                continue;  // 时间上错开且都未进区 → 无需协调
            }
            conflicts_.push_back(ConflictMarker{
                zones.front().x, zones.front().y, 0.0, a.id, b.id});

            // 3) 无预约:裁决整片区域的持有车(holder)并预约。
            //    互斥优先:已在区内者必须清出→它持有;都不在→末端泊入/出库(CLEAR/EXIT,
            //    §11.1)优先,否则统一优先级 priorityWinner。
            int holder = -1;
            if (a_committed && !b_committed) {
                holder = a.id;
            } else if (b_committed && !a_committed) {
                holder = b.id;
            } else if (a_committed && b_committed) {
                // 两车都已在区内(原子门失效的残留态)→ 真楔死,双停,交硬护栏+破环逃生。
                brakeIfNeeded(a, se_a, b.id);
                brakeIfNeeded(b, se_b, a.id);
                continue;
            } else {
                const bool a_term = terminalDocking(a);
                const bool b_term = terminalDocking(b);
                if (a_term != b_term) holder = a_term ? a.id : b.id;  // CLEAR 优先
                else holder = priorityWinner(a, b);  // -1=tiebreak 关
            }

            if (holder == a.id) {
                brakeIfNeeded(b, se_b, a.id);
                commit_owner_[pkey] = a.id;
            } else if (holder == b.id) {
                brakeIfNeeded(a, se_a, b.id);
                commit_owner_[pkey] = b.id;
            } else {
                // tiebreak 关闭:双方停,不预约(纯避撞,可能死锁)。
                brakeIfNeeded(a, se_a, b.id);
                brakeIfNeeded(b, se_b, a.id);
            }
        }
    }
}

void RuleEngine::resolveFollowing(std::vector<VehicleAgent>& vehicles) {
    // 本周期重新认定"真·同向同车道跟车对",供 resolvePairwiseConflicts 决定是否跳过。
    // 这是跟车判据的唯一事实源 → 杜绝两层判据不一致的缝隙。
    following_pairs_.clear();
    auto motionHeading = [](const VehicleAgent& v) {
        constexpr double kPi = 3.14159265358979323846;
        double heading = v.track.poseAtS(v.path_s).theta;
        if (v.track.typeAtS(v.path_s) == WpType::REVERSE) {
            heading += kPi;
        }
        return heading;
    };

    auto terminalDocking = [&](const VehicleAgent& v) {
        if (!v.active()) return false;
        const double terminal_distance =
            std::max(cfg_.target_request_distance, cfg_.target_stop_distance);
        return v.remainingS() <= terminal_distance;
    };

    for (VehicleAgent& v : vehicles) {
        if (!v.active()) continue;
        if (terminalDocking(v)) continue;

        const RoughWp pose_v = v.track.poseAtS(v.path_s);
        const double heading_v = motionHeading(v);

        for (const VehicleAgent& other : vehicles) {
            if (other.id == v.id || !other.active()) continue;
            const RoughWp pose_o = other.track.poseAtS(other.path_s);
            // 同向跟车判定用「静态冲突块的对角方向 same_dir」而非当前瞬时朝向——后者在
            // 交叉/汇入处会瞬时对齐,把真正交叉的两车误判成跟车;而 pairwise 据此(following_pairs_)
            // 跳过 → 该对无任何交叉门控 → 让行方在 action_hold 回弹下蹭过停止线、撞上停着的车
            // (实测 V1↔V5)。块 same_dir 是纯几何静态量,稳定不闪。要求两车确有共享冲突块且
            // 每块都同向,才算同车道跟车;否则(交叉/对向/无重叠)一律交原子门门控。
            const std::vector<ConflictZone> fzones = findConflictZones(v, other);
            if (fzones.empty()) continue;
            bool all_same_dir = true;
            for (const ConflictZone& z : fzones) {
                if (!z.same_dir) { all_same_dir = false; break; }
            }
            if (!all_same_dir) continue;

            const double dx = pose_o.x - pose_v.x;
            const double dy = pose_o.y - pose_v.y;
            const double fwd = dx * std::cos(heading_v) +
                               dy * std::sin(heading_v);
            if (fwd <= 0.0) continue;

            const double lat = std::abs(-dx * std::sin(heading_v) +
                                         dy * std::cos(heading_v));
            if (lat > mp_.vehicle_width) continue;

            // 确认 v 真在同车道跟随 other(同向、other 在正前方、横向≤一个车宽)。登记此对
            // 为跟车对——即使下面因间距大而暂不刹车,也要登记,使 pairwise 一致地跳过它
            // (否则大间距跟车对会被 pairwise 当交叉、两车都 committed→双刹楔死)。
            following_pairs_.insert({std::min(v.id, other.id),
                                     std::max(v.id, other.id)});

            const double dist = std::hypot(dx, dy);
            const double gap = dist - mp_.vehicle_length;

            VehicleAction follow_action;
            if (gap <= cfg_.following_min_distance) {
                follow_action = VehicleAction::STOP;
            } else if (gap <= cfg_.following_creep_distance) {
                follow_action = VehicleAction::CREEP;
            } else if (gap <= cfg_.following_normal_distance) {
                follow_action = VehicleAction::YIELD;
            } else {
                continue;
            }
            applyActionRequest(v, follow_action,
                               "following_V" + std::to_string(other.id),
                               other.id);
        }
    }
}

void RuleEngine::enforceForwardClearance(std::vector<VehicleAgent>& vehicles) {
    // 普适前向净空护栏(§11.13.1 出口检查精神 + 补 following/crossing 分类接缝漏洞)。
    // 接缝 bug:近乎同向、向不同库位汇聚的两车,被 pairwise 当跟车跳过、又不满足
    // resolveFollowing 的横向/间距条件 → 两套都没刹 → 后车 NOMINAL 直撞停在路口的前车
    // → 形成谁前进都撞对方的十字楔死(canStepForward 双 false、破环无效、硬护栏反复)。
    // 兜底:任何车沿自身固定路径在「自己刹车距离 + 车头前伸」内会压上另一辆车的当前
    // 车身,就提前 STOP。不论被哪套逻辑处理这道都在。比硬护栏(0 余量、贴死才停)早刹、
    // 留余量 → 两车干净对停而非重叠 → 破环车 canStepForward 能判出谁可走 → 解开。
    const double front_ext = mp_.body_front_ext();
    auto bodyAt = [&](const VehicleAgent& v, double s) {
        return makeBody(v.track.poseAtS(s), mp_, 0.0);
    };
    auto currentS = [&](const VehicleAgent& v) {
        return (v.mode == VehicleMode::DWELL) ? v.track.length() : v.path_s;
    };
    for (VehicleAgent& v : vehicles) {
        if (!v.active()) continue;
        if (v.deadlock_breaker) continue;  // 破环车豁免:它正被授权冲出环
        if (v.requested_action == VehicleAction::STOP) continue;
        const double v_cur = std::max(0.0, v.current_speed);
        const double brake_dist =
            (v_cur * v_cur) / (2.0 * std::max(1e-6, cfg_.max_decel));
        // 前探距离必须足够远,让车「早早停在冲突区外、留出间隙」,而不是冲到贴上才刹
        // (低速时 brake_dist 极小,只算它会一直蹭到接触才停=楔死)。故在刹车距离之外
        // 再加:车头前伸 + 一个固定安全间隙 kStandoff。kStandoff 同时是「干净对停」后
        // 两车之间留出的余量,使破环车 canStepForward 有空间判别谁能动。另设最小前探
        // kMinLook,保证即便停着(brake_dist≈0)也能看到近处已挡在交叉口的车。
        constexpr double kStandoff = 0.16;  // 停在冲突区外留出的安全间隙
        constexpr double kMinLook = 0.22;   // 最小前探(覆盖交叉接近段,防停车时漏看)
        const double look =
            std::max(brake_dist + kStandoff, kMinLook) + front_ext;
        const double s_end = std::min(v.track.length(), v.path_s + look);
        constexpr double kStep = 0.03;
        int block_id = -1;
        for (double s = v.path_s; s <= s_end + 1e-9 && block_id < 0; s += kStep) {
            const OBB body = bodyAt(v, std::min(s, s_end));
            for (const VehicleAgent& o : vehicles) {
                if (o.id == v.id) continue;
                if (o.mode == VehicleMode::NEED_TASK || o.track.empty()) continue;
                if (overlaps(body, bodyAt(o, currentS(o)))) {
                    block_id = o.id;
                    break;
                }
            }
        }
        if (block_id >= 0) {
            applyActionRequest(v, VehicleAction::STOP,
                               "clear_block_V" + std::to_string(block_id),
                               block_id);
        }
    }
}

void RuleEngine::resolveTargetSlotOccupancy(
    std::vector<VehicleAgent>& vehicles) {
    constexpr double kMouthWait = 0.35;  // hold this far short of the target
    constexpr double kSlotClear = 0.30;  // occupant must travel this far to free it

    for (VehicleAgent& v : vehicles) {
        if (!v.active()) continue;
        const double dist_to_mouth = v.remainingS() - kMouthWait;
        const double v_cur = std::max(0.0, v.current_speed);
        const double braking_dist =
            (v_cur * v_cur) / (2.0 * std::max(1e-6, cfg_.max_decel));
        if (dist_to_mouth > braking_dist + 0.06) continue;

        for (const VehicleAgent& o : vehicles) {
            if (o.id == v.id) continue;
            if (o.current_slot != v.target_slot) continue;  // not at v's slot
            const bool occupying =
                (o.mode == VehicleMode::DWELL) ||
                (o.active() && o.path_s < kSlotClear);
            if (!occupying) continue;
            applyActionRequest(v, VehicleAction::STOP,
                               "wait_slot_V" + std::to_string(o.id), o.id);
            break;
        }
    }
}

void RuleEngine::applyRequestedActions(std::vector<VehicleAgent>& vehicles,
                                       double dt) {
    const double hold = cfg_.action_hold_time;
    for (VehicleAgent& v : vehicles) {
        if (v.mode != VehicleMode::ACTIVE) {
            v.action = VehicleAction::STOP;
            v.requested_action = VehicleAction::STOP;
            v.action_hold_remaining = 0.0;
            continue;
        }

        // 死锁打破豁免倒计时：豁免期内不允许规则层将车辆降为 STOP
        if (v.cycle_break_immunity > 0.0) {
            v.cycle_break_immunity = std::max(0.0, v.cycle_break_immunity - dt);
        }

        const VehicleAction prev = v.action;            // last cycle's output
        VehicleAction req = v.requested_action;         // this cycle's rules

        // 死锁豁免：仅当 blocker 确实是死锁环成员（等待链最终指回 v）
        // 时才降为 CREEP；若 blocker 只是恰好停着等第三方（非死锁），
        // 安全第一，保持 STOP，不能朝停着的车 CREEP 过去。
        if (v.cycle_break_immunity > 0.0 && req == VehicleAction::STOP) {
            bool blocker_in_cycle = false;
            if (v.blocker_id >= 0) {
                int cur_id = v.blocker_id;
                const int kMaxHops = static_cast<int>(vehicles.size()) + 1;
                for (int hop = 0; hop < kMaxHops && cur_id >= 0; ++hop) {
                    if (cur_id == v.id) { blocker_in_cycle = true; break; }
                    int next_id = -1;
                    for (const VehicleAgent& o : vehicles) {
                        if (o.id == cur_id) {
                            if (o.action == VehicleAction::STOP && o.blocker_id >= 0)
                                next_id = o.blocker_id;
                            break;
                        }
                    }
                    cur_id = next_id;
                }
            }
            if (blocker_in_cycle) {
                req = VehicleAction::CREEP;  // 验证为死锁环，缓行打破
            } else {
                // blocker 不是死锁环成员，安全第一
                v.cycle_break_immunity = 0.0;
            }
        }

        // §9 破环车保底:即便某层仍要它 STOP,也强制至少 CREEP 冲出环(它已在资源/
        // 优先级层拿到最高优先级,这里保证动作落地)。硬护栏仍兜底防真碰撞。
        if (v.deadlock_breaker && req == VehicleAction::STOP) {
            req = VehicleAction::CREEP;
            v.reason = "deadlock_break";
        }

        if (hold <= 0.0) {                              // smoothing disabled
            v.action = req;
            continue;
        }

        if (moreRestrictive(req, prev)) {
            v.action = req;
            v.action_hold_remaining = hold;
        } else if (req == prev) {
            v.action = prev;
            v.action_hold_remaining =
                std::max(0.0, v.action_hold_remaining - dt);
        } else {
            v.action_hold_remaining -= dt;
            if (v.action_hold_remaining > 0.0) {
                v.action = prev;
                if (v.reason == "clear") v.reason = "action_hold";
            } else {
                v.action = minAction(relaxOneStep(prev), req);
                v.action_hold_remaining = hold;
            }
        }
    }
}

void RuleEngine::breakDeadlockCycles(std::vector<VehicleAgent>& vehicles) {
    const size_t n = vehicles.size();

    auto indexOfId = [&](int id) -> int {
        for (size_t i = 0; i < n; ++i) {
            if (vehicles[i].id == id) return static_cast<int>(i);
        }
        return -1;
    };
    auto waitEdge = [&](int i) -> int {
        const VehicleAgent& v = vehicles[i];
        if (v.mode != VehicleMode::ACTIVE) return -1;
        if (v.action != VehicleAction::STOP) return -1;  // only fully-stopped waits
        if (v.blocker_id < 0) return -1;
        return indexOfId(v.blocker_id);
    };

    std::vector<int> visit_state(n, 0);  // 0 unvisited, 1 in-progress, 2 done
    for (size_t s = 0; s < n; ++s) {
        if (visit_state[s] != 0) continue;
        std::vector<int> path;
        int cur = static_cast<int>(s);
        while (cur >= 0 && visit_state[cur] == 0) {
            visit_state[cur] = 1;
            path.push_back(cur);
            cur = waitEdge(cur);
        }
        if (cur >= 0 && visit_state[cur] == 1) {
            const auto cycle_begin = std::find(path.begin(), path.end(), cur);
            int release = -1;
            int release_id = std::numeric_limits<int>::max();
            for (auto p = cycle_begin; p != path.end(); ++p) {
                if (vehicles[*p].id < release_id) {
                    release_id = vehicles[*p].id;
                    release = *p;
                }
            }
            if (release >= 0) {
                VehicleAgent& r = vehicles[release];
                r.action = VehicleAction::YIELD;
                r.requested_action = VehicleAction::YIELD;
                r.action_hold_remaining = 0.0;
                r.cycle_break_immunity = 0.6;  // 0.6s 内规则层不得将该车重新降为 STOP
                r.wait_time = 0.0;
                r.reason = "cycle_break";
            }
        }
        for (int p : path) visit_state[p] = 2;
    }
}

void RuleEngine::arbitrateResources(std::vector<VehicleAgent>& vehicles,
                                    double dt) {
    if (resmap_ == nullptr) return;
    const double front = mp_.body_front_ext();
    const double rear = mp_.body_rear_ext();
    const double decel = std::max(1e-6, cfg_.max_decel);
    const double nominal = speedForAction(VehicleAction::NOMINAL);

    // 资源申请窗口(§15 第二类:不要提前老远占用资源)。只有当车已逼近资源到
    // 「必须开始决策能否在停止线前停住」的预警点时才请求/预约;离得远的资源不预约,
    // 以免一辆还在老远的高优先级车把资源锁死、让近处车无谓干等("明明没啥事就不走")。
    // 窗口 = 标称速刹停距离 + 2 个车身(决策缓冲),纯几何派生,非拍脑袋常数。
    const double request_window =
        nominal * nominal / (2.0 * decel) + 2.0 * mp_.vehicle_length;

    // 1) 按资源聚合请求者:active 车的固定路径会用到该 capacity=1 互斥资源
    //    (窄道/路口/货位口)、车尾还没整车驶出、且已进入申请窗口。
    struct Req { size_t idx; double s_enter; double s_exit; };
    std::map<int, std::vector<Req>> by_res;
    for (size_t i = 0; i < vehicles.size(); ++i) {
        const VehicleAgent& v = vehicles[i];
        if (!v.active()) continue;
        for (const ResourceSpan& sp : v.resource_spans) {
            const TrafficResource* r = resmap_->byId(sp.resource_id);
            if (r == nullptr || r->capacity != 1) continue;
            if (r->type != ResourceType::NARROW &&
                r->type != ResourceType::INTERSECTION &&
                r->type != ResourceType::SLOT_DOCK) {
                continue;
            }
            if (v.path_s - rear > sp.s_exit + 1e-6) continue;  // 已整车驶出
            if (sp.s_enter - v.path_s > request_window) continue;  // 尚远,不预约
            by_res[sp.resource_id].push_back(Req{i, sp.s_enter, sp.s_exit});
        }
    }

    auto bodyInside = [&](const VehicleAgent& v, const Req& q) {
        return v.path_s + front >= q.s_enter - 1e-6 &&
               v.path_s - rear <= q.s_exit + 1e-6;
    };

    // 某车当前车身(后轴还原车身中心)。DWELL 用终点位姿,空轨迹跳过。
    auto poseOf = [&](const VehicleAgent& o) {
        const double s = (o.mode == VehicleMode::DWELL) ? o.track.length()
                                                        : o.path_s;
        return o.track.poseAtS(std::min(s, o.track.length()));
    };
    // 出口检查(§11.4/§11.13.1):候选车驶出资源后的落脚处(s_exit + 半车长)
    // 是否被别的车身占住。被占 → 进去就会卡在资源里 → 不发令牌(除非它已在区内
    // 必须驶完)。这杜绝"令牌发给进得去出不来的车、它攥着令牌却动不了挡死所有人"。
    auto canExit = [&](size_t vi, const Req& q) {
        const VehicleAgent& v = vehicles[vi];
        const double len = v.track.length();
        const double s_check = std::min(q.s_exit + 0.5 * mp_.vehicle_length, len);
        const OBB body = makeBody(v.track.poseAtS(s_check), mp_, 0.0);
        for (size_t o = 0; o < vehicles.size(); ++o) {
            if (o == vi) continue;
            const VehicleAgent& ov = vehicles[o];
            if (ov.mode != VehicleMode::ACTIVE && ov.mode != VehicleMode::DWELL)
                continue;
            if (ov.track.empty()) continue;
            if (overlaps(body, makeBody(poseOf(ov), mp_, 0.0))) return false;
        }
        return true;
    };
    // 可授予 = 已在区内(必须驶完)或 出口畅通。
    auto grantableK = [&](const std::vector<Req>& rs, size_t k) {
        return bodyInside(vehicles[rs[k].idx], rs[k]) || canExit(rs[k].idx, rs[k]);
    };

    // 2) 逐资源仲裁:已持令牌且仍在请求 → 保持(持权到整车驶出,防翻转,§11);
    //    否则按统一优先级 PriorityKey 选 winner 并发令牌(§11.2)。
    for (auto& kv : by_res) {
        const int rid = kv.first;
        std::vector<Req>& reqs = kv.second;

        // 持有者保持令牌的前提:它仍在请求 且 仍可授予(能驶出/已在区内)。
        // 在「可授予」候选里按统一 PriorityKey 选 winner(§11.2)。持令牌者带
        // already_has_token 加成(防翻转,§11),破环车带 emergency 加成(临时最高,§9)
        // —— 于是 破环车 > 持令牌者 > 其他,一套优先级统一裁决,不再用 holder 捷径。
        const int holder = tokens_.holder(rid);
        int winner_k = -1;
        {
            ResourceRequest best;
            bool has_best = false;
            for (size_t k = 0; k < reqs.size(); ++k) {
                if (!grantableK(reqs, k)) continue;  // 出口被堵的不参与(进去会卡死)
                const VehicleAgent& v = vehicles[reqs[k].idx];
                ResourceRequest rq;
                rq.vehicle_id = v.id;
                rq.wait_time = v.wait_time;
                rq.loaded = v.loaded;
                rq.task_count = v.task_count;
                rq.already_inside = bodyInside(v, reqs[k]);
                rq.already_has_token = (v.id == holder);
                rq.emergency_or_clear = v.deadlock_breaker;  // §9 破环车临时最高
                rq.starving = v.wait_time > cfg_.starvation_wait_time;
                rq.eta = rq.already_inside
                             ? 0.0
                             : timeToReachS(v, VehicleAction::NOMINAL,
                                            std::max(0.0, reqs[k].s_enter - front));
                if (!has_best || PriorityKey::betterThan(rq, best)) {
                    best = rq;
                    has_best = true;
                    winner_k = static_cast<int>(k);
                }
            }
        }
        // 无人可授予(都出不去)→ 释放令牌,且下面让所有逼近者停在上游(谁都别进)。
        const int winner_id = (winner_k >= 0) ? vehicles[reqs[winner_k].idx].id : -1;
        if (winner_id >= 0) {
            tokens_.grant(rid, winner_id, now_);  // 授予/刷新(防超时)
        } else {
            tokens_.release(rid);
        }

        // 3) 非 winner(及无人可授予时的全部逼近者):在上游停止线让行。只在「按当前
        //    速度+max_decel 即将刹不住」时才发 STOP(§10 停止线;§15 不过早等——离得
        //    远就继续接近,不原地干等)。
        for (size_t k = 0; k < reqs.size(); ++k) {
            if (winner_k >= 0 && static_cast<int>(k) == winner_k) continue;
            VehicleAgent& v = vehicles[reqs[k].idx];
            const double stop_line = reqs[k].s_enter - front;
            const double dist = stop_line - v.path_s;
            const double vc = std::max(0.0, v.current_speed);
            const double brake = vc * vc / (2.0 * decel) + vc * dt;
            if (dist <= brake + 1e-9) {
                const std::string why = (winner_id >= 0)
                    ? ("res_wait_V" + std::to_string(winner_id))
                    : "res_exit_blocked";
                applyActionRequest(v, VehicleAction::STOP, why, winner_id);
            }
        }
    }

    tokens_.expireStale(now_, std::max(2.0, cfg_.prediction_horizon));
}

void RuleEngine::refreshResourceSpans(std::vector<VehicleAgent>& vehicles) {
    if (resmap_ == nullptr) return;
    for (VehicleAgent& v : vehicles) {
        if (v.track.empty()) {
            if (!v.resource_spans.empty()) v.resource_spans.clear();
            v.spans_track_len = -1.0;
            continue;
        }
        // track 长度变化即视为换了新路径(新任务),重算资源占用区间。
        if (std::abs(v.track.length() - v.spans_track_len) > 1e-6) {
            v.resource_spans = resmap_->spansForPath(v.track);
            v.spans_track_len = v.track.length();
        }
    }
}

void RuleEngine::resolveDeadlock(std::vector<VehicleAgent>& vehicles, double dt) {
    // §9/§11.11:用上一周期残留的等待边(blocker_id,本周期 reset 前仍有效)建等待图,
    // 检测环,选破环车给临时最高优先级(deadlock_breaker)。等待图是功能图(每车至多
    // 一条出边=它在等的 blocker),跟指针即可找环。
    constexpr double kBreakerHold = 2.0;  // 破环身份迟滞保持秒数(防闪烁蹭行)
    const size_t n = vehicles.size();
    // 迟滞:保持期未到的破环车继续是破环车(它一动起来就不再 STOP 等待、环检测会消失,
    // 若不保持就会标志闪烁→被旧层反复摁停→蹭行)。保持期到了才清。
    for (VehicleAgent& v : vehicles) {
        v.deadlock_breaker_hold = std::max(0.0, v.deadlock_breaker_hold - dt);
        v.deadlock_breaker = (v.deadlock_breaker_hold > 0.0);
    }

    auto indexOfId = [&](int id) -> int {
        for (size_t i = 0; i < n; ++i)
            if (vehicles[i].id == id) return static_cast<int>(i);
        return -1;
    };
    // 破环车必须选「真正能往前走的那辆」(§16):用车身几何实测——车 i 沿固定路径
    // 前进一小步(probe),车身是否仍与任何其它车的当前车身不重叠。能=它前进可脱困、
    // 抖开环;不能(前方就是对冲车)=放它也是被硬护栏摁死、环永远破不了。这修正了
    // 旧的 min-id 盲选:楔死时常把「动不了的那辆」选成破环车而徒劳。
    auto bodyAtCurrent = [&](const VehicleAgent& v) {
        const double s =
            (v.mode == VehicleMode::DWELL) ? v.track.length() : v.path_s;
        return makeBody(v.track.poseAtS(s), mp_, 0.0);
    };
    auto canStepForward = [&](int i) -> bool {
        const VehicleAgent& v = vehicles[i];
        if (v.track.empty()) return false;
        constexpr double kProbe = 0.10;  // 前探约半身,足以判别前方是否被对冲车堵死
        const double s_next = std::min(v.track.length(), v.path_s + kProbe);
        const OBB body_next = makeBody(v.track.poseAtS(s_next), mp_, 0.0);
        for (size_t j = 0; j < n; ++j) {
            if (static_cast<int>(j) == i) continue;
            const VehicleAgent& o = vehicles[j];
            if (o.track.empty() || o.mode == VehicleMode::NEED_TASK) continue;
            if (overlaps(body_next, bodyAtCurrent(o))) return false;
        }
        return true;
    };
    auto waitEdge = [&](int i) -> int {
        const VehicleAgent& v = vehicles[i];
        if (v.mode != VehicleMode::ACTIVE) return -1;
        if (v.action != VehicleAction::STOP) return -1;  // 只看完全停住的等待
        if (v.blocker_id < 0) return -1;
        return indexOfId(v.blocker_id);
    };

    std::vector<int> state(n, 0);  // 0 未访 1 在栈 2 完成
    for (size_t s = 0; s < n; ++s) {
        if (state[s] != 0) continue;
        std::vector<int> path;
        int cur = static_cast<int>(s);
        while (cur >= 0 && state[cur] == 0) {
            state[cur] = 1;
            path.push_back(cur);
            cur = waitEdge(cur);
        }
        if (cur >= 0 && state[cur] == 1) {  // 找到环:从 cur 首次出现到末尾
            const auto begin = std::find(path.begin(), path.end(), cur);
            // 协调图第5步:严格全序(unifiedPriority)使"谁让谁"关系本应无环 ⇒ 这里检测到
            // 环=无环保证被破坏的 bug 信号(通常是 priorityWinner 里的 slot 资源前置约束
            // 成环,即任务分配出现循环占位)。降为断言:**告警**把它暴露出来。破环逃生暂留
            // 作安全网(确认长跑不再告警后可移除)。
            {
                std::string ring;
                for (auto p = begin; p != path.end(); ++p) {
                    if (p != begin) ring += "->";
                    ring += "V" + std::to_string(vehicles[*p].id);
                }
                ROS_WARN_THROTTLE(
                    5.0,
                    "[DIAG cycle] 等待图检测到环 %s —— 严格全序下本不该出现,"
                    "疑为 slot 资源前置约束成环(循环占位)。暂由破环逃生兜底。",
                    ring.c_str());
            }
            // 选破环车:优先「前进一步能脱困」的车(canStepForward),同类里取 id 最小
            // (确定性,§19)。若环内无人能前进(真·楔死,需倒车的罕见情形),退回 id
            // 最小。只放一辆:其余正常让行停住——绝不会两辆对冲车一起被放而相撞。
            int breaker = -1, best_id = std::numeric_limits<int>::max();
            int breaker_any = -1, best_any = std::numeric_limits<int>::max();
            for (auto p = begin; p != path.end(); ++p) {
                const int vid = vehicles[*p].id;
                if (vid < best_any) { best_any = vid; breaker_any = *p; }
                if (canStepForward(*p) && vid < best_id) {
                    best_id = vid;
                    breaker = *p;
                }
            }
            if (breaker < 0) breaker = breaker_any;  // 无人能前进→退回 min-id
            // 破环逃生已停用(降为纯检测+告警):在「禁止倒车」前提下,前向破环本身是碰撞源
            // ——它豁免让行方刹车、强推它前冲脱困,而前方就是环里别的车 → 直接顶上去(实测
            // V2 brkr=1 被强推顶进卡死的 V7,此后两车贴死、硬护栏每拍触发=那 16857 次"碰撞")。
            // 既然不能倒车,环就应是「静止对峙(楔死)」而非「对撞」:安全优先,先把碰撞降级。
            // 真正出路是从源头不让环形成(出库口/出口检查),见 草履虫规则_协调图统一架构设计。
            (void)breaker; (void)kBreakerHold;
            // vehicles[breaker].deadlock_breaker = true;          // 已停用
            // vehicles[breaker].deadlock_breaker_hold = kBreakerHold;
        }
        for (int p : path) state[p] = 2;
    }
}

void RuleEngine::decide(std::vector<VehicleAgent>& vehicles, double dt) {
    conflicts_.clear();
    now_ += dt;                      // 内部仿真时钟(令牌防抖/超时)
    resolveDeadlock(vehicles, dt);   // Phase4:用上周期等待边检测环、选破环车(reset 前)
    refreshResourceSpans(vehicles);  // Phase 2:刷新每车路径的资源占用缓存

    for (VehicleAgent& v : vehicles) {
        v.blocker_id = -1;
        if (v.mode != VehicleMode::ACTIVE) {
            v.cycle_break_immunity = 0.0;
            v.requested_action = VehicleAction::STOP;
            v.reason = "not_active";
            continue;
        }
        v.requested_action = VehicleAction::NOMINAL;
        v.reason = "clear";
    }

    resolveFollowing(vehicles);
    // 深层根治(用户洞察:路径固定→只信精确几何):停用粗粒度资源盒仲裁(路口/车道
    // 令牌),它把"共用一个路口盒"当冲突造成幻象冲突→打架→死锁。改由精确的 pairwise
    // 几何冲突(findConflictZones:沿固定路径采样车身OBB,只标真实重叠弧段)作唯一交叉
    // 协调权威。八竿子打不着的两车它根本不报冲突→各自全速。
    // arbitrateResources(vehicles, dt);   // 已停用(资源盒=幻象冲突源)
    resolvePairwiseConflicts(vehicles, dt);// 精确几何冲突:交叉协调的唯一权威(§8.6)
    resolveTargetSlotOccupancy(vehicles);  // slot-mouth queueing (spec 6/7)
    enforceForwardClearance(vehicles);     // 普适前向净空兜底:堵分类接缝→防十字楔死
    applyRequestedActions(vehicles, dt);
    if (cfg_.enable_cycle_break) breakDeadlockCycles(vehicles);  // spec 16

    for (VehicleAgent& v : vehicles) {
        if (v.mode != VehicleMode::ACTIVE) continue;  // DWELL/NEED_TASK do not accumulate
        if (v.action == VehicleAction::STOP ||
            v.action == VehicleAction::CREEP ||
            v.action == VehicleAction::YIELD) {
            v.wait_time += dt;
        } else {
            v.wait_time = 0.0;
        }
    }
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
