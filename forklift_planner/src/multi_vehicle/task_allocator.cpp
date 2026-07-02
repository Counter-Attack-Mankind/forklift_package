#include "forklift_planner/multi_vehicle/task_allocator.h"

#include <ros/console.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "forklift_planner/multi_vehicle/footprint.h"

namespace forklift_planner {
namespace multi_vehicle {

namespace {

constexpr double kPi = M_PI;

double normAngle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}

std::string slotLabel(const ForkliftMap& map, int id) {
    const Slot& s = map.slots().at(id);
    return "slot " + std::to_string(s.id) +
           " row=" + std::to_string(s.row_id) +
           " col=" + std::to_string(s.col);
}

}  // namespace


//=======================================（构造函数，用于初始化）========================
TaskAllocator::TaskAllocator(const MapParam& mp, const PlannerParam& pp,
                             const MultiVehicleConfig& cfg,
                             const ForkliftMap& map,
                             PathGenerator& generator)
    : mp_(mp), pp_(pp), cfg_(cfg), map_(map), generator_(generator)
{
    const int n = static_cast<int>(map_.slots().size());
    target_visit_counts_.assign(n, 0);
    edge_visit_counts_.assign(n * n, 0);
    row_visit_counts_.assign(8, 0);
    outbound_valid_counts_.assign(n, 0);
    outbound_cross_row_counts_.assign(n, 0);
}

const char* TaskAllocator::rejectReasonName(TaskRejectReason reason) const {
    switch (reason) {
        case TaskRejectReason::NONE: return "none";
        case TaskRejectReason::SAME_SLOT: return "same_slot";
        case TaskRejectReason::EMPTY_PATH: return "empty_path";
        case TaskRejectReason::CURVATURE_DISCONTINUITY:
            return "curvature_discontinuity";
        case TaskRejectReason::FOOTPRINT_OUT_OF_BOUNDS:
            return "footprint_out_of_bounds";
        case TaskRejectReason::SHELF_COLLISION: return "shelf_collision";
        case TaskRejectReason::KINK: return "kink";
    }
    return "unknown";
}

TaskPlanCache& TaskAllocator::cacheAt(int src, int target) {
    const int n = static_cast<int>(map_.slots().size());
    return task_cache_[src * n + target];
}

const TaskPlanCache& TaskAllocator::cacheAt(int src, int target) const {
    const int n = static_cast<int>(map_.slots().size());
    return task_cache_[src * n + target];
}

//===============================（路径合法性生成检查）================================
bool TaskAllocator::poseInSlotSweep(const RoughWp& pose,
                                    const Slot& slot) const {
    const double ax = slot.dock_x();
    const double ay = slot.dock_y();
    const double bx = slot.pre_dock_x;
    const double by = slot.pre_dock_y;
    const double vx = bx - ax;
    const double vy = by - ay;
    const double len = std::hypot(vx, vy);
    if (len < 1e-6) return false;

    const double ux = vx / len;
    const double uy = vy / len;
    const double dx = pose.x - ax;
    const double dy = pose.y - ay;
    const double longitudinal = dx * ux + dy * uy;
    const double lateral = -dx * uy + dy * ux;

    const double long_margin = mp_.vehicle_length * 0.65;
    const double lat_margin = mp_.vehicle_width * 0.60;
    return longitudinal >= -long_margin &&
           longitudinal <= len + long_margin &&
           std::abs(lateral) <= lat_margin;
}

bool TaskAllocator::footprintCollidesWithShelf(const RoughWp& pose,
                                               const Slot& src,
                                               const Slot& target) const {
    if (poseInSlotSweep(pose, src) || poseInSlotSweep(pose, target)) {
        return false;
    }
    for (const ShelfBlock& shelf : map_.shelf_blocks()) {
        if (footprintIntersectsShelf(pose, shelf, mp_, 0.0)) {
            return true;
        }
    }
    return false;
}

RoughWp TaskAllocator::interpolatePose(const RoughWp& a, const RoughWp& b,
                                       double ratio) const {
    RoughWp p;
    p.x = a.x + (b.x - a.x) * ratio;
    p.y = a.y + (b.y - a.y) * ratio;
    p.theta = normAngle(a.theta + normAngle(b.theta - a.theta) * ratio);
    p.type = ratio < 0.5 ? a.type : b.type;
    return p;
}

TaskRejectReason TaskAllocator::validatePath(const RoughPath& path,
                                             const PathGenerationInfo& info,
                                             const Slot& src,
                                             const Slot& target) const {
    if (path.size() < 2) {
        return TaskRejectReason::EMPTY_PATH;
    }
    if (cfg_.reject_curvature_discontinuity && info.used_arc_fallback) {
        return TaskRejectReason::CURVATURE_DISCONTINUITY;
    }

    if (cfg_.reject_path_kinks) {
        // Compare consecutive non-zero segments. Path assembly can leave two
        // waypoints at the same corner; checking waypoint triples directly
        // skips both sides of such a duplicate and lets a 90-degree corner
        // through. Keeping the previous non-zero segment closes that gap.
        bool have_prev = false;
        double prev_dx = 0.0;
        double prev_dy = 0.0;
        double prev_len = 0.0;
        WpType prev_type = path.front().type;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            const double dx = path[i + 1].x - path[i].x;
            const double dy = path[i + 1].y - path[i].y;
            const double len = std::hypot(dx, dy);
            if (len < 1e-4) continue;

            const WpType type = path[i + 1].type;
            if (!have_prev) {
                prev_dx = dx;
                prev_dy = dy;
                prev_len = len;
                prev_type = type;
                have_prev = true;
                continue;
            }

            double c = (prev_dx * dx + prev_dy * dy) / (prev_len * len);
            c = std::max(-1.0, std::min(1.0, c));
            const double ang = std::acos(c);  // 0=straight, pi=clean cusp
            const bool legal_reverse_cusp =
                prev_type != type && ang >= cfg_.kink_cusp_angle;
            if (ang > cfg_.kink_min_angle && !legal_reverse_cusp) {
                return TaskRejectReason::KINK;
            }

            prev_dx = dx;
            prev_dy = dy;
            prev_len = len;
            prev_type = type;
        }
    }

    const double check_ds = std::max(0.005, cfg_.path_validation_step);
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const double seg_len = std::hypot(path[i + 1].x - path[i].x,
                                          path[i + 1].y - path[i].y);
        const int steps =
            std::max(1, static_cast<int>(std::ceil(seg_len / check_ds)));
        for (int k = 0; k < steps; ++k) {
            const double ratio =
                static_cast<double>(k) / static_cast<double>(steps);
            const TaskRejectReason reason =
                validatePose(interpolatePose(path[i], path[i + 1], ratio),
                             src, target);
            if (reason != TaskRejectReason::NONE) return reason;
        }
    }
    return validatePose(path.back(), src, target);
}

TaskRejectReason TaskAllocator::validatePose(const RoughWp& pose,
                                             const Slot& src,
                                             const Slot& target) const {
    if (cfg_.reject_boundary_violations &&
        !footprintInsideField(pose, mp_, 0.0)) {
        return TaskRejectReason::FOOTPRINT_OUT_OF_BOUNDS;
    }
    if (cfg_.reject_shelf_collisions &&
        footprintCollidesWithShelf(pose, src, target)) {
        return TaskRejectReason::SHELF_COLLISION;
    }
    return TaskRejectReason::NONE;
}

//===============================================（任务生成）=============================

TaskPlanCache TaskAllocator::makeTaskPlan(int src_id, int target_id) const {
    TaskPlanCache out;
    if (src_id == target_id) {
        out.reject_reason = TaskRejectReason::SAME_SLOT;
        return out;
    }

    const Slot& src = map_.slots().at(src_id);
    const Slot& target = map_.slots().at(target_id);
    out.path = generator_.generate(src, target, &out.info);
    out.reject_reason = validatePath(out.path, out.info, src, target);
    out.valid = out.reject_reason == TaskRejectReason::NONE;
    return out;
}

TaskPlanCache TaskAllocator::makeTaskPlanFromPose(const VehicleAgent& vehicle,
                                                  int target_id) const {
    TaskPlanCache out;
    if (target_id == vehicle.current_slot) {
        out.reject_reason = TaskRejectReason::SAME_SLOT;
        return out;
    }

    const Slot& src_slot = map_.slots().at(vehicle.current_slot);
    const Slot& target = map_.slots().at(target_id);
    const RoughWp p = vehicleCurrentPose(vehicle);
    double theta = p.theta;
    if (!vehicle.track.empty() &&
        vehicle.track.typeAtS(std::min(vehicle.path_s, vehicle.track.length())) == WpType::REVERSE) {
        theta = normAngle(theta + kPi);
    }

    out.path = generator_.generateFromPose(p.x, p.y, theta, target, &out.info);
    out.reject_reason = validatePath(out.path, out.info, src_slot, target);
    out.valid = out.reject_reason == TaskRejectReason::NONE;
    return out;
}

void TaskAllocator::buildCache() {
    const int n = static_cast<int>(map_.slots().size());
    task_cache_.assign(n * n, TaskPlanCache{});
    std::fill(outbound_valid_counts_.begin(), outbound_valid_counts_.end(), 0);
    std::fill(outbound_cross_row_counts_.begin(),
              outbound_cross_row_counts_.end(), 0);

    const bool quiet_planner_logs =
        cfg_.quiet_task_filter_precompute && !cfg_.log_invalid_task_pairs;
    if (quiet_planner_logs) {
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Error);
        ros::console::notifyLoggerLevelsChanged();
    }

    int valid = 0;
    std::array<int, 7> rejected{};
    std::array<int, 8> valid_target_rows{};
    for (int src = 0; src < n; ++src) {
        for (int target = 0; target < n; ++target) {
            TaskPlanCache plan = makeTaskPlan(src, target);
            if (plan.valid) {
                ++valid;
                ++outbound_valid_counts_[src];
                if (map_.slots().at(src).row_id != map_.slots().at(target).row_id) {
                    ++outbound_cross_row_counts_[src];
                }
                const int row = map_.slots().at(target).row_id;
                if (row >= 0 && row < static_cast<int>(valid_target_rows.size())) {
                    ++valid_target_rows[row];
                }
            } else {
                const int idx = static_cast<int>(plan.reject_reason);
                if (idx >= 0 && idx < static_cast<int>(rejected.size())) {
                    ++rejected[idx];
                }
                if (cfg_.log_invalid_task_pairs &&
                    plan.reject_reason != TaskRejectReason::SAME_SLOT) {
                    ROS_INFO("[multi_patrol] filtered task %d -> %d: %s",
                             src, target, rejectReasonName(plan.reject_reason));
                }
            }
            cacheAt(src, target) = std::move(plan);
        }
    }

    if (quiet_planner_logs) {
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Info);
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_INFO("[multi_patrol] task cache built: slots=%d pairs=%d valid=%d "
             "same=%d empty=%d curvature=%d boundary=%d shelf=%d kink=%d "
             "valid_target_rows=[%d,%d,%d,%d,%d,%d,%d,%d]",
             n, n * n, valid,
             rejected[static_cast<int>(TaskRejectReason::SAME_SLOT)],
             rejected[static_cast<int>(TaskRejectReason::EMPTY_PATH)],
             rejected[static_cast<int>(TaskRejectReason::CURVATURE_DISCONTINUITY)],
             rejected[static_cast<int>(TaskRejectReason::FOOTPRINT_OUT_OF_BOUNDS)],
             rejected[static_cast<int>(TaskRejectReason::SHELF_COLLISION)],
             rejected[static_cast<int>(TaskRejectReason::KINK)],
             valid_target_rows[0], valid_target_rows[1], valid_target_rows[2],
             valid_target_rows[3], valid_target_rows[4], valid_target_rows[5],
             valid_target_rows[6], valid_target_rows[7]);

    // [DIAG] Trap slots: sources with zero valid outbound paths. A vehicle that
    // starts or parks here can never leave. Log the reject-reason breakdown so
    // we can tell a genuine geometric dead-end from a generator defect.
    for (int src = 0; src < n; ++src) {
        int valid_out = 0;
        std::array<int, 7> hist{};
        for (int target = 0; target < n; ++target) {
            const TaskPlanCache& c = cacheAt(src, target);
            if (c.valid) { ++valid_out; continue; }
            const int idx = static_cast<int>(c.reject_reason);
            if (idx >= 0 && idx < 7) ++hist[idx];
        }
        if (valid_out == 0) {
            ROS_WARN("[DIAG trap] %s: 0 valid outbound | same=%d empty=%d "
                     "curv=%d bound=%d shelf=%d kink=%d",
                     slotLabel(map_, src).c_str(),
                     hist[1], hist[2], hist[3], hist[4], hist[5], hist[6]);
        }
    }

    // [DIAG spin] 只读诊断:扫描每条有效路径相邻点的车身朝向突变。密集采样下(步长
    // ~turn_ds),正常拐弯每步朝向变化仅约几度;相邻两点朝向突变 > kSpinThresh 即为
    // "中段瞬间旋转"(车身硬扭),无论是否伴随 forward/reverse 类型切换——合法 cusp
    // 处车身朝向应连续(只翻运动方向)。打印 slot 对、点序号、Δ朝向、前后朝向/类型、
    // 坐标,精确定位后再决定如何修(终点/出库朝向自由选 θ/θ+π 以保连续)。不改任何行为。
    constexpr double kSpinThresh = 0.9;  // rad ≈ 52°,远大于单步正常拐弯角
    auto angDiff = [](double a, double b) {
        return std::abs(std::atan2(std::sin(b - a), std::cos(b - a)));
    };
    int spin_paths = 0, spin_points = 0;
    double worst = 0.0; int worst_src = -1, worst_tgt = -1;
    bool dumped_window = false;
    for (int src = 0; src < n; ++src) {
        for (int target = 0; target < n; ++target) {
            const TaskPlanCache& c = cacheAt(src, target);
            if (!c.valid || c.path.size() < 2) continue;
            bool path_has_spin = false;
            // [DIAG window] 对第一条出问题路径转储接缝附近的原始 waypoint
            if (!dumped_window) {
                for (size_t i = 1; i < c.path.size(); ++i) {
                    if (angDiff(c.path[i-1].theta, c.path[i].theta) <= kSpinThresh)
                        continue;
                    const size_t lo = i >= 4 ? i - 4 : 0;
                    const size_t hi = std::min(c.path.size(), i + 4);
                    for (size_t k = lo; k < hi; ++k) {
                        const RoughWp& w = c.path[k];
                        ROS_WARN("[DIAG window] %s->%s [%zu]%s th=%.1f (%.4f,%.4f)%s",
                                 slotLabel(map_, src).c_str(),
                                 slotLabel(map_, target).c_str(), k,
                                 w.type == WpType::REVERSE ? "R" : "F",
                                 w.theta * 180.0 / M_PI, w.x, w.y,
                                 k == i ? "  <-- JUMP here" : "");
                    }
                    dumped_window = true;
                    break;
                }
            }
            for (size_t i = 1; i < c.path.size(); ++i) {
                const RoughWp& p0 = c.path[i - 1];
                const RoughWp& p1 = c.path[i];
                const double dth = angDiff(p0.theta, p1.theta);
                if (dth <= kSpinThresh) continue;
                ++spin_points;
                path_has_spin = true;
                const double ds = std::hypot(p1.x - p0.x, p1.y - p0.y);
                if (dth > worst) { worst = dth; worst_src = src; worst_tgt = target; }
                ROS_WARN("[DIAG spin] %s->%s i=%zu/%zu d_th=%.0fdeg ds=%.3f "
                         "th:%.0f(%s)->%.0f(%s) @(%.3f,%.3f)->(%.3f,%.3f)",
                         slotLabel(map_, src).c_str(),
                         slotLabel(map_, target).c_str(),
                         i, c.path.size(), dth * 180.0 / M_PI, ds,
                         p0.theta * 180.0 / M_PI,
                         p0.type == WpType::REVERSE ? "R" : "F",
                         p1.theta * 180.0 / M_PI,
                         p1.type == WpType::REVERSE ? "R" : "F",
                         p0.x, p0.y, p1.x, p1.y);
            }
            if (path_has_spin) ++spin_paths;
        }
    }
    ROS_INFO("[spin-check] %d valid paths have mid-path heading jumps "
             ">%.0fdeg (%d jump points). worst=%.0fdeg at %s->%s",
             spin_paths, kSpinThresh * 180.0 / M_PI, spin_points,
             worst * 180.0 / M_PI,
             worst_src >= 0 ? slotLabel(map_, worst_src).c_str() : "-",
             worst_tgt >= 0 ? slotLabel(map_, worst_tgt).c_str() : "-");
}

bool TaskAllocator::containsRecent(const std::vector<int>& values,
                                   int value) const {
    return std::find(values.begin(), values.end(), value) != values.end();
}

double TaskAllocator::deterministicJitter(const VehicleAgent& vehicle,
                                          int target) const {
    unsigned int x = static_cast<unsigned int>(
        (vehicle.id + 1) * 73856093u ^
        (vehicle.task_count + 1) * 19349663u ^
        (target + 1) * 83492791u ^
        (cfg_.random_seed + 1) * 2654435761u);
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return static_cast<double>(x % 1000u) / 1000.0;
}

int TaskAllocator::activeTargetCount(const VehicleAgent& vehicle,
                                     const std::vector<VehicleAgent>& all,
                                     int target) const {
    int count = 0;
    for (const VehicleAgent& other : all) {
        if (other.id == vehicle.id) continue;
        if ((other.mode == VehicleMode::ACTIVE ||
             other.mode == VehicleMode::DWELL) &&
            other.target_slot == target) {
            ++count;
        }
    }
    return count;
}

bool TaskAllocator::hasValidOutbound(int slot) const {
    if (!cfg_.precompute_task_filter) return true;  // unknown without cache
    if (slot < 0 ||
        slot >= static_cast<int>(outbound_valid_counts_.size())) {
        return true;
    }
    return outbound_valid_counts_[slot] > 0;
}

RoughWp TaskAllocator::vehicleCurrentPose(const VehicleAgent& v) const {
    if (!v.track.empty()) {
        // 物理当前位姿一律按 path_s 取（轨迹上车实际走到的弧长）。
        // 不能对 DWELL 特判成 track.length()：泊着的车一旦已排好下一程
        // 出库轨迹（path_s=0、track 是 35→63 这种），length() 会返回「未来
        // 目的地」而非当前所在库位，导致 slotReservedByOther 误判该库为空、
        // 把它派给别的车，引发库口交接对撞。arrived-DWELL 时 path_s 本就=length，
        // 取 path_s 同样正确。
        const double s = std::min(v.path_s, v.track.length());
        return v.track.poseAtS(s);
    }
    const Slot& s = map_.slots().at(v.current_slot);
    const double th = std::atan2(s.pre_dock_y - s.dock_y(),
                                 s.pre_dock_x - s.dock_x());
    RoughWp p;
    p.x = s.dock_x() - mp_.rear_axle_to_center * std::cos(th);
    p.y = s.dock_y() - mp_.rear_axle_to_center * std::sin(th);
    p.theta = th;
    p.type = WpType::FORWARD;
    return p;
}

// 库位唯一资源规则：已被别的车「预约为目的地」或「物理占着」时不再分配。
bool TaskAllocator::slotReservedByOther(const VehicleAgent& vehicle,
                                        const std::vector<VehicleAgent>& all,
                                        int slot) const {
    for (const VehicleAgent& o : all) {
        if (o.id == vehicle.id) continue;
        if (o.target_slot == slot &&
            (o.mode == VehicleMode::ACTIVE || o.mode == VehicleMode::DWELL)) {
            return true;
        }
        if (o.current_slot == slot &&
            poseInSlotSweep(vehicleCurrentPose(o), map_.slots().at(slot))) {
            return true;
        }
    }
    return false;
}

bool TaskAllocator::planAvailable(const VehicleAgent& vehicle, int target,
                                  bool require_no_arc) const {
    if (target == vehicle.current_slot) return false;
    if (cfg_.precompute_task_filter) {
        const TaskPlanCache& cached = cacheAt(vehicle.current_slot, target);
        if (!cached.valid) return false;
        return !require_no_arc || !cached.info.used_arc_fallback;
    }

    TaskPlanCache plan = makeTaskPlan(vehicle.current_slot, target);
    if (!plan.valid) return false;
    return !require_no_arc || !plan.info.used_arc_fallback;
}

int TaskAllocator::chooseNextTarget(const VehicleAgent& vehicle,
                                    const std::vector<VehicleAgent>& all,
                                    bool require_no_arc) const {
    const int n = static_cast<int>(map_.slots().size());
    const Slot& current = map_.slots().at(vehicle.current_slot);
    int best_target = -1;
    double best_score = -std::numeric_limits<double>::infinity();

    for (int pass = 0; pass < 2; ++pass) {
        const bool allow_same_row = pass == 1;
        for (int target = 0; target < n; ++target) {
            if (!planAvailable(vehicle, target, require_no_arc)) continue;
            if (!hasValidOutbound(target)) continue;
            if (slotReservedByOther(vehicle, all, target)) continue;

            const Slot& dst = map_.slots().at(target);
            if (!allow_same_row && dst.row_id == current.row_id) continue;
            const int row = std::max(0, std::min(dst.row_id, 7));
            const int edge_idx = vehicle.current_slot * n + target;
            const int row_visits =
                row < static_cast<int>(row_visit_counts_.size())
                    ? row_visit_counts_[row] : 0;

            double score = 0.0;
            score += 8.0 * std::abs(dst.row_id - current.row_id);
            score += 1.5 * std::abs(dst.col - current.col);
            if (target < static_cast<int>(outbound_cross_row_counts_.size())) {
                score += 2.0 * outbound_cross_row_counts_[target];
                score += 0.4 * outbound_valid_counts_[target];
                if (outbound_cross_row_counts_[target] == 0) score -= 35.0;
            }
            score -= 18.0 * target_visit_counts_[target];
            score -= 14.0 * row_visits;
            score -= 25.0 * edge_visit_counts_[edge_idx];
            if (containsRecent(vehicle.recent_targets, target)) score -= 80.0;
            if (containsRecent(vehicle.recent_rows, dst.row_id)) score -= 24.0;
            if (dst.row_id == 7) score += 55.0;
            if (dst.row_id == 0) score += 18.0;

            score -= 12.0 * activeTargetCount(vehicle, all, target);
            score += deterministicJitter(vehicle, target) * 6.0;

            if (score > best_score) {
                best_score = score;
                best_target = target;
            }
        }
        if (best_target >= 0) break;
    }
    return best_target;
}

void TaskAllocator::rememberTask(VehicleAgent& vehicle, int target) {
    const int n = static_cast<int>(map_.slots().size());
    const int row = map_.slots().at(target).row_id;
    if (target >= 0 && target < static_cast<int>(target_visit_counts_.size())) {
        ++target_visit_counts_[target];
    }
    if (row >= 0 && row < static_cast<int>(row_visit_counts_.size())) {
        ++row_visit_counts_[row];
    }
    const int edge_idx = vehicle.current_slot * n + target;
    if (edge_idx >= 0 && edge_idx < static_cast<int>(edge_visit_counts_.size())) {
        ++edge_visit_counts_[edge_idx];
    }

    vehicle.recent_targets.push_back(target);
    if (vehicle.recent_targets.size() >
        static_cast<size_t>(std::max(0, cfg_.recent_target_memory))) {
        vehicle.recent_targets.erase(vehicle.recent_targets.begin());
    }
    vehicle.recent_rows.push_back(row);
    if (vehicle.recent_rows.size() >
        static_cast<size_t>(std::max(0, cfg_.recent_row_memory))) {
        vehicle.recent_rows.erase(vehicle.recent_rows.begin());
    }
}

bool TaskAllocator::tryPlan(VehicleAgent& vehicle, int target,
                            bool require_no_arc) {
    if (target == vehicle.current_slot) return false;

    TaskPlanCache plan;
    const bool continue_from_current_pose =
        !vehicle.track.empty() &&
        vehicle.mode == VehicleMode::ACTIVE &&
        vehicle.path_s < vehicle.track.length() - 1e-6;
    if (continue_from_current_pose) {
        plan = makeTaskPlanFromPose(vehicle, target);
        if (!plan.valid) return false;
    } else if (cfg_.precompute_task_filter) {
        const TaskPlanCache& cached = cacheAt(vehicle.current_slot, target);
        if (!cached.valid) return false;
        plan = cached;
    } else {
        plan = makeTaskPlan(vehicle.current_slot, target);
        if (!plan.valid) return false;
    }
    if (require_no_arc && plan.info.used_arc_fallback) return false;

    rememberTask(vehicle, target);
    vehicle.target_slot = target;
    vehicle.track.set(plan.path);
    ++vehicle.path_gen;  // 新固定路径实例 → 令 RuleEngine 的 C_ij 冲突块缓存失效重算
    vehicle.path_s = 0.0;
    vehicle.current_speed = 0.0;
    vehicle.wait_time = 0.0;
    vehicle.dwell_remaining = 0.0;
    vehicle.mode = vehicle.track.empty() ? VehicleMode::NEED_TASK
                                         : VehicleMode::ACTIVE;
    vehicle.action = VehicleAction::NOMINAL;
    vehicle.requested_action = VehicleAction::NOMINAL;
    vehicle.reason = "new_task";
    if (vehicle.mode != VehicleMode::ACTIVE) return false;

    ROS_INFO("[multi_patrol] V%d task %d: %s -> %s  wpts=%zu len=%.3f arc=%d",
             vehicle.id, vehicle.task_count,
             slotLabel(map_, vehicle.current_slot).c_str(),
             slotLabel(map_, vehicle.target_slot).c_str(),
             vehicle.track.path().size(), vehicle.track.length(),
             plan.info.used_arc_fallback ? 1 : 0);

    // [DIAG] Scan the drawn path for a geometric kink: at each interior waypoint
    // measure the turn angle between the incoming and outgoing POSITION deltas
    // (not theta). A smooth clothoid/arc turns only ~2deg per sample, so any
    // single-vertex turn > ~40deg is a real kink. ~90deg = right-angle corner,
    // ~180deg = reverse cusp (we report both so they can be told apart).
    {
        const RoughPath& wp = vehicle.track.path();
        double worst_ang = 0.0;
        size_t worst_i = 0;
        for (size_t i = 1; i + 1 < wp.size(); ++i) {
            const double d1x = wp[i].x - wp[i - 1].x;
            const double d1y = wp[i].y - wp[i - 1].y;
            const double d2x = wp[i + 1].x - wp[i].x;
            const double d2y = wp[i + 1].y - wp[i].y;
            const double n1 = std::hypot(d1x, d1y);
            const double n2 = std::hypot(d2x, d2y);
            if (n1 < 1e-4 || n2 < 1e-4) continue;
            double c = (d1x * d2x + d1y * d2y) / (n1 * n2);
            c = std::max(-1.0, std::min(1.0, c));
            const double ang = std::acos(c);  // 0=straight, pi=reversal
            if (ang > worst_ang) { worst_ang = ang; worst_i = i; }
        }
        if (worst_ang > 0.70) {  // > ~40 deg single-vertex turn = kink
            const size_t i = worst_i;
            const bool cusp = (wp[i - 1].type != wp[i].type) ||
                              (wp[i].type != wp[i + 1].type);
            ROS_WARN("[DIAG kink] V%d %s -> %s : turn=%.1fdeg at wp%zu/%zu "
                     "p=(%.3f,%.3f) types=%d/%d/%d %s",
                     vehicle.id,
                     slotLabel(map_, vehicle.current_slot).c_str(),
                     slotLabel(map_, vehicle.target_slot).c_str(),
                     worst_ang * 180.0 / kPi, i, wp.size(),
                     wp[i].x, wp[i].y,
                     static_cast<int>(wp[i - 1].type),
                     static_cast<int>(wp[i].type),
                     static_cast<int>(wp[i + 1].type),
                     cusp ? "(reverse-cusp)" : "(FORWARD-corner)");
        }
    }
    return true;
}

// 简单测试版:在所有可达目标里,选「生成路径全程前进(无 REVERSE 段=无尖点)且路径最短」的那个。
// 用路径弧长(而非直线距离)排序,忠实"走得近"。覆盖跨排/分散选靶;无前进可达目标则返回 -1。
int TaskAllocator::chooseNearestForwardTarget(const VehicleAgent& vehicle,
                                              const std::vector<VehicleAgent>& all) const {
    double len = 0.0;
    return nearestForwardTargetLen(vehicle, all, len);
}

int TaskAllocator::nearestForwardTargetLen(const VehicleAgent& vehicle,
                                           const std::vector<VehicleAgent>& all,
                                           double& out_len) const {
    const int n = static_cast<int>(map_.slots().size());
    int best = -1;
    double best_len = std::numeric_limits<double>::infinity();
    for (int t = 0; t < n; ++t) {
        if (t == vehicle.current_slot) continue;
        if (!hasValidOutbound(t)) continue;                 // 别派进出不来的陷阱位
        if (slotReservedByOther(vehicle, all, t)) continue; // 已被别车预约/占用
        TaskPlanCache plan;
        if (cfg_.precompute_task_filter) plan = cacheAt(vehicle.current_slot, t);
        else plan = makeTaskPlan(vehicle.current_slot, t);
        if (!plan.valid) continue;
        if (cfg_.skip_arc_fallback_paths && plan.info.used_arc_fallback) continue;
        // 全程前进:任一航点是 REVERSE 段就淘汰(出库倒车或倒车泊入都会有 REVERSE → 即尖点来源)。
        bool has_reverse = false;
        double len = 0.0;
        for (size_t i = 0; i < plan.path.size(); ++i) {
            if (plan.path[i].type == WpType::REVERSE) { has_reverse = true; break; }
            if (i > 0) len += std::hypot(plan.path[i].x - plan.path[i - 1].x,
                                         plan.path[i].y - plan.path[i - 1].y);
        }
        if (has_reverse) continue;
        if (len < best_len) { best_len = len; best = t; }
    }
    out_len = (best >= 0) ? best_len : 0.0;
    return best;
}

// 该库位的全部「全程前进(无 REVERSE 段)」可达目标(不看他车预约,仅几何/缓存可行性)。
// out_len 非空时,同时输出每个目标的实际路径弧长(供按"走得近"挑对)。
void TaskAllocator::forwardTargets(int slot, std::vector<int>& out,
                                   std::vector<double>* out_len) const {
    out.clear();
    if (out_len) out_len->clear();
    const int n = static_cast<int>(map_.slots().size());
    for (int t = 0; t < n; ++t) {
        if (t == slot) continue;
        TaskPlanCache plan;
        if (cfg_.precompute_task_filter) plan = cacheAt(slot, t);
        else plan = makeTaskPlan(slot, t);
        if (!plan.valid) continue;
        if (cfg_.skip_arc_fallback_paths && plan.info.used_arc_fallback) continue;
        bool has_reverse = false;
        double len = 0.0;
        for (size_t i = 0; i < plan.path.size(); ++i) {
            if (plan.path[i].type == WpType::REVERSE) { has_reverse = true; break; }
            if (i > 0) len += std::hypot(plan.path[i].x - plan.path[i - 1].x,
                                         plan.path[i].y - plan.path[i - 1].y);
        }
        if (has_reverse) continue;
        out.push_back(t);
        if (out_len) out_len->push_back(len);
    }
}

// 该库位是否至少有一个「全程前进(无 REVERSE 段)」可达目标。
bool TaskAllocator::hasForwardTarget(int slot) const {
    std::vector<int> ft; forwardTargets(slot, ft); return !ft.empty();
}

bool TaskAllocator::assignNextTask(VehicleAgent& vehicle,
                                   const std::vector<VehicleAgent>& all) {
    const bool prefer_no_arc = cfg_.skip_arc_fallback_paths;
    // 简单测试版:优先用预设终点(target_slots[id]),校验是「全程前进」目标且未被预约;否则自动选最近
    // 前进目标。一把进库、不绕远;都选不到就停着等(不退回跨排选靶)。
    if (cfg_.simple_forward_demo) {
        int t = -1;
        if (vehicle.id >= 0 &&
            vehicle.id < static_cast<int>(cfg_.target_slots.size())) {
            const int pt = cfg_.target_slots[vehicle.id];
            if (pt >= 0 && pt != vehicle.current_slot &&
                !slotReservedByOther(vehicle, all, pt)) {
                std::vector<int> ft; forwardTargets(vehicle.current_slot, ft);
                if (std::find(ft.begin(), ft.end(), pt) != ft.end()) t = pt;
                else ROS_WARN("[multi_patrol][simple] V%d 预设终点 %d 非全程前进目标 → 改自动选最近前进",
                              vehicle.id, pt);
            }
        }
        if (t < 0) t = chooseNearestForwardTarget(vehicle, all);
        if (t >= 0 && tryPlan(vehicle, t, prefer_no_arc)) {
            ROS_INFO("[multi_patrol][simple] V%d 选最近前进目标 %s len=%.3f (无尖点一把进)",
                     vehicle.id, slotLabel(map_, t).c_str(), vehicle.track.length());
            return true;
        }
        vehicle.mode = VehicleMode::NEED_TASK;
        vehicle.action = VehicleAction::STOP;
        vehicle.requested_action = VehicleAction::STOP;
        vehicle.reason = "no_forward_task";
        ROS_ERROR("[multi_patrol][simple] V%d 从 %s 找不到全程前进的目标(无尖点路径)",
                  vehicle.id, slotLabel(map_, vehicle.current_slot).c_str());
        return false;
    }
    int target = chooseNextTarget(vehicle, all, prefer_no_arc);
    if (vehicle.task_count == 0 &&
        vehicle.id >= 0 &&
        vehicle.id < static_cast<int>(cfg_.target_slots.size())) {
        const int pt = cfg_.target_slots[vehicle.id];
        const bool target_in_range =
            pt >= 0 && pt < static_cast<int>(map_.slots().size());
        if (target_in_range && pt != vehicle.current_slot &&
            !slotReservedByOther(vehicle, all, pt) &&
            planAvailable(vehicle, pt, prefer_no_arc)) {
            target = pt;
        } else if (pt >= 0 && (!target_in_range || pt != vehicle.current_slot)) {
            const std::string target_label =
                target_in_range ? slotLabel(map_, pt)
                                : ("slot " + std::to_string(pt));
            ROS_WARN("[multi_patrol] V%d preset first target %s is unavailable "
                     "from %s; choosing an automatic target instead",
                     vehicle.id, target_label.c_str(),
                     slotLabel(map_, vehicle.current_slot).c_str());
        }
    }
    if (target >= 0 && tryPlan(vehicle, target, prefer_no_arc)) {
        return true;
    }

    if (prefer_no_arc && !cfg_.reject_curvature_discontinuity) {
        ROS_WARN("[multi_patrol] V%d cannot find a no-fallback path from %s; "
                 "allowing fallback to keep patrol moving",
                 vehicle.id, slotLabel(map_, vehicle.current_slot).c_str());
        target = chooseNextTarget(vehicle, all, false);
        if (target >= 0 && tryPlan(vehicle, target, false)) {
            return true;
        }
    }

    vehicle.mode = VehicleMode::NEED_TASK;
    vehicle.action = VehicleAction::STOP;
    vehicle.requested_action = VehicleAction::STOP;
    vehicle.reason = "no_valid_task";
    ROS_ERROR("[multi_patrol] V%d has no valid next task from %s",
              vehicle.id, slotLabel(map_, vehicle.current_slot).c_str());
    return false;
}

bool TaskAllocator::replanFromPose(VehicleAgent& vehicle,
                                   const std::vector<VehicleAgent>& all) {
    // 死锁恢复(C):从卡死车的「当前实际位姿」重规划到一个空库位,使其驶离争用区。
    if (vehicle.track.empty()) return false;
    constexpr double kPi = 3.14159265358979323846;
    const RoughWp p = vehicle.track.poseAtS(vehicle.path_s);
    double theta = p.theta;  // 取车身行进朝向(REVERSE 段 +π),保证新路径从该朝向起步
    if (vehicle.track.typeAtS(vehicle.path_s) == WpType::REVERSE) theta += kPi;

    // 「别往堵着的口子钻」:候选目标里,选第一条「起步段(前 ~1.5m)不压上任何他车车身」的
    // 路径——避开与他车在库位口对穿的路(死锁恢复反复换到又对穿的路=churn 根因)。
    auto bodyNow = [&](const VehicleAgent& o) {
        const double os = (o.mode == VehicleMode::DWELL) ? o.track.length() : o.path_s;
        return makeBody(o.track.poseAtS(os), mp_, 0.0);
    };
    auto startClear = [&](const RoughPath& path) -> bool {
        double acc = 0.0;
        for (size_t i = 0; i < path.size(); ++i) {
            if (i > 0) acc += std::hypot(path[i].x - path[i - 1].x,
                                         path[i].y - path[i - 1].y);
            if (acc > 1.5) break;  // 只查起步段
            const OBB vb = makeBody(path[i], mp_, 0.0);
            for (const VehicleAgent& o : all) {
                if (o.id == vehicle.id || o.track.empty() ||
                    o.mode == VehicleMode::NEED_TASK) continue;
                if (overlaps(vb, bodyNow(o))) return false;
            }
        }
        return true;
    };
    auto slotTaken = [&](int t) {
        for (const VehicleAgent& o : all)
            if (o.id != vehicle.id &&
                (o.target_slot == t || o.current_slot == t)) return true;
        return false;
    };

    // 候选序:chooseNextTarget 首选打头,其余空库位兜底。
    std::vector<int> cands;
    const int pref = chooseNextTarget(vehicle, all, false);
    if (pref >= 0) cands.push_back(pref);
    for (int t = 0; t < static_cast<int>(map_.slots().size()); ++t) cands.push_back(t);

    int target = -1;
    RoughPath path;
    PathGenerationInfo info;
    for (int t : cands) {
        if (t < 0 || t == vehicle.current_slot || slotTaken(t)) continue;
        RoughPath cp = generator_.generateFromPose(p.x, p.y, theta,
                                                   map_.slots().at(t), &info);
        if (cp.size() < 2) continue;
        if (std::hypot(cp.front().x - p.x, cp.front().y - p.y) > 0.05) continue;
        if (!startClear(cp)) continue;  // 起步段会撞他车 → 这条对穿,换
        target = t; path = std::move(cp); break;
    }
    if (target < 0) return false;  // 所有方向都被堵(真·密集包围)→ 本拍救不了

    rememberTask(vehicle, target);
    vehicle.target_slot = target;
    vehicle.track.set(path);
    ++vehicle.path_gen;
    vehicle.path_s = 0.0;
    vehicle.current_speed = 0.0;
    vehicle.wait_time = 0.0;
    vehicle.dwell_remaining = 0.0;
    vehicle.mode = VehicleMode::ACTIVE;
    vehicle.action = VehicleAction::NOMINAL;
    vehicle.requested_action = VehicleAction::NOMINAL;
    vehicle.reason = "deadlock_replan";
    ROS_WARN("[deadlock-replan] V%d 从位姿(%.2f,%.2f,%.0fdeg)重规划 -> slot %d "
             "wpts=%zu len=%.2f arc=%d",
             vehicle.id, p.x, p.y, theta * 180.0 / kPi, target,
             vehicle.track.path().size(), vehicle.track.length(),
             info.used_arc_fallback ? 1 : 0);
    return true;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
