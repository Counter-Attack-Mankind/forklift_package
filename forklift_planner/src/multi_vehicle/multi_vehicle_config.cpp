#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"

#include <XmlRpcValue.h>

#include <algorithm>

namespace forklift_planner {
namespace multi_vehicle {

namespace {

std::vector<int> readIntVector(ros::NodeHandle& nh, const std::string& name,
                               const std::vector<int>& fallback) {
    XmlRpc::XmlRpcValue value;
    if (!nh.getParam(name, value) || value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        return fallback;
    }

    std::vector<int> out;
    out.reserve(static_cast<size_t>(value.size()));
    for (int i = 0; i < value.size(); ++i) {
        if (value[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            out.push_back(static_cast<int>(value[i]));
        }
    }
    return out.empty() ? fallback : out;
}

}  // namespace

MultiVehicleConfig MultiVehicleConfig::fromROSParam(ros::NodeHandle& nh) {
    MultiVehicleConfig c;
    const std::string ns = "forklift_planner/multi_vehicle/";

    nh.param(ns + "vehicle_count", c.vehicle_count, c.vehicle_count);
    nh.param(ns + "random_seed", c.random_seed, c.random_seed);
    nh.param(ns + "nominal_speed", c.nominal_speed, c.nominal_speed);
    nh.param(ns + "max_speed", c.max_speed, c.max_speed);
    nh.param(ns + "max_accel", c.max_accel, c.max_accel);
    nh.param(ns + "max_decel", c.max_decel, c.max_decel);
    nh.param(ns + "safety_margin", c.safety_margin, c.safety_margin);
    nh.param(ns + "conflict_margin", c.conflict_margin, c.conflict_margin);
    nh.param(ns + "dwell_time", c.dwell_time, c.dwell_time);
    nh.param(ns + "rolling_horizon", c.rolling_horizon, c.rolling_horizon);
    nh.param(ns + "rolling_refresh_period", c.rolling_refresh_period,
             c.rolling_refresh_period);
    nh.param(ns + "real_mode", c.real_mode, c.real_mode);
    nh.param(ns + "one_shot_traj", c.one_shot_traj, c.one_shot_traj);
    nh.param(ns + "real_pose_timeout", c.real_pose_timeout, c.real_pose_timeout);
    nh.param(ns + "real_arrive_tol", c.real_arrive_tol, c.real_arrive_tol);
    nh.param(ns + "real_emergency_margin", c.real_emergency_margin, c.real_emergency_margin);
    nh.param(ns + "lat_accel_max", c.lat_accel_max, c.lat_accel_max);
    nh.param(ns + "prediction_horizon", c.prediction_horizon, c.prediction_horizon);
    nh.param(ns + "prediction_step", c.prediction_step, c.prediction_step);
    nh.param(ns + "creep_ratio", c.creep_ratio, c.creep_ratio);
    nh.param(ns + "yield_ratio", c.yield_ratio, c.yield_ratio);
    nh.param(ns + "boost_ratio", c.boost_ratio, c.boost_ratio);
    nh.param(ns + "enable_boost", c.enable_boost, c.enable_boost);
    nh.param(ns + "emergency_time", c.emergency_time, c.emergency_time);
    nh.param(ns + "final_decision_time", c.final_decision_time, c.final_decision_time);
    nh.param(ns + "warning_time", c.warning_time, c.warning_time);
    nh.param(ns + "target_request_distance", c.target_request_distance,
             c.target_request_distance);
    nh.param(ns + "target_stop_distance", c.target_stop_distance,
             c.target_stop_distance);
    nh.param(ns + "following_normal_distance", c.following_normal_distance,
             c.following_normal_distance);
    nh.param(ns + "following_creep_distance", c.following_creep_distance,
             c.following_creep_distance);
    nh.param(ns + "following_min_distance", c.following_min_distance,
             c.following_min_distance);
    nh.param(ns + "starvation_wait_time", c.starvation_wait_time,
             c.starvation_wait_time);
    nh.param(ns + "action_hold_time", c.action_hold_time, c.action_hold_time);
    nh.param(ns + "enable_priority_tiebreak", c.enable_priority_tiebreak,
             c.enable_priority_tiebreak);
    nh.param(ns + "enable_stall_release", c.enable_stall_release,
             c.enable_stall_release);
    nh.param(ns + "enable_deadlock_reverse", c.enable_deadlock_reverse,
             c.enable_deadlock_reverse);
    nh.param(ns + "enable_cycle_break", c.enable_cycle_break,
             c.enable_cycle_break);
    nh.param(ns + "enable_deadlock_recovery", c.enable_deadlock_recovery,
             c.enable_deadlock_recovery);
    nh.param(ns + "simple_forward_demo", c.simple_forward_demo, c.simple_forward_demo);
    nh.param(ns + "show_paths", c.show_paths, c.show_paths);
    nh.param(ns + "show_prediction_conflicts", c.show_prediction_conflicts,
             c.show_prediction_conflicts);
    nh.param(ns + "skip_arc_fallback_paths", c.skip_arc_fallback_paths,
             c.skip_arc_fallback_paths);
    nh.param(ns + "reject_curvature_discontinuity",
             c.reject_curvature_discontinuity,
             c.reject_curvature_discontinuity);
    nh.param(ns + "reject_boundary_violations", c.reject_boundary_violations,
             c.reject_boundary_violations);
    nh.param(ns + "reject_shelf_collisions", c.reject_shelf_collisions,
             c.reject_shelf_collisions);
    nh.param(ns + "reject_path_kinks", c.reject_path_kinks, c.reject_path_kinks);
    nh.param(ns + "kink_min_angle", c.kink_min_angle, c.kink_min_angle);
    nh.param(ns + "kink_cusp_angle", c.kink_cusp_angle, c.kink_cusp_angle);
    nh.param(ns + "path_validation_step", c.path_validation_step,
             c.path_validation_step);
    nh.param(ns + "precompute_task_filter", c.precompute_task_filter,
             c.precompute_task_filter);
    nh.param(ns + "log_invalid_task_pairs", c.log_invalid_task_pairs,
             c.log_invalid_task_pairs);
    nh.param(ns + "quiet_task_filter_precompute",
             c.quiet_task_filter_precompute,
             c.quiet_task_filter_precompute);
    nh.param(ns + "recent_target_memory", c.recent_target_memory,
             c.recent_target_memory);
    nh.param(ns + "recent_row_memory", c.recent_row_memory, c.recent_row_memory);

    c.start_slots = readIntVector(nh, ns + "start_slots",
                                  {0, 8, 17, 22, 33, 42, 49, 58});
    c.target_slots = readIntVector(nh, ns + "target_slots", {});
    nh.param(ns + "randomize_start", c.randomize_start, c.randomize_start);

    c.vehicle_count = std::max(1, std::min(c.vehicle_count, 8));
    c.prediction_step = std::max(0.02, c.prediction_step);
    c.prediction_horizon = std::max(c.prediction_step, c.prediction_horizon);
    c.nominal_speed = std::max(0.01, c.nominal_speed);
    c.max_speed = std::max(c.nominal_speed, c.max_speed);
    c.conflict_margin = std::max(0.0, c.conflict_margin);
    c.creep_ratio = std::max(0.0, std::min(c.creep_ratio, 1.0));
    c.yield_ratio = std::max(c.creep_ratio, std::min(c.yield_ratio, 1.0));
    c.boost_ratio = std::max(1.0, c.boost_ratio);
    c.dwell_time = std::max(0.0, c.dwell_time);
    c.rolling_horizon = std::max(0.1, c.rolling_horizon);
    c.rolling_refresh_period = std::max(0.1, c.rolling_refresh_period);
    c.path_validation_step = std::max(0.005, c.path_validation_step);
    return c;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
