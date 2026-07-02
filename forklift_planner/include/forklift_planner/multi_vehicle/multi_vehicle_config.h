#pragma once

#include <ros/ros.h>

#include <string>
#include <vector>

namespace forklift_planner {
namespace multi_vehicle {

struct MultiVehicleConfig {
    int vehicle_count = 8;
    int random_seed = 42;

    double nominal_speed = 0.20;
    double max_speed = 0.26;
    double max_accel = 0.20;
    double max_decel = 0.30;
    double safety_margin = 0.0;
    double conflict_margin = 0.12;

    double dwell_time = 20.0;       // sleep time
    double rolling_horizon = 10.0;          // future planner time 
    double rolling_refresh_period = 2.0;        //  refresh period

    double prediction_horizon = 10.0;
    double prediction_step = 0.05;

    double creep_ratio = 0.25;
    double yield_ratio = 0.50;
    double boost_ratio = 1.20;
    bool enable_boost = true;

    double emergency_time = 0.80;
    double final_decision_time = 2.00;
    double warning_time = 5.00;

    double target_request_distance = 0.45;
    double target_stop_distance = 0.12;
    double following_normal_distance = 0.35;
    double following_creep_distance = 0.22;
    double following_min_distance = 0.13;
    double starvation_wait_time = 8.0;

    // Minimum time (s) an action must be held before it may relax toward a less
    // restrictive action. Stops per-tick STOP<->NOMINAL / YIELD<->NOMINAL
    // chatter (spec section 15) and forces a staged STOP->CREEP->NOMINAL restart
    // (spec section 14). Tightening (braking harder) is always applied at once.
    // Set <= 0 to disable smoothing (raw rule output each cycle).
    double action_hold_time = 0.4;

    // Optional coordination rules layered ON TOP of the hard collision guard.
    // The collision guard itself (no two vehicle bodies may overlap) is always
    // active and cannot be turned off -- it is the safety bottom line. These two
    // are policy, not safety. Set both false for pure collision avoidance: it is
    // provably collision-free but will deadlock on symmetric conflicts.
    bool enable_priority_tiebreak = true;  // who proceeds in a symmetric conflict
    // global_stall_release(盲目轻推卡住的车)与资源模型冲突:它每帧把 res_wait 的车
    // 往前蹭 → STOP↔CREEP 抖动"磨磨唧唧"(§15 第三类)。资源模型靠"优先级 winner 必能
    // 通过 + 出口检查 + cycle_break"提供活性,无需盲推。故默认关闭。
    bool enable_stall_release = false;     // (旧兜底,已被资源模型取代)
    // 旧启发式死锁兜底:与资源模型互掐(顺 res_wait 链误判成环、每帧给 YIELD 与
    // res_wait STOP 交替→蹭行)。资源活性改由 Phase4 正式等待图提供。落地前先关。
    bool enable_deadlock_reverse = false;
    bool enable_cycle_break = false;
    // 死锁恢复(检测持续环→从当前位姿重规划到别的库位脱困)。这种「开到一半
    // 发现锁死→重规划脱困」的反应式兜底直接扔掉,改从源头(规则+分配)根治。默认关闭;
    // 看门狗仍可纯检测(deadlock_ticks 计数),仅不执行重规划动作。
    bool enable_deadlock_recovery = false;

    // 实车模式:位置来自动捕 /object(替代 advanceVehicles 积分),输出 /traj_i + /coord_speed_i
    // 给 pure_pursuit。协调(updateDwellAndTasks/decide/到库DWELL/预测错峰)与 sim 逐字节一致。
    bool real_mode = false;
    bool one_shot_traj = false;          // true=一次性规划，false=滚动时域
    // 实车安全/到点阈值(仅 real_mode 用,现场可调,不影响 sim):
    double real_pose_timeout = 0.5;   // s,某车动捕失联>此值→强制其 coord_speed=0(防盲走)
    double real_arrive_tol = 0.05;    // m,path_s 距 length<此值即判到点(>PP的2cm硬停容差,防卡死)
    // 实车硬护栏(第二层兜底):用真实 /object 足迹两两检测,充气 margin 后重叠即双方急停。
    // sim 的 advanceVehicles 有这层,realAdvance 没有 → 预测层(decide)漏判时实车无人兜底。
    // margin 要 < 预测层维持的间距才不误触发(预测层按 conflict_margin=0.12 把车拉开,
    // 故 margin<0.12 时正常运行不会触,只在预测层快失守、两车逼到 <margin 时才急停);
    // 默认 0.08,可现场调;设 0 = 关闭实车硬护栏。
    double real_emergency_margin = 0.08;  // m,0=关闭实车硬护栏
    // 曲率限速(规划侧,sim 与实车共用):弯道速度 v≤√(a_lat/κ),令规划速度运动学完备
    // (纵向已有加减速+精确刹车距离,这里补"速度随曲率降"的横向运动学约束)。sim/实车一致才能
    // 用 sim 验证它对协调的影响。0=关。
    double lat_accel_max = 0.10;          // m/s² 侧向加速度上限:越小弯道越慢、跟得越紧

    bool show_paths = true;
    bool show_prediction_conflicts = true;
    bool skip_arc_fallback_paths = true;
    bool reject_curvature_discontinuity = true;
    bool reject_boundary_violations = true;
    bool reject_shelf_collisions = true;
    bool reject_path_kinks = true;        // reject paths with a kinematic kink
    double kink_min_angle = 0.61;         // rad (~35deg): below = smooth travel
    double kink_cusp_angle = 2.53;        // rad (~145deg): above = clean reverse cusp
    double path_validation_step = 0.02;
    bool precompute_task_filter = true;
    bool log_invalid_task_pairs = false;
    bool quiet_task_filter_precompute = true;

    // 简单测试版(eight-veh-sim):每车选「路径最短且全程前进(无 REVERSE 段=无尖点)」的目标,
    // 一把开进去、走得近。覆盖默认的跨排/分散选靶逻辑。仅用于实车链路冒烟测试。
    bool simple_forward_demo = false;

    int recent_target_memory = 5;
    int recent_row_memory = 4;

    std::vector<int> start_slots;
    // 简单测试版:每车预设终点库位(与 start_slots 同序,车 i → target_slots[i])。空=自动选最近前进目标。
    std::vector<int> target_slots;
    bool randomize_start = false;  // true: draw distinct start slots from random_seed

    static MultiVehicleConfig fromROSParam(ros::NodeHandle& nh);
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
