#include <ros/ros.h>

#include <algorithm>
#include <array>
#include <clocale>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <deque>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "forklift_map/forklift_map.h"
#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/footprint.h"
#include "forklift_planner/multi_vehicle/marker_publisher.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/rule_engine.h"
#include "forklift_planner/multi_vehicle/task_allocator.h"
#include "forklift_planner/multi_vehicle/traffic_resource_map.h"
#include "forklift_planner/path_generator.h"
#include "forklift_planner/planner_param.h"
#include "geometry_msgs/Point.h"
#include "sandbox_msgs/AprilObject.h"
#include "sandbox_msgs/Trajectory.h"
#include "sandbox_msgs/TrajectoryPoint.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

namespace {

std_msgs::ColorRGBA rgba(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

}  // namespace

class MultiVehiclePatrolNode {
public:
    MultiVehiclePatrolNode() : nh_("~") {
        ros::NodeHandle param_nh;
        mp_ = MapParam::fromROSParam(param_nh);
        pp_ = PlannerParam::fromROSParam(param_nh);
        cfg_ = forklift_planner::multi_vehicle::MultiVehicleConfig::fromROSParam(
            param_nh);
        nh_.param("target_only", target_only_, -1);
        nh_.param("one_shot", one_shot_, one_shot_);
        rb_horizon_ = cfg_.rolling_horizon;
        rb_horizon_refresh_period_ = cfg_.rolling_refresh_period;
        rb_horizon_refresh_ = std::max(
            1, static_cast<int>(std::lround(rb_horizon_refresh_period_ * pp_.update_rate)));
        rb_one_shot_traj_ = cfg_.one_shot_traj;
        if (target_only_ >= 0) {
            target_only_ = std::max(0, std::min(7, target_only_));
            cfg_.vehicle_count = std::max(cfg_.vehicle_count, target_only_ + 1);
            ROS_WARN("[real] single-target mode: only V%d will receive a task/controller; "
                     "its path still uses start_slots[%d] -> target_slots[%d].",
                     target_only_, target_only_, target_only_);
        }

        map_ = std::make_unique<ForkliftMap>(mp_);
        resource_map_ = std::make_unique<
            forklift_planner::multi_vehicle::TrafficResourceMap>(
            mp_, map_->slots(), map_->road_segments());
        generator_ = std::make_unique<PathGenerator>(mp_, pp_);
        allocator_ = std::make_unique<forklift_planner::multi_vehicle::TaskAllocator>(
            mp_, pp_, cfg_, *map_, *generator_);
        rule_engine_ = std::make_unique<forklift_planner::multi_vehicle::RuleEngine>(
            mp_, cfg_);
        rule_engine_->setResourceMap(resource_map_.get());
        marker_pub_ = std::make_unique<forklift_planner::multi_vehicle::MarkerPublisher>(
            nh_, mp_, pp_, cfg_);
        // A方案:仿真也画每车完整轨迹。real 模式 setupRealIO 会再 advertise(同topic,无害)。
        horizon_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_planner/markers", 10);

        if (cfg_.precompute_task_filter) {
            allocator_->buildCache();
        }
        initAgents();
        one_shot_done_.assign(agents_.size(), false);
        dumpResourceSpans();  // Phase 1.3 验证:打印各车路径经过的资源占用区间

        if (cfg_.real_mode) {
            setupRealIO();  // 实车模式:建 /object 订阅 + /traj_i//coord_speed_i 发布,打印摆位
            timer_ = nh_.createTimer(ros::Duration(1.0 / pp_.update_rate),
                                     &MultiVehiclePatrolNode::tick, this);
            ROS_WARN("[multi_patrol] *** 实车模式 real_mode=true ***:位置取 /object,"
                     "发 /traj_i + /coord_speed_i;协调与 sim 一致。");
            return;  // 实车模式不走 batch
        }

        // 1. 无头批处理快速回归:~batch_ticks>0 时不建实时 timer,由 main 调 runBatch 狂跑。
        // 也支持 ~batch_minutes(按仿真分钟换算成拍数,更直观)。
        ros::NodeHandle pnh("~");
        int batch_ticks = 0;
        double batch_minutes = 0.0;
        pnh.param("batch_ticks", batch_ticks, 0);
        pnh.param("batch_minutes", batch_minutes, 0.0);
        if (batch_ticks <= 0 && batch_minutes > 0.0)
            batch_ticks = static_cast<int>(batch_minutes * 60.0 * pp_.update_rate);
        cfg_batch_ticks_ = batch_ticks > 0 ? static_cast<unsigned long long>(batch_ticks) : 0;
        if (cfg_batch_ticks_ > 0) {
            ROS_WARN("[batch] 无头快速回归模式:将狂跑 %llu 拍(≈%.0f 仿真分钟),"
                     "不发 marker、不按实时。", cfg_batch_ticks_,
                     cfg_batch_ticks_ / (pp_.update_rate * 60.0));
            return;  // 不建 timer
        }

        // 2. ROS实时模式，创建timer，按照步长调用tick()函数

        timer_ = nh_.createTimer(ros::Duration(1.0 / pp_.update_rate),
                                 &MultiVehiclePatrolNode::tick, this);

        ROS_INFO("[multi_patrol] started RViz timestamp simulation: vehicles=%d "
                 "seed=%d speed=%.2f max=%.2f dwell=%.2f horizon=%.2f step=%.2f",
                 cfg_.vehicle_count, cfg_.random_seed, cfg_.nominal_speed,
                 cfg_.max_speed, cfg_.dwell_time, cfg_.prediction_horizon,
                 cfg_.prediction_step);
    }

private:
    using VehicleAgent = forklift_planner::multi_vehicle::VehicleAgent;
    using VehicleAction = forklift_planner::multi_vehicle::VehicleAction;
    using VehicleMode = forklift_planner::multi_vehicle::VehicleMode;

    bool targetEnabled(int id) const {
        return target_only_ < 0 || id == target_only_;
    }

    const char* modeName(VehicleMode mode) const {
        switch (mode) {
            case VehicleMode::NEED_TASK: return "NEED_TASK";
            case VehicleMode::ACTIVE: return "ACTIVE";
            case VehicleMode::DWELL: return "DWELL";
        }
        return "UNKNOWN";
    }

    void initAgents() {
        agents_.clear();
        agents_.reserve(static_cast<size_t>(cfg_.vehicle_count));

        const std::array<std_msgs::ColorRGBA, 8> colors = {
            rgba(0.00f, 0.28f, 0.82f),
            rgba(0.78f, 0.04f, 0.04f),
            rgba(0.00f, 0.52f, 0.16f),
            rgba(0.78f, 0.48f, 0.00f),
            rgba(0.46f, 0.16f, 0.78f),
            rgba(0.00f, 0.52f, 0.58f),
            rgba(0.82f, 0.24f, 0.00f),
            rgba(0.76f, 0.00f, 0.46f),
        };

        std::mt19937 rng(static_cast<unsigned int>(cfg_.random_seed));
        std::bernoulli_distribution load_dist(0.5);
        const int slot_count = static_cast<int>(map_->slots().size());

        // 简单测试版:一个库位「能当起点」= 既能出库(hasValidOutbound),又至少有一个全程前进
        // (无尖点)的可达目标(hasForwardTarget)。普通模式只要求能出库。
        auto startOK = [&](int s) {
            if (!allocator_->hasValidOutbound(s)) return false;
            if (cfg_.simple_forward_demo && !allocator_->hasForwardTarget(s)) return false;
            return true;
        };

        // 简单测试版一次性诊断:贪心算 want 对互不冲突(起点≠终点、起点两两不同、终点两两不同)的
        // 无尖点前进对,分别按「最短(走得近)」和「最长(走得远)」各算一套,打印供离线预设到 start/target_slots。
        if (cfg_.simple_forward_demo) {
            const int want = std::max(1, cfg_.vehicle_count);
            auto greedyMatch = [&](bool prefer_long) {
                std::vector<int> cs, ct; std::vector<double> cl;
                std::vector<bool> us(slot_count, false), ut(slot_count, false);
                for (int iter = 0; iter < want; ++iter) {
                    int bs = -1, bt = -1; double blen = prefer_long ? -1.0 : 1e18;
                    for (int s = 0; s < slot_count; ++s) {
                        if (us[s] || ut[s] || !startOK(s)) continue;
                        std::vector<int> ft; std::vector<double> fl;
                        allocator_->forwardTargets(s, ft, &fl);  // 实际路径弧长
                        for (size_t k = 0; k < ft.size(); ++k) {
                            const int t = ft[k];
                            if (us[t] || ut[t] || t == s) continue;
                            const bool better = prefer_long ? (fl[k] > blen) : (fl[k] < blen);
                            if (better) { blen = fl[k]; bs = s; bt = t; }
                        }
                    }
                    if (bs < 0) break;
                    us[bs] = ut[bt] = true; cs.push_back(bs); ct.push_back(bt); cl.push_back(blen);
                }
                std::string ss, ts; double tot = 0.0;
                for (size_t k = 0; k < cs.size(); ++k) {
                    ss += std::to_string(cs[k]) + (k+1<cs.size()?", ":"");
                    ts += std::to_string(ct[k]) + (k+1<ct.size()?", ":"");
                    tot += cl[k];
                }
                ROS_WARN("[simple-scan] %s %zu 对(总长%.2fm):", prefer_long?"【走得远】":"【走得近】", cs.size(), tot);
                for (size_t k = 0; k < cs.size(); ++k)
                    ROS_WARN("[simple-scan]   对%zu: %d -> %d  len=%.3f", k+1, cs[k], ct[k], cl[k]);
                ROS_WARN("[simple-scan]   start_slots:  [%s]", ss.c_str());
                ROS_WARN("[simple-scan]   target_slots: [%s]", ts.c_str());
            };
            greedyMatch(false);  // 短
            greedyMatch(true);   // 长
        }

        std::vector<int> random_starts;
        if (cfg_.randomize_start) {
            for (int s = 0; s < slot_count; ++s) {
                if (startOK(s)) random_starts.push_back(s);
            }
            std::shuffle(random_starts.begin(), random_starts.end(), rng);
        }

        std::vector<bool> used(static_cast<size_t>(slot_count), false);
        for (int i = 0; i < cfg_.vehicle_count; ++i) {
            VehicleAgent v;
            v.id = i;
            int start_slot;
            if (cfg_.randomize_start && !random_starts.empty()) {
                start_slot = random_starts[static_cast<size_t>(i) %
                                           random_starts.size()];
            } else {
                start_slot = cfg_.start_slots.empty()
                    ? i
                    : cfg_.start_slots[static_cast<size_t>(i) %
                                       cfg_.start_slots.size()];
            }
            start_slot = ((start_slot % slot_count) + slot_count) % slot_count;

            // 简单测试版下 startOK 还要求该起点有「全程前进(无尖点)」目标,保证每车都能一把开进。
            if (used[start_slot] || !startOK(start_slot)) {
                int repl = -1;
                for (int s = 0; s < slot_count; ++s) {
                    if (!used[s] && startOK(s)) {
                        repl = s;
                        break;
                    }
                }
                if (repl >= 0) {
                    if (!startOK(start_slot)) {
                        ROS_WARN("[multi_patrol] start slot %d 不可用"
                                 "(陷阱/无前进目标); V%d 改从 slot %d 起步",
                                 start_slot, i, repl);
                    }
                    start_slot = repl;
                }
            }
            used[start_slot] = true;

            v.current_slot = start_slot;
            v.target_slot = v.current_slot;
            v.loaded = load_dist(rng);
            v.color = colors[static_cast<size_t>(i) % colors.size()];
            v.mode = VehicleMode::NEED_TASK;
            agents_.push_back(v);
        }

        for (VehicleAgent& v : agents_) {
            if (targetEnabled(v.id)) {
                allocator_->assignNextTask(v, agents_);
            }
        }
        resetStatusLogState();
    }

    // Phase 1.3 验证:对每辆 active 车,打印其固定路径经过的资源及弧长区间,
    // 用于核对资源地图(SLOT_BODY/DOCK 等)与实际路径一致。一次性。
    void dumpResourceSpans() {
        ROS_INFO("[res_map] resources built: %zu",
                 resource_map_->resources().size());
        for (const VehicleAgent& v : agents_) {
            if (v.track.empty()) continue;
            const auto spans = resource_map_->spansForPath(v.track);
            for (const auto& sp : spans) {
                const auto* r = resource_map_->byId(sp.resource_id);
                ROS_INFO("[res_map] V%d uses %s s=[%.3f,%.3f] (len=%.3f)",
                         v.id, r ? r->name.c_str() : "?",
                         sp.s_enter, sp.s_exit, v.track.length());
            }
        }
    }

    void resetStatusLogState() {
        const size_t n = agents_.size();
        last_logged_mode_.assign(n, VehicleMode::NEED_TASK);
        last_logged_action_.assign(n, VehicleAction::STOP);
        last_logged_reason_.assign(n, "");
        last_logged_blocker_.assign(n, -999);
        last_logged_task_count_.assign(n, -1);
        last_status_log_time_.assign(n, ros::Time(0));
        last_diag_time_.assign(n, ros::Time(0));
    }

    double limitedSpeed(double current, double desired, double dt) const {
        if (desired > current) {
            return std::min(desired, current + cfg_.max_accel * dt);
        }
        return std::max(desired, current - cfg_.max_decel * dt);
    }

    RoughWp poseForCollision(const VehicleAgent& v, double path_s) const {
        if (!v.track.empty()) {
            if (v.mode == VehicleMode::DWELL) {
                return v.track.poseAtS(v.track.length());
            }
            return v.track.poseAtS(std::min(path_s, v.track.length()));
        }
        // 无轨迹的 idle 车(NEED_TASK)仍物理停在 current_slot 上，别的车不能从它身上
        // 碾过去。车身中心在 dock，朝向朝库外(pre_dock 方向)，参考点沿鼻向后移 d。
        const Slot& s = map_->slots().at(static_cast<size_t>(v.current_slot));
        const double th = std::atan2(s.pre_dock_y - s.dock_y(),
                                     s.pre_dock_x - s.dock_x());
        RoughWp p;
        p.x = s.dock_x() - mp_.rear_axle_to_center * std::cos(th);
        p.y = s.dock_y() - mp_.rear_axle_to_center * std::sin(th);
        p.theta = th;
        p.type = WpType::FORWARD;
        return p;
    }

    // 前瞻预测(集中式全信息+确定性):克隆全局状态,复用真实 updateDwellAndTasks+decide+
    // advanceVehicles 闭环空跑 H 拍(预测检查被 sim_mode_ 屏蔽,不递归),若 H 内出现「持续闭环
    // 死锁」(findDeadlockMembers 非空,wait≥kWait 的全停闭环)即返回 true。跑完精确还原,对真实零影响。
    bool simPredictsDeadlock() {
        const double dt = 1.0 / pp_.update_rate;
        constexpr int H = 400;          // 前瞻 ~40s
        constexpr double kWait = 10.0;  // 全停闭环持续此秒数 = 真死锁
        const std::vector<VehicleAgent> sa = agents_;
        const auto sr = rule_engine_->snapshot();
        const auto sl = allocator_->snapshot();
        const bool prev = sim_mode_;
        sim_mode_ = true;
        bool dead = false;
        for (int s = 0; s < H && !dead; ++s) {
            updateDwellAndTasks(dt);
            rule_engine_->decide(agents_, dt);
            advanceVehicles(dt);
            if (!findDeadlockMembers(kWait).empty()) dead = true;
        }
        agents_ = sa;
        rule_engine_->restore(sr);
        allocator_->restore(sl);
        sim_mode_ = prev;
        return dead;
    }

    // ───────── AD 滚动时域:世界模型推演 → 有限时域时间参数化轨迹 ─────────
    // 零自由度=全局确定性:从当前真实状态(/object 位置 + 动捕差分速度,已在 agents_ 里)出发,
    // 复用真实 updateDwellAndTasks+decide+advanceVehicles 精确前推 horizon 秒,逐拍记录每车
    // (x,y,yaw,带符号速度,time)。这条轨迹运动学完备(模型自带曲率限速/加减速/停止/倒车换向过零),
    // 位置序列隐含方向→控制器纯跟踪即可,不需 coord_speed 符号/typeAtS 那套。每周期刷新=滚动时域。
    // hold[i]=true 表示该车整段不动(静止特例)→ 控制器 idle 不控制。

    void rollWorldModel(double horizon, std::vector<sandbox_msgs::Trajectory>& out,
                        std::vector<bool>& hold) {

        const double dt = 1.0 / pp_.update_rate;            //系统每触发一次，就向前推进dt时间
        const int H = std::max(1, (int)std::lround(horizon / dt));      //四舍五入决定仿真步数，但至少模拟1步

        //=====（初始化轨迹，全部置空并且默认车辆均为静止状态）===========
        const size_t n = agents_.size();
        out.assign(n, sandbox_msgs::Trajectory{});
        hold.assign(n, true);
        //初始化轨迹，所有离散点中的目标点按序排列，并且坐标系设为世界坐标系
        for (size_t i = 0; i < n; ++i) { out[i].target = (int)i; out[i].header.frame_id = "world"; }

        //===========（状态快照与回滚）============
        const std::vector<VehicleAgent> sa = agents_;    //（将Agents通过拷贝构造函数给sa，sa设为const，后续仅改变备份的agents，对显示不产生影响）  
        const auto sr = rule_engine_->snapshot();       //保存规则引擎状态
        const auto sl = allocator_->snapshot();         //保存任务分配器状态
        const bool prev = sim_mode_;                    //保存现在模式（仿真或是实际）
        sim_mode_ = true;       //切换到仿真模式，因为现在属于提前规划，必须视为仿真

        //=========（创建匿名函数recored，用来记录当前车辆状态，并每拍给每辆车生成一个TrajPoint）========
        auto record = [&](int s) {
            for (size_t i = 0; i < n; ++i) {
                const VehicleAgent& v = agents_[i];         //对每辆车都记录一次当前状态，v是agents_[i] 的只读引用
                const RoughWp p = poseForCollision(v, v.path_s);    //根据车辆v当前走到路径上的距离 path_s，求它当前在地图里的姿态
                // 几何判前进/倒车(用户判据):存的航向(=车头)与路径切向(=前进方向)反向→倒车。
                // 不靠 typeAtS 标签(可能没标对)。前进段速度取正、倒车段取负——就这么简单。
                
                //====判断是否倒车=====
                bool rev = false;
                if (!v.track.empty()) {
                    const double L = v.track.length(), ds = 0.05;
                    const double s = std::min(std::max(v.path_s, 0.0), L);
                    const auto pa = v.track.poseAtS(std::max(0.0, s - ds));
                    const auto pc = v.track.poseAtS(std::min(L, s + ds));
                    rev = std::cos(p.theta - std::atan2(pc.y - pa.y, pc.x - pa.x)) < 0.0;
                }
                sandbox_msgs::TrajectoryPoint tp;
                tp.x = p.x; tp.y = p.y; tp.yaw = p.theta;                          // 车头朝向
                tp.velocity = (rev ? -1.0 : 1.0) * std::max(0.0, v.current_speed); // 前进+/倒车-
                tp.time = s * dt;
                out[i].points.push_back(tp);
                if (v.current_speed > 1e-3) hold[i] = false;   // 整段都不动才算 hold
            }
        };

        //=====（将刚才判断记录的，作为预测的第0个点）====
        record(0);

        //开始向未来预测H步长，H = 预测时间/dt
        for (int s = 1; s <= H; ++s) {
            updateDwellAndTasks(dt);
            rule_engine_->decide(agents_, dt);
            advanceVehicles(dt);
            record(s);
        }

        //将沙盒预测造成所有的改动恢复
        agents_ = sa;
        rule_engine_->restore(sr);
        allocator_->restore(sl);
        sim_mode_ = prev;
    }

    // 滚动时域发布:推演 rb_horizon_ 秒,把每车时间参数化轨迹发到 /traj_i(刷新=滚动)。
    // hold 车(整段不动)发单点轨迹(size=1)作静止标志 → 控制器 idle 不控制。
    // 滚动时域发布：重新推演并覆盖每辆车的短时轨迹。
    void publishHorizon() {
        std::vector<sandbox_msgs::Trajectory> trajs;
        std::vector<bool> hold;
        
        // =======世界模型推演，传入（预测时长，预测得到每辆车未来轨迹，预测得到每辆车未来是否保持静止）=============
        rollWorldModel(rb_horizon_, trajs, hold);


        const ros::Time now = ros::Time::now();
        const double tnow = now.toSec();
        std::vector<bool> guard(trajs.size(), false);
        if (cfg_.real_mode) {
            guard = realHardGuard();
        }

          //==================（对每辆车先在Rviz中删除旧轨迹）==========================
        visualization_msgs::MarkerArray arr;
        for (size_t i = 0; i < trajs.size(); ++i) {
            visualization_msgs::Marker del;
            del.header.frame_id = pp_.frame_id;
            del.header.stamp = now;
            del.ns = "horizon_traj";
            del.id = static_cast<int>(i);
            del.action = visualization_msgs::Marker::DELETE;
            arr.markers.push_back(del);

            const VehicleAgent& v = agents_[i];

            // ========（若车辆正在休眠，则发布单点静止轨迹，不刷新轨迹）=============
            if (v.mode == VehicleMode::DWELL || v.dwell_remaining > 1e-6) {
                if (i < traj_pubs_.size()) {
                    sandbox_msgs::Trajectory hold;
                    hold.target = static_cast<int>(i);
                    hold.header.frame_id = "world";
                    hold.header.stamp = now;

                    const RoughWp p = poseForCollision(v, v.path_s);
                    sandbox_msgs::TrajectoryPoint tp;
                    tp.x = p.x;
                    tp.y = p.y;
                    tp.yaw = p.theta;
                    tp.velocity = 0.0;
                    tp.time = 0.0;
                    hold.points.push_back(tp);

                    traj_pubs_[i].publish(hold);
            }

        continue;
}
            // ============（在实车条件下，若车辆位置摆放不正确，则不发布指令）========================
            if (cfg_.real_mode && !real_pose_ok_[i]) continue;


            // ============（若车辆急停、超时、保护，则轨迹退化为单点）========================
            const bool stale = cfg_.real_mode && cfg_.real_pose_timeout > 0.0 &&
                               (tnow - rb_last_seen_[i]) > cfg_.real_pose_timeout;
            if (rb_estop_ || stale || guard[i]) hold[i] = true;

            trajs[i].header.stamp = now;
            if (hold[i] && !trajs[i].points.empty()) {
                trajs[i].points.resize(1);
            }

            //=================（画新的Rviz轨迹，并发布轨迹给控制器）=======================
            visualization_msgs::Marker m;
            m.header.frame_id = pp_.frame_id;
            m.header.stamp = now;
            m.ns = "horizon_traj";
            m.id = static_cast<int>(i);
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.02;
            m.color = agents_[i].color;
            m.color.a = 0.9;
            for (const auto& p : trajs[i].points) {
                geometry_msgs::Point gp;
                gp.x = p.x;
                gp.y = p.y;
                gp.z = 0.09;
                m.points.push_back(gp);
            }
            arr.markers.push_back(m);

            if (i < traj_pubs_.size()) {
                traj_pubs_[i].publish(trajs[i]);
            }
        }
        if (!arr.markers.empty()) horizon_marker_pub_.publish(arr);
    }

    bool publishFullTrajectories() {
        std::vector<sandbox_msgs::Trajectory> trajs;
        std::vector<bool> hold;
        rollWorldModel(rb_full_horizon_, trajs, hold);   // 按全程上限推演一次(尾部静止点稍后裁掉)
        const ros::Time now = ros::Time::now();
        visualization_msgs::MarkerArray arr;
        bool all_done = true;
        for (size_t i = 0; i < trajs.size(); ++i) {
            if (!targetEnabled(static_cast<int>(i))) continue;
            if (agents_[i].track.empty()) continue;      // 无路径的车不计入(不卡总进度)
            if (one_shot_done_[i]) continue;             // 已发过的车不重发(latched 已在控制器手上)
            if (cfg_.real_mode && !real_pose_ok_[i]) {                     // 动捕未就位 → 这辆暂不发,标记未完成下拍补
                ROS_WARN_THROTTLE(1.0, "[real][one_shot] V%zu 动捕未就位 → 暂不发轨迹(就位后补发)", i);
                all_done = false;
                continue;
            }
            trimTrailingStationary(trajs[i]);            // 裁掉到点后的尾部静止点,只留一个停止点
            trajs[i].header.stamp = now;
            if (i < traj_pubs_.size()) traj_pubs_[i].publish(trajs[i]);             // latch 发一次,控制器自主跟到底
            one_shot_done_[i] = true;
            logFullTraj(i, trajs[i]);
            // RViz:整条轨迹画成该车颜色 LINE_STRIP(ns=horizon_traj,沿用现有显示)
            if (trajs[i].points.size() >= 2) {
                visualization_msgs::Marker m;
                m.header.frame_id = pp_.frame_id; m.header.stamp = now;
                m.ns = "horizon_traj"; m.id = (int)i;
                m.type = visualization_msgs::Marker::LINE_STRIP;
                m.action = visualization_msgs::Marker::ADD;
                m.pose.orientation.w = 1.0; m.scale.x = 0.02;
                m.color = agents_[i].color; m.color.a = 0.9;
                for (const auto& p : trajs[i].points) {
                    geometry_msgs::Point gp; gp.x = p.x; gp.y = p.y; gp.z = 0.09;
                    m.points.push_back(gp);
                }
                arr.markers.push_back(m);
            }
        }
        if (!arr.markers.empty()) horizon_marker_pub_.publish(arr);
        return all_done;
    }

    // 急停:给每车发单点 hold 轨迹(当前真实位姿,v=0)。外部 pure_pursuit 收到 size=1 的轨迹后
    // 锁在该点附近停车(不再前进)。一次性纯盲跟下这是唯一的软件急停手段(协调层已不干预纵向)。
    void publishHoldAll() {
        const ros::Time now = ros::Time::now();
        int sent = 0;
        for (size_t i = 0; i < agents_.size(); ++i) {
            if (!targetEnabled(static_cast<int>(i))) continue;
            if (cfg_.real_mode && !real_pose_ok_[i]) continue;          // 没真实位姿就别发垃圾点
            sandbox_msgs::Trajectory t;
            t.target = (int)i; t.header.frame_id = "world"; t.header.stamp = now;
            sandbox_msgs::TrajectoryPoint p;
            p.x = real_x_[i]; p.y = real_y_[i]; p.yaw = real_yaw_[i];
            p.velocity = 0.0; p.time = 0.0;
            t.points.push_back(p);
            traj_pubs_[i].publish(t);                 // latch:控制器停在原地
            ++sent;
        }
        ROS_ERROR("[real][one_shot] *** 急停:已对 %d 辆车发单点 hold 轨迹 → 控制器停车 ***", sent);
    }

    // 急停解除:从当前真实位置重新推演全程并重发。不能简单重发原轨迹——外部控制器按轨迹 time
    // 跟踪,重发会把 start_time_ 归零、车在中途却从 t=0 等起 → 卡死。重置 one_shot 标志,让主循环
    // 调 publishFullTrajectories 重新 rollWorldModel(从 realAdvance 同步好的当前 path_s 出发,
    // time=0 即对齐"现在")。
    void resumeFromEstop() {
        std::fill(one_shot_done_.begin(), one_shot_done_.end(), false);
        one_shot_published_ = false;
        ROS_WARN("[real][one_shot] *** 急停解除:从当前真实位置重新推演全程,准备重发 ***");
    }

    // 裁掉到达终点后的尾部静止点(velocity≈0):全程推演为留余量按大 horizon 跑,到点后会拖一长串
    // 原地不动的点,白白撑大轨迹消息。保留末尾一个 velocity=0 的停止点,控制器据此判 reached。
    void trimTrailingStationary(sandbox_msgs::Trajectory& t) {
        if (t.points.size() < 3) return;
        int k = (int)t.points.size() - 1;
        while (k > 0 && std::fabs(t.points[k].velocity) < 1e-3) --k;  // 最后一个仍在动的点
        const int stop = std::min(k + 1, (int)t.points.size() - 1);   // 紧随其后的停止点(在终点)
        sandbox_msgs::TrajectoryPoint sp = t.points[stop];
        sp.velocity = 0.0;
        t.points.resize(k + 1);
        t.points.push_back(sp);
    }

    // 一次性轨迹诊断日志:点数 / 时长 / 路径全长 / 有无倒车段 / 是否真跑到终点(否则 full_horizon 太短)。
    void logFullTraj(size_t i, const sandbox_msgs::Trajectory& t) {
        if (t.points.empty()) return;
        const double dur = t.points.back().time;
        bool has_rev = false;
        for (const auto& p : t.points) if (p.velocity < -1e-3) { has_rev = true; break; }
        const double len = agents_[i].track.empty() ? 0.0 : agents_[i].track.length();
        const bool reached = std::fabs(t.points.back().velocity) < 1e-3;
        ROS_WARN("[real][one_shot] V%zu 发整条轨迹 → /traj_%zu: 点数=%zu 时长=%.1fs 全长=%.2fm 倒车段=%s%s",
                 i, i, t.points.size(), dur, len, has_rev ? "有" : "无",
                 reached ? "" : "  ⚠ 末点仍在动:full_horizon 太短,加大 ~full_horizon");
    }

    //==============《任务指派与路径生成函数》===================================
    void updateDwellAndTasks(double dt) {

        
        for (VehicleAgent& v : agents_) {

            //*************** 1. 若该车未启用，则车状态一直置为STOP *******
            if (!targetEnabled(v.id)) {
                v.action = VehicleAction::STOP;
                v.requested_action = VehicleAction::STOP;
                v.current_speed = 0.0;
                continue;
            }

            // ********2. 若该车状态为需要任务，则尝试指派任务 ************
            // NEED_TASK 的车每拍重试派活——分配可能因"当下所有路都与在途车对穿"而暂时失败,
            // 但别车一移动局面就变,必须重试,否则车永久饿死(实测 6 车卡死的根因之一)。
            if (v.mode == VehicleMode::NEED_TASK) {
                allocator_->assignNextTask(v, agents_);         //***关键任务指派函数和路径生成 ******/
                continue;
            }


            //********** 3.若车不为休眠状态，则直接跳过 *********************
            if (v.mode != VehicleMode::DWELL) continue;

            //**********4.反之，若车在休眠状态，则停止动作，计算倒计睡眠时间，同时根据设置执行操作*/
            v.dwell_remaining = std::max(0.0, v.dwell_remaining - dt);
            v.action = VehicleAction::STOP;
            v.requested_action = VehicleAction::STOP;
            v.current_speed = 0.0;
                    
                //********* 4.1 若车已过睡眠时间，并且为一次性规划，则跳过不再派发任务 ******/
            if (v.dwell_remaining <= 1e-9) {
                if (one_shot_) {        // 一次性 demo:到达目标后永久停,不再派活(8车各跑一程 A→B)
                    v.action = VehicleAction::STOP;
                    v.requested_action = VehicleAction::STOP;
                    continue;
                }

                //******** 4.2 若车已过睡眠时间，并且为不间断跑，则切换速度，并派发任务 ******/
                v.loaded = !v.loaded;
                allocator_->assignNextTask(v, agents_);
                
                // 前瞻预测性避免("提前预料到就避免"):若这次发车会在 H 内导致持续死锁,就**错峰**
                // ——撤销发车、在车位再等一会(不堵路),让别车先过那段窄区,稍后再试。仅在真预测到
                // 死锁时才扣(精准,非广撒网);连扣上限防极端饥饿。整块仅真实模式执行(sim 内不递归)。
                if (!sim_mode_ && v.mode == VehicleMode::ACTIVE) {
                    if (predict_holds_[v.id] < 6 && simPredictsDeadlock()) {
                        v.loaded = !v.loaded;          // 撤销本次载货翻转(没真出发)
                        v.mode = VehicleMode::DWELL;
                        v.dwell_remaining = 2.0;
                        v.action = VehicleAction::STOP;
                        v.requested_action = VehicleAction::STOP;
                        v.reason = "predict_hold";
                        ++predict_holds_[v.id];
                    } else {
                        predict_holds_[v.id] = 0;      // 真发车了 → 清零连扣计数
                    }
                }
            }
        }
    }

    //==============================================

    void advanceVehicles(double dt) {
        std::vector<double> next_s(agents_.size(), 0.0);
        std::vector<double> next_speed(agents_.size(), 0.0);
        std::vector<double> planned_s(agents_.size(), 0.0);
        std::vector<bool> blocked(agents_.size(), false);

        for (size_t i = 0; i < agents_.size(); ++i) {
            VehicleAgent& v = agents_[i];
            next_s[i] = v.path_s;
            planned_s[i] = v.path_s;
            next_speed[i] = v.current_speed;
            if (!v.active()) continue;

            // 规划速度=动作档,再被曲率限速卡住(与实车 coord_speed 同一套,sim 才能真实验证)。
            const double desired_speed = std::min(rule_engine_->speedForAction(v.action),
                                                  curvatureSpeed(v));
            next_speed[i] = limitedSpeed(v.current_speed, desired_speed, dt);
            next_s[i] = std::min(v.track.length(), v.path_s + next_speed[i] * dt);
            planned_s[i] = next_s[i];
        }

        auto plannedS = [&](size_t idx) {
            return planned_s[idx];
        };

        auto bodyAt = [&](size_t idx, double path_s) {
            const RoughWp pose = poseForCollision(agents_[idx], path_s);
            return forklift_planner::multi_vehicle::makeBody(pose, mp_, 0.0);
        };

        auto overlapsAt = [&](size_t i, double s_i, size_t j, double s_j) {
            return forklift_planner::multi_vehicle::overlaps(bodyAt(i, s_i),
                                                             bodyAt(j, s_j));
        };

        auto canPlace = [&](size_t idx, double candidate_s) {
            for (size_t k = 0; k < agents_.size(); ++k) {
                if (k == idx) continue;
                if (agents_[k].mode != VehicleMode::ACTIVE &&
                    agents_[k].mode != VehicleMode::DWELL) {
                    continue;
                }
                if (overlapsAt(idx, candidate_s, k, plannedS(k))) return false;
            }
            return true;
        };

        auto canPlaceIgnoringPair = [&](size_t idx, double candidate_s,
                                        size_t pair_other) {
            for (size_t k = 0; k < agents_.size(); ++k) {
                if (k == idx || k == pair_other) continue;
                if (agents_[k].mode != VehicleMode::ACTIVE &&
                    agents_[k].mode != VehicleMode::DWELL) {
                    continue;
                }
                if (overlapsAt(idx, candidate_s, k, plannedS(k))) return false;
            }
            return true;
        };

        auto tryClearBlocker = [&](size_t idx, int blocker_id) {
            VehicleAgent& v = agents_[idx];
            if (!v.active()) return false;
            if (next_s[idx] > v.path_s + 1e-9) return false;

            const double creep_speed =
                limitedSpeed(v.current_speed,
                             rule_engine_->speedForAction(VehicleAction::CREEP),
                             dt);
            const double candidate_s =
                std::min(v.track.length(), v.path_s + creep_speed * dt);
            if (candidate_s <= v.path_s + 1e-9) return false;
            if (!canPlace(idx, candidate_s)) return false;

            blocked[idx] = false;
            next_speed[idx] = creep_speed;
            next_s[idx] = candidate_s;
            planned_s[idx] = candidate_s;
            v.action = VehicleAction::CREEP;
            v.requested_action = VehicleAction::CREEP;
            v.reason = "clear_blocker_V" + std::to_string(blocker_id);
            return true;
        };

        auto blockVehicle = [&](size_t idx) {
            if (!agents_[idx].active()) return false;
            const bool changed =
                !blocked[idx] || std::abs(planned_s[idx] - agents_[idx].path_s) > 1e-9 ||
                next_speed[idx] > 1e-9;
            blocked[idx] = true;
            planned_s[idx] = agents_[idx].path_s;
            next_s[idx] = agents_[idx].path_s;
            next_speed[idx] = 0.0;
            return changed;
        };

        auto resolvePlannedOverlaps = [&]() {
        const size_t max_guard_iterations =
            std::max<size_t>(4, agents_.size() * agents_.size() * 2);
        for (size_t iter = 0; iter < max_guard_iterations; ++iter) {
            bool changed = false;
            bool any_overlap = false;

            for (size_t i = 0; i < agents_.size(); ++i) {
                if (agents_[i].mode != VehicleMode::ACTIVE &&
                    agents_[i].mode != VehicleMode::DWELL) {
                    continue;
                }

                for (size_t j = i + 1; j < agents_.size(); ++j) {
                    if (agents_[j].mode != VehicleMode::ACTIVE &&
                        agents_[j].mode != VehicleMode::DWELL) {
                        continue;
                    }
                    if (!overlapsAt(i, plannedS(i), j, plannedS(j))) {
                        continue;
                    }

                    any_overlap = true;
                    const bool i_active = agents_[i].active();
                    const bool j_active = agents_[j].active();
                    if (i_active && !j_active) {
                        changed = blockVehicle(i) || changed;
                    } else if (!i_active && j_active) {
                        changed = blockVehicle(j) || changed;
                    } else if (i_active && j_active) {
                        const bool i_moves =
                            next_s[i] > agents_[i].path_s + 1e-9;
                        const bool j_moves =
                            next_s[j] > agents_[j].path_s + 1e-9;
                        const bool i_only_safe =
                            i_moves &&
                            !overlapsAt(i, next_s[i], j, agents_[j].path_s);
                        const bool j_only_safe =
                            j_moves &&
                            !overlapsAt(i, agents_[i].path_s, j, next_s[j]);
                        const bool both_next_safe =
                            i_moves && j_moves &&
                            !overlapsAt(i, next_s[i], j, next_s[j]) &&
                            canPlaceIgnoringPair(i, next_s[i], j) &&
                            canPlaceIgnoringPair(j, next_s[j], i);
                        const bool both_stop_safe =
                            !overlapsAt(i, agents_[i].path_s, j, agents_[j].path_s);

                        if (both_next_safe) {
                            const bool changed_i =
                                blocked[i] ||
                                std::abs(planned_s[i] - next_s[i]) > 1e-9;
                            const bool changed_j =
                                blocked[j] ||
                                std::abs(planned_s[j] - next_s[j]) > 1e-9;
                            blocked[i] = false;
                            blocked[j] = false;
                            planned_s[i] = next_s[i];
                            planned_s[j] = next_s[j];
                            changed = changed_i || changed_j || changed;
                        } else if (i_only_safe && !j_only_safe) {
                            changed = blockVehicle(j) || changed;
                            tryClearBlocker(i, agents_[j].id);
                        } else if (!i_only_safe && j_only_safe) {
                            changed = blockVehicle(i) || changed;
                            tryClearBlocker(j, agents_[i].id);
                        } else if (i_only_safe && j_only_safe) {
                            const int winner = rule_engine_->priorityWinner(
                                agents_[i], agents_[j]);
                            if (winner == agents_[i].id) {
                                changed = blockVehicle(j) || changed;
                                tryClearBlocker(i, agents_[j].id);
                            } else if (winner == agents_[j].id) {
                                changed = blockVehicle(i) || changed;
                                tryClearBlocker(j, agents_[i].id);
                            } else {
                                // Tie-break rule disabled: no winner, stop both.
                                changed = blockVehicle(i) || changed;
                                changed = blockVehicle(j) || changed;
                            }
                        } else if (both_stop_safe) {
                            changed = blockVehicle(i) || changed;
                            changed = blockVehicle(j) || changed;
                        } else {
                            const int winner = rule_engine_->priorityWinner(
                                agents_[i], agents_[j]);
                            if (winner == agents_[i].id) {
                                changed = blockVehicle(j) || changed;
                                if (!tryClearBlocker(i, agents_[j].id)) {
                                    changed = blockVehicle(i) || changed;
                                }
                            } else if (winner == agents_[j].id) {
                                changed = blockVehicle(i) || changed;
                                if (!tryClearBlocker(j, agents_[i].id)) {
                                    changed = blockVehicle(j) || changed;
                                }
                            } else {
                                // Tie-break rule disabled: no winner, stop both.
                                changed = blockVehicle(i) || changed;
                                changed = blockVehicle(j) || changed;
                            }
                        }
                    }
                    if (!sim_mode_) {  // 前瞻仿真中只要其物理挡停效果,不计数/不打日志
                        ++hard_guard_events_;
                        hard_guard_pairs_.insert(
                            {std::min(agents_[i].id, agents_[j].id),
                             std::max(agents_[i].id, agents_[j].id)});
                        if (first_guard_tick_ == 0) first_guard_tick_ = tick_count_;
                        ROS_ERROR_THROTTLE(
                            1.0,
                            "[multi_patrol] hard collision guard: V%d vs V%d; "
                            "minimal stop applied",
                            agents_[i].id, agents_[j].id);
                    }
                }
            }

            if (!any_overlap || !changed) {
                break;
            }
        }
        };

        resolvePlannedOverlaps();

        bool any_active_motion = false;
        bool any_blocked_active = false;
        for (size_t i = 0; i < agents_.size(); ++i) {
            if (!agents_[i].active()) continue;
            any_blocked_active = any_blocked_active || blocked[i];
            if (!blocked[i] && planned_s[i] > agents_[i].path_s + 1e-9) {
                any_active_motion = true;
            }
        }

        if (cfg_.enable_stall_release &&
            (!any_active_motion || any_blocked_active)) {
            // 真实开车原则(规格§15)：只往「确实空着」的地方挪——前方若有别人的车身
            // (含安全余量)就老实等，绝不往里蹭。故每次轻推都先用 conflict_margin 校验
            // 候选车身是否清空；不清空就不推。这样既能放走"前方其实空、只是过度谨慎"
            // 的车(恢复流动)，又不会把车顶进别人(避免 stall_release 蹭出 V1↔V3 那种
            // 残留重叠 → hard_collision_guard 反复触发)。真·僵死留给 deadlock_reverse。
            const double cm = cfg_.conflict_margin * 0.5;
            auto clearAheadWithMargin = [&](size_t idx, double cand_s) {
                const auto me = forklift_planner::multi_vehicle::makeBody(
                    poseForCollision(agents_[idx], cand_s), mp_, cm);
                for (size_t k = 0; k < agents_.size(); ++k) {
                    if (k == idx) continue;
                    if (agents_[k].mode != VehicleMode::ACTIVE &&
                        agents_[k].mode != VehicleMode::DWELL) {
                        continue;
                    }
                    const auto other = forklift_planner::multi_vehicle::makeBody(
                        poseForCollision(agents_[k], plannedS(k)), mp_, cm);
                    if (forklift_planner::multi_vehicle::overlaps(me, other)) {
                        return false;
                    }
                }
                return true;
            };
            for (size_t i = 0; i < agents_.size(); ++i) {
                VehicleAgent& v = agents_[i];
                if (!v.active()) continue;
                if (!blocked[i] && planned_s[i] > v.path_s + 1e-9) continue;
                const double creep_speed =
                    limitedSpeed(v.current_speed,
                                 rule_engine_->speedForAction(VehicleAction::CREEP),
                                 dt);
                const double candidate_s =
                    std::min(v.track.length(), v.path_s + creep_speed * dt);
                if (candidate_s <= v.path_s + 1e-9) continue;
                // 只往清空的空间挪；前方被占(含余量)则不推，老实等。
                if (!clearAheadWithMargin(i, candidate_s)) continue;
                blocked[i] = false;
                next_speed[i] = creep_speed;
                next_s[i] = candidate_s;
                planned_s[i] = candidate_s;
                v.action = VehicleAction::CREEP;
                v.requested_action = VehicleAction::CREEP;
                v.reason = "global_stall_release";
            }
            resolvePlannedOverlaps();
        }

        if (cfg_.enable_deadlock_reverse) {
            auto motionHeading = [](const VehicleAgent& v) {
                constexpr double kPi = 3.14159265358979323846;
                double h = v.track.poseAtS(v.path_s).theta;
                if (v.track.typeAtS(v.path_s) == WpType::REVERSE) h += kPi;
                return h;
            };
            auto forwardBlocker = [&](size_t idx) -> int {
                const VehicleAgent& v = agents_[idx];
                const double fwd_s = std::min(
                    v.track.length(),
                    v.path_s + rule_engine_->speedForAction(VehicleAction::CREEP) * dt);
                for (size_t k = 0; k < agents_.size(); ++k) {
                    if (k == idx) continue;
                    if (agents_[k].mode != VehicleMode::ACTIVE &&
                        agents_[k].mode != VehicleMode::DWELL) {
                        continue;
                    }
                    if (overlapsAt(idx, fwd_s, k, plannedS(k))) {
                        return static_cast<int>(k);
                    }
                }
                return -1;
            };

            constexpr double kReverseWait = 3.0;  // s stuck before backing out
            constexpr double kHeadOnDot = -0.5;   // headings nearly opposite
            bool any_reversed = false;
            for (size_t i = 0; i < agents_.size(); ++i) {
                VehicleAgent& v = agents_[i];
                if (!v.active() || v.wait_time < kReverseWait) continue;
                const int bk = forwardBlocker(i);
                if (bk < 0) continue;
                const VehicleAgent& b = agents_[bk];
                if (b.active() && planned_s[bk] > b.path_s + 1e-9) continue;
                const double hv = motionHeading(v);
                const double hb = motionHeading(b);
                const double dot = std::cos(hv) * std::cos(hb) +
                                   std::sin(hv) * std::sin(hb);
                if (dot > kHeadOnDot) continue;
                if (rule_engine_->priorityWinner(v, b) == v.id) continue;
                const double rev_speed =
                    rule_engine_->speedForAction(VehicleAction::CREEP);
                const double candidate_s =
                    std::max(0.0, v.path_s - rev_speed * dt);
                if (candidate_s >= v.path_s - 1e-9) continue;  // at path start
                if (!canPlace(i, candidate_s)) continue;       // someone behind
                blocked[i] = false;
                next_speed[i] = 0.0;
                next_s[i] = candidate_s;
                planned_s[i] = candidate_s;
                v.action = VehicleAction::CREEP;
                v.requested_action = VehicleAction::CREEP;
                v.reason = "deadlock_reverse_V" + std::to_string(bk);
                any_reversed = true;
            }
            if (any_reversed) resolvePlannedOverlaps();
        }

        for (size_t i = 0; i < agents_.size(); ++i) {
            VehicleAgent& v = agents_[i];
            if (!v.active()) continue;

            if (blocked[i]) {
                v.current_speed = 0.0;
                v.action = VehicleAction::STOP;
                v.requested_action = VehicleAction::STOP;
                v.reason = "hard_collision_guard";
                // §9 完整性:把"被谁的车身物理挡住"也回填为等待边,供 RuleEngine 的
                // 等待图(resolveDeadlock)检测环。否则硬护栏导致的互堵看不见、破不了。
                const double fwd_s = std::min(
                    v.track.length(),
                    v.path_s + rule_engine_->speedForAction(VehicleAction::CREEP) * dt);
                v.blocker_id = -1;
                for (size_t k = 0; k < agents_.size(); ++k) {
                    if (k == i) continue;
                    if (agents_[k].mode != VehicleMode::ACTIVE &&
                        agents_[k].mode != VehicleMode::DWELL) {
                        continue;
                    }
                    if (overlapsAt(i, fwd_s, k, plannedS(k))) {
                        v.blocker_id = agents_[k].id;
                        break;
                    }
                }
                continue;
            }

            v.current_speed = next_speed[i];
            v.path_s = next_s[i];

            if (v.path_s >= v.track.length() - 1e-9) {
                v.current_slot = v.target_slot;
                ++v.task_count;
                v.mode = VehicleMode::DWELL;
                v.action = VehicleAction::STOP;
                v.requested_action = VehicleAction::STOP;
                v.current_speed = 0.0;
                v.wait_time = 0.0;
                v.dwell_remaining = cfg_.dwell_time;
                v.reason = "dwell";
                // 批处理(长测)里关掉每次到位的 INFO——24h×8车×数千任务=2万+条,会把
                // 关键的"首撞/首楔"现场 dump 在 rosout 滚动里冲掉。实时/RViz 模式保留。
                if (cfg_batch_ticks_ == 0)
                    ROS_INFO("[multi_patrol] tick=%llu sim_t=%.2f V%d arrived slot %d; "
                             "dwell %.2fs; load=%s",
                             static_cast<unsigned long long>(tick_count_), sim_time_,
                             v.id, v.current_slot, cfg_.dwell_time,
                             v.loaded ? "loaded" : "empty");
            }
        }
    }

    void logAgentStatus() {
        const ros::Time now = ros::Time::now();
        for (size_t i = 0; i < agents_.size(); ++i) {
            const VehicleAgent& v = agents_[i];
            const bool changed =
                v.mode != last_logged_mode_[i] ||
                v.action != last_logged_action_[i] ||
                v.reason != last_logged_reason_[i] ||
                v.blocker_id != last_logged_blocker_[i] ||
                v.task_count != last_logged_task_count_[i];
            const bool stopped_active =
                v.mode == VehicleMode::ACTIVE && v.action == VehicleAction::STOP;
            const bool periodic =
                stopped_active &&
                (last_status_log_time_[i].isZero() ||
                 (now - last_status_log_time_[i]).toSec() >= 2.0);
            if (!changed && !periodic) continue;

            const double length = v.track.empty() ? 0.0 : v.track.length();
            const double rem = v.track.empty() ? 0.0 : v.remainingS();
            ROS_INFO("[multi_patrol][state] tick=%llu sim_t=%.2f V%d "
                     "mode=%s action=%s reason=%s "
                     "blocker=%d task=%d slot=%d->%d s=%.3f/%.3f rem=%.3f "
                     "speed=%.3f wait=%.2f dwell=%.2f",
                     static_cast<unsigned long long>(tick_count_), sim_time_,
                     v.id, modeName(v.mode), actionName(v.action),
                     v.reason.empty() ? "-" : v.reason.c_str(), v.blocker_id,
                     v.task_count, v.current_slot, v.target_slot, v.path_s,
                     length, rem, v.current_speed, v.wait_time,
                     v.dwell_remaining);

            last_logged_mode_[i] = v.mode;
            last_logged_action_[i] = v.action;
            last_logged_reason_[i] = v.reason;
            last_logged_blocker_[i] = v.blocker_id;
            last_logged_task_count_[i] = v.task_count;
            last_status_log_time_[i] = now;
        }
    }

    // TEMPORARY: dump relative geometry of any vehicle stuck (speed~0, wait>5s)
    // against its blocker, to classify head-on vs follower-misclassification vs
    // priority circularity. Remove once the V6/V7 deadlock root cause is fixed.
    void logStuckDiagnostics() {
        const ros::Time now = ros::Time::now();
        auto motionHeading = [](const VehicleAgent& v) {
            constexpr double kPi = 3.14159265358979323846;
            double h = v.track.poseAtS(v.path_s).theta;
            if (v.track.typeAtS(v.path_s) == WpType::REVERSE) h += kPi;
            return h;
        };
        for (size_t i = 0; i < agents_.size(); ++i) {
            const VehicleAgent& v = agents_[i];
            if (v.mode != VehicleMode::ACTIVE) continue;
            if (v.current_speed > 1e-3 || v.wait_time < 5.0) continue;
            if (!last_diag_time_[i].isZero() &&
                (now - last_diag_time_[i]).toSec() < 3.0) continue;
            last_diag_time_[i] = now;

            const RoughWp pv = v.track.poseAtS(v.path_s);
            const int wt = static_cast<int>(v.track.typeAtS(v.path_s));
            if (v.blocker_id < 0 ||
                v.blocker_id >= static_cast<int>(agents_.size())) {
                ROS_DEBUG("[DIAG stuck] V%d wait=%.1f reason=%s wp=%d "
                         "pose=(%.3f,%.3f) blocker=none",
                         v.id, v.wait_time, v.reason.c_str(), wt, pv.x, pv.y);
                continue;
            }
            const VehicleAgent& b = agents_[v.blocker_id];
            const RoughWp pb = b.track.poseAtS(b.path_s);
            const double hv = motionHeading(v);
            const double hb = motionHeading(b);
            const double dx = pb.x - pv.x;
            const double dy = pb.y - pv.y;
            const double dot = std::cos(hv) * std::cos(hb) +
                               std::sin(hv) * std::sin(hb);
            const double fwd = dx * std::cos(hv) + dy * std::sin(hv);
            const double lat = std::abs(-dx * std::sin(hv) + dy * std::cos(hv));
            const double gap = std::hypot(dx, dy) - mp_.vehicle_length;
            ROS_DEBUG("[DIAG stuck] V%d wait=%.1f reason=%s wp=%d | "
                     "blkV%d(act=%s spd=%.3f wp=%d) dot=%.2f fwd=%.3f lat=%.3f "
                     "gap=%.3f vw=%.3f",
                     v.id, v.wait_time, v.reason.c_str(), wt, b.id,
                     actionName(b.action), b.current_speed,
                     static_cast<int>(b.track.typeAtS(b.path_s)), dot, fwd, lat,
                     gap, mp_.vehicle_width);
        }
    }

    //==================== 最重要的节拍函数======================================
    void tick(const ros::TimerEvent&) {
        const double dt = 1.0 / pp_.update_rate;        //控制周期与仿真系统推移周期一致
        ++tick_count_;          //记录系统运行了多少隔周期
        sim_time_ += dt;        //仿真时间增加一个固定时间步长


        // 1. 实车模式---未摆放好姿态模式
        if (cfg_.real_mode) {
            if (!rb_started_) {
                marker_pub_->publish(agents_, rule_engine_->conflicts());   //发布车辆、地图
                publishRealTrailMarkers();  //发布真实车身尾迹
                logPlacementStatus();       //打印摆车状态
                return;
            }

        //2. 实车模式---选择一次性发布完整轨迹
            if (rb_one_shot_traj_) {
                if (rb_estop_ && !rb_estop_prev_) publishHoldAll();         //刚刚进入急停，给所有车发单点轨迹
                else if (!rb_estop_ && rb_estop_prev_) resumeFromEstop();   //刚刚解除急停，就从当前位置重新发轨迹
                rb_estop_prev_ = rb_estop_;

                if (!rb_estop_) {               //若没有急停
                    if (!one_shot_published_) one_shot_published_ = publishFullTrajectories();
                    updateDwellAndTasks(dt);        //检测到没有发送轨迹，一次性发布整条轨迹
                    rule_engine_->decide(agents_, dt);
                }
                realAdvance(dt);        //根据真实车身位置重新定位
                logAgentStatus();
                marker_pub_->publish(agents_, rule_engine_->conflicts());
                publishRealTrailMarkers();
                return;
            }

        //3. 实车模式----滚动时域规划
            updateDwellAndTasks(dt);    //更新任务---任务   dt 
            rule_engine_->decide(agents_, dt);      //规则调度---决策
            realAdvance(dt);        //  用真实位姿更新车辆状态---执行
            if (tick_count_ % 5 == 0) runDeadlockRecovery();        //5*0.1=0.5s进行一次死锁检测
            if (tick_count_ % rb_horizon_refresh_ == 0) publishHorizon();   //20*0.1=2s发布新的轨迹
            publishRealOutputs(dt);
            marker_pub_->publish(agents_, rule_engine_->conflicts());
            publishRealTrailMarkers();
            return;
        }

        //=======仿真模式=======
        // dt = 1.0 / pp_.update_rate; dt= 1 / 10 = 0.1s。

        //从当前仿真状态的一拍推进，相当于系统真实时间向前走了一个dt
        updateDwellAndTasks(dt);        //更新任务，到点停留并且分配新任务，生成路径
        rule_engine_->decide(agents_, dt);      //多车协调决策，分配速度
        advanceVehicles(dt);            //仿真推进


        //4. 仿真模式---一次性触发完成
        if (tick_count_ % 5 == 0) runDeadlockRecovery();

        if (rb_one_shot_traj_) {    
            if (!one_shot_published_) one_shot_published_ = publishFullTrajectories();
        }


        //5. 仿真模式-----滚动时域规划完成 0.1*20=2s
        else if (tick_count_ % rb_horizon_refresh_ == 0) {
            publishHorizon();       //从当前状态复制一份，临时向未来推演一段时间，生成未来轨迹
        }

        logAgentStatus();
        logStuckDiagnostics();
        marker_pub_->publish(agents_, rule_engine_->conflicts());
    }
    
    //===========================================================================


    void publishSimTrackMarkers() {
        visualization_msgs::MarkerArray arr;
        for (size_t i = 0; i < agents_.size(); ++i) {
            const RoughPath& path = agents_[i].track.path();
            if (path.size() < 2) continue;
            visualization_msgs::Marker m;
            m.header.frame_id = pp_.frame_id; m.header.stamp = ros::Time::now();
            m.ns = "sim_track"; m.id = static_cast<int>(i);
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0; m.scale.x = 0.02;
            m.color = agents_[i].color; m.color.a = 0.9;
            for (const auto& p : path) {
                geometry_msgs::Point gp; gp.x = p.x; gp.y = p.y; gp.z = 0.08;
                m.points.push_back(gp);
            }
            arr.markers.push_back(m);
        }
        if (!arr.markers.empty()) horizon_marker_pub_.publish(arr);
    }

    void publishRealTrailMarkers() {
        visualization_msgs::MarkerArray arr;
        const ros::Time now = ros::Time::now();
        for (size_t i = 0; i < real_trails_.size(); ++i) {
            if (!targetEnabled(static_cast<int>(i))) continue;
            if (real_trails_[i].size() < 2) continue;
            visualization_msgs::Marker m;
            m.header.frame_id = pp_.frame_id;
            m.header.stamp = now;
            m.ns = "real_trail";
            m.id = static_cast<int>(i);
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.018;
            if (i < agents_.size()) {
                m.color = agents_[i].color;
            } else {
                m.color = rgba(1.0f, 1.0f, 1.0f, 1.0f);
            }
            m.color.a = 1.0;
            for (const auto& p : real_trails_[i]) m.points.push_back(p);
            arr.markers.push_back(m);
        }
        if (!arr.markers.empty()) horizon_marker_pub_.publish(arr);
    }

    // ───────── 实车模式:I/O 建立 + 摆位打印 ─────────
    void setupRealIO() {
        const int n = cfg_.vehicle_count;
        traj_pubs_.resize(n);
        speed_pubs_.resize(n);
        state_pubs_.resize(n);
        real_x_.assign(n, 0.0);
        real_y_.assign(n, 0.0);
        real_yaw_.assign(n, 0.0);
        real_pose_ok_.assign(n, false);
        rb_prev_path_s_.assign(n, 0.0);
        rb_cmd_speed_.assign(n, 0.0);
        rb_last_seen_.assign(n, 0.0);
        rb_published_gen_.assign(n, -1);
        one_shot_done_.assign(n, false);
        rb_track_gen_.assign(n, -1);
        rb_logged_gen_.assign(n, -1);
        real_trails_.assign(n, {});
        for (int i = 0; i < n; ++i) {
            traj_pubs_[i] = nh_.advertise<sandbox_msgs::Trajectory>(
                "/traj_" + std::to_string(i), 1, /*latch=*/true);
            speed_pubs_[i] = nh_.advertise<std_msgs::Float64>(
                "/coord_speed_" + std::to_string(i), 1, /*latch=*/false);
            state_pubs_[i] = nh_.advertise<std_msgs::String>(
                "/coord_state_" + std::to_string(i), 1, /*latch=*/false);
        }
        object_sub_ = nh_.subscribe("/object", 20,
                                    &MultiVehiclePatrolNode::objectCallback, this);
        // 启动/急停键(由独立键盘节点 estop_key.py 发):Enter→/rb_start 开跑;空格→/estop 切换急停。
        // 独立节点是因为 roslaunch 起的本节点拿不到终端 stdin,键盘要在自己的终端 tab 里读。
        start_sub_ = nh_.subscribe("/rb_start", 1,
                                   &MultiVehiclePatrolNode::rbStartCallback, this);
        estop_sub_ = nh_.subscribe("/estop", 1,
                                   &MultiVehiclePatrolNode::estopCallback, this);
        // AD 滚动时域参数:推演/发布时长 + 刷新周期(拍)。
    
        rb_horizon_ = cfg_.rolling_horizon;
        rb_horizon_refresh_period_ = cfg_.rolling_refresh_period;
        rb_horizon_refresh_ = std::max(1, (int)std::lround(rb_horizon_refresh_period_ * pp_.update_rate));
       
        ROS_WARN("[real] AD rolling horizon: horizon=%.1fs, refresh every %d ticks",
         rb_horizon_, rb_horizon_refresh_);
        // 一次性整条轨迹模式(默认开):start 后推演全程发一次 latch,对接原版 pure_pursuit 按时间跟踪。
        
        rb_one_shot_traj_ = cfg_.one_shot_traj;
        ros::param::param("~full_horizon", rb_full_horizon_, rb_full_horizon_);
        ROS_WARN("[real] 轨迹发布模式: %s (full_horizon=%.0fs)",
                 rb_one_shot_traj_ ? "一次性整条/纯盲跟" : "AD滚动时域", rb_full_horizon_);
        // 起始摆位标记:发到现有 marker topic(不同 ns),RViz 不改配置即可显示足迹+ID+朝向。
        start_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_planner/markers", 1, /*latch=*/true);
        horizon_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_planner/markers", 10);   // 推演轨迹可视化(同topic不同ns,RViz直接显示)
        // 方案一:打印每辆车应摆放的真实坐标(track 起点)。
        ROS_WARN("==== 实车摆位(请把每辆车按编号摆到下列位置, 单位 m, yaw 弧度)====");
        std::ofstream ofs("/tmp/realbridge_positions.txt");
        ofs << "# 实车摆位(按编号)。id slot x y yaw,单位 m/rad。\n";
        for (const VehicleAgent& v : agents_) {
            if (v.track.empty()) { ROS_WARN("  车 %d: (无路径)", v.id); continue; }
            const auto p0 = v.track.poseAtS(0.0);
            ROS_WARN("  车 %d → 库位 %d, (x=%.3f, y=%.3f, yaw=%.3f)",
                     v.id, v.current_slot, p0.x, p0.y, p0.theta);
            char line[160];
            std::snprintf(line, sizeof(line), "%3d %4d %7.3f %7.3f %7.3f\n",
                          v.id, v.current_slot, p0.x, p0.y, p0.theta);
            ofs << line;
        }
        ofs.close();
        publishStartMarkers();  // RViz 画出每车起点:足迹框 + ID 数字 + 朝向箭头
    }

    // 起始摆位 RViz 标记:每车在 track 起点画 足迹框(LINE_STRIP)+ ID 文字 + 朝向箭头,
    // 各车独立颜色,latched 发到 /forklift_planner/markers(ns=start_*,与运行期 marker 不冲突)。
    void publishStartMarkers() {
        auto colorFor = [](int id) {
            static const float c[8][3] = {{1,0,0},{0,1,0},{0,0.5,1},{1,0.85,0},
                                          {1,0,1},{0,1,1},{1,0.5,0},{0.7,0.7,0.7}};
            std_msgs::ColorRGBA col; col.a = 1.0;
            col.r = c[id % 8][0]; col.g = c[id % 8][1]; col.b = c[id % 8][2];
            return col;
        };
        visualization_msgs::MarkerArray arr;
        for (const VehicleAgent& v : agents_) {
            if (v.track.empty()) continue;
            const RoughWp p0 = v.track.poseAtS(0.0);
            // 车身足迹:用半透明 CUBE(车身几何中心 + 朝向 + 车长×车宽),抬到 z 上方避免被
            // 地图遮住(之前 LINE_STRIP 太细且与地图同平面 → 看不见)。半透明可对位。
            const RoughWp bc = forklift_planner::multi_vehicle::bodyCenterPose(p0, mp_);
            visualization_msgs::Marker box;
            box.header.frame_id = pp_.frame_id;
            box.header.stamp = ros::Time::now();
            box.ns = "start_footprint"; box.id = v.id;
            box.type = visualization_msgs::Marker::CUBE;
            box.action = visualization_msgs::Marker::ADD;
            box.pose.position.x = bc.x; box.pose.position.y = bc.y; box.pose.position.z = 0.02;
            box.pose.orientation.z = std::sin(bc.theta / 2.0);
            box.pose.orientation.w = std::cos(bc.theta / 2.0);
            box.scale.x = mp_.vehicle_length; box.scale.y = mp_.vehicle_width; box.scale.z = 0.03;
            box.color = colorFor(v.id); box.color.a = 0.35;  // 半透明
            arr.markers.push_back(box);
            // ID 文字
            visualization_msgs::Marker txt;
            txt.header = box.header;
            txt.ns = "start_id"; txt.id = v.id;
            txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            txt.action = visualization_msgs::Marker::ADD;
            txt.pose.position.x = p0.x; txt.pose.position.y = p0.y; txt.pose.position.z = 0.12;
            txt.pose.orientation.w = 1.0;
            txt.scale.z = 0.10;
            txt.color = colorFor(v.id);
            txt.text = std::to_string(v.id);
            arr.markers.push_back(txt);
            // 朝向箭头:从车身中心沿车头方向画一条明显的箭头(抬到车身上方 z=0.06,
            // ARROW 用两点时 scale.x=杆径/scale.y=箭头径/scale.z=箭头长——之前 scale.z=0 没箭头)。
            visualization_msgs::Marker ar;
            ar.header = box.header;
            ar.ns = "start_heading"; ar.id = v.id;
            ar.type = visualization_msgs::Marker::ARROW;
            ar.action = visualization_msgs::Marker::ADD;
            geometry_msgs::Point a0, a1;
            a0.x = bc.x; a0.y = bc.y; a0.z = 0.06;
            a1.x = bc.x + 0.18 * std::cos(p0.theta);
            a1.y = bc.y + 0.18 * std::sin(p0.theta); a1.z = 0.06;
            ar.points.push_back(a0); ar.points.push_back(a1);
            ar.scale.x = 0.02; ar.scale.y = 0.045; ar.scale.z = 0.05;
            ar.color = colorFor(v.id); ar.color.a = 1.0;
            ar.pose.orientation.w = 1.0;
            arr.markers.push_back(ar);
        }
        // 地图原点+XY轴一并经 latched 话题先发(real_mode tick 要等所有车动捕就绪才publish,
        // 摆车前看不到轴 → 在这儿先发,标定时立刻可见)。
        marker_pub_->addOriginAxes(arr);
        start_marker_pub_.publish(arr);
        ROS_WARN("[multi_patrol] 已在 RViz 画出 8 车起始摆位(足迹+ID+朝向)+地图原点XY轴,对着摆即可。");
        ROS_WARN("[multi_patrol] 摆好后在【键盘节点终端】按 Enter 开跑;运行中按 空格 = 急停(再按解除)。");
    }

    // /object(动捕,mm,后轮中心=后轴参考)→ 各车真实位姿(转米)
    void objectCallback(const sandbox_msgs::AprilObject::ConstPtr& msg) {
        if (msg->type != sandbox_msgs::AprilObject::VEHICLE) return;
        const int id = msg->id;
        if (id < 0 || id >= static_cast<int>(agents_.size())) return;
        real_x_[id] = msg->x / 1000.0;  // mm→m(nokov 发 mm,pure_pursuit 也 /1000)
        real_y_[id] = msg->y / 1000.0;
        real_yaw_[id] = msg->yaw;
        real_pose_ok_[id] = true;
        rb_last_seen_[id] = ros::Time::now().toSec();  // 动捕看门狗:记最后一次见到的时刻
        // RViz 显示真实位姿(实际位置,非投影):同步进 agent 供 marker 用。
        agents_[id].real_pose_valid = true;
        agents_[id].real_x = real_x_[id];
        agents_[id].real_y = real_y_[id];
        agents_[id].real_yaw = real_yaw_[id];

        if (targetEnabled(id) && id < static_cast<int>(real_trails_.size())) {
            geometry_msgs::Point p;
            p.x = real_x_[id];
            p.y = real_y_[id];
            p.z = 0.12;
            auto& trail = real_trails_[id];
            const bool moved_enough =
                trail.empty() || std::hypot(p.x - trail.back().x, p.y - trail.back().y) > 0.01;
            if (moved_enough) {
                trail.push_back(p);
                constexpr size_t kMaxRealTrailPoints = 2000;
                while (trail.size() > kMaxRealTrailPoints) trail.pop_front();
            }
        }
    }

    // Enter:摆位完成、开跑。未全部就位也允许启动(未就位的车由动捕看门狗摁停),但会告警。
    void rbStartCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (!msg->data || rb_started_) return;
        int missing = 0;
        for (size_t i = 0; i < real_pose_ok_.size(); ++i) {
            if (targetEnabled(static_cast<int>(i)) && !real_pose_ok_[i]) ++missing;
        }
        rb_started_ = true;
        if (missing > 0)
            ROS_WARN("[real] 收到启动,但还有 %d 辆车动捕未就位 → 它们会被看门狗摁停,直到被看到。", missing);
        ROS_WARN("[real] *** 已启动:开始协调推进 ***");
    }

    // 空格:急停切换。true=全车瞬时停;再按一下=false 恢复协调速度。
    void estopCallback(const std_msgs::Bool::ConstPtr& msg) {
        rb_estop_ = msg->data;
        ROS_ERROR("[real] *** 急停 %s ***", rb_estop_ ? "已触发(全车停)" : "已解除(恢复)");
    }

    // 摆位阶段节流播报:哪些车动捕已到位 / 还缺哪些(否则启动门控是"哑"的,现场不知在等谁)。
    void logPlacementStatus() {
        std::string seen, miss;
        for (size_t i = 0; i < real_pose_ok_.size(); ++i) {
            if (!targetEnabled(static_cast<int>(i))) continue;
            (real_pose_ok_[i] ? seen : miss) += "V" + std::to_string(i) + " ";
        }
        if (miss.empty())
            ROS_WARN_THROTTLE(2.0, "[real] 全部就位 ✓ [%s] —— 去【启动/急停键盘】终端按 Enter 启动"
                              "(在打印本日志的终端里按 Enter 无效!)", seen.c_str());
        else
            ROS_WARN_THROTTLE(2.0, "[real] 摆位中:已到位 [%s] 还缺 [%s](等动捕看到)。"
                              "齐了去【启动/急停键盘】终端按 Enter。", seen.c_str(), miss.c_str());
    }

    static double projectOntoTrackRange(const forklift_planner::multi_vehicle::PathTrack& tr,
                                        double x, double y, double lo, double hi) {
        double best_s = lo, best_d2 = 1e18;
        for (double s = lo; s <= hi + 1e-9; s += 0.01) {
            const auto p = tr.poseAtS(s);
            const double d2 = (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y);
            if (d2 < best_d2) { best_d2 = d2; best_s = s; }
        }
        return best_s;
    }

    // 替代 advanceVehicles 的「位置推进」:实车位置取自 /object 投影,不做 sim 积分/硬护栏。
    // 到库→DWELL 逐字节复刻 advanceVehicles 705-714,保证和 sim 同样的"到点停 10s"。
    void realAdvance(double dt) {
        for (size_t i = 0; i < agents_.size(); ++i) {
            VehicleAgent& v = agents_[i];
            if (v.mode != VehicleMode::ACTIVE || v.track.empty()) continue;
            if (cfg_.real_mode && !real_pose_ok_[i]) continue;  // 动捕还没看到这辆 → 别拿 (0,0) 投影出垃圾 path_s
            if (rb_track_gen_[i] != v.path_gen) {  // 新任务/新路径 → path_s 归零
                rb_track_gen_[i] = v.path_gen;
                rb_prev_path_s_[i] = 0.0;
            }
            const double lo = std::max(0.0, rb_prev_path_s_[i] - 0.10);
            const double hi = std::min(v.track.length(), rb_prev_path_s_[i] + 0.50);
            double new_s = projectOntoTrackRange(v.track, real_x_[i], real_y_[i], lo, hi);
            new_s = std::max(new_s, rb_prev_path_s_[i]);  // 单调不减(沿固定路径只前进)
            v.current_speed = std::max(0.0, std::min((new_s - rb_prev_path_s_[i]) / dt,
                                                     cfg_.max_speed));
            v.path_s = new_s;
            rb_prev_path_s_[i] = new_s;
            // 到库→DWELL —— 复刻 advanceVehicles 705-714(实车容差 cfg_.real_arrive_tol)
            if (v.path_s >= v.track.length() - cfg_.real_arrive_tol) {
                // 关键(联动 rule_engine):sim 里 DWELL 车 path_s≈length,故其碰撞足迹
                // poseAtS(path_s)=槽位;rule_engine 73/85/737/742 处直接用 poseAtS(path_s)
                // 算静止车足迹(没套 DWELL?length:path_s)。实车若停在 length-5cm,这些足迹
                // 就比 sim 偏 5cm → 与停驻车的冲突判定偏离仿真。故到点即把 path_s 夹到 length,
                // 让"协调眼里的 DWELL 车"=精确槽位,与 sim 逐字节一致(实际 5cm 物理差归控制器管)。
                v.path_s = v.track.length();
                rb_prev_path_s_[i] = v.track.length();
                v.current_slot = v.target_slot;
                ++v.task_count;
                v.mode = VehicleMode::DWELL;
                v.action = VehicleAction::STOP;
                v.requested_action = VehicleAction::STOP;
                v.current_speed = 0.0;
                v.wait_time = 0.0;
                v.dwell_remaining = cfg_.dwell_time;
                v.reason = "dwell";
            }
        }
    }

    // 实车硬护栏(第二层兜底,替代 sim 里 advanceVehicles 的 hard_collision_guard):
    // 用真实 /object 位姿算两两足迹(充气 real_emergency_margin),重叠即双方急停。
    // 预测层(decide)是第一层;它漏判时这层兜住,防真车相撞。返回每车是否需急停。
    // 设计取舍:margin<预测层间距→正常不触;只在两车逼近到 <margin 才停(双停=安全优先,
    // 死活留给协调/死锁检测理顺)。DWELL/idle 车也算静态障碍(用其真实位姿)。
    std::vector<bool> realHardGuard() {
        std::vector<bool> estop(agents_.size(), false);
        const double m = cfg_.real_emergency_margin;
        if (m <= 0.0) return estop;  // 0 = 关闭
        auto realBody = [&](size_t i) {
            RoughWp p; p.x = real_x_[i]; p.y = real_y_[i];
            p.theta = real_yaw_[i]; p.type = WpType::FORWARD;
            return forklift_planner::multi_vehicle::makeBody(p, mp_, m);
        };
        for (size_t i = 0; i < agents_.size(); ++i) {
            if (cfg_.real_mode && !real_pose_ok_[i]) continue;
            const auto bi = realBody(i);
            for (size_t j = i + 1; j < agents_.size(); ++j) {
                if (!real_pose_ok_[j]) continue;
                if (forklift_planner::multi_vehicle::overlaps(bi, realBody(j))) {
                    // 只急停"在动的"那辆(ACTIVE);停驻/空闲车本就静止,标它无意义且会污染状态。
                    // 静态车是障碍方,移动车才是要被拦下的一方(若两辆都在动则都停)。
                    bool any = false;
                    if (agents_[i].mode == VehicleMode::ACTIVE) { estop[i] = true; any = true; }
                    if (agents_[j].mode == VehicleMode::ACTIVE) { estop[j] = true; any = true; }
                    if (any) {
                        ROS_ERROR_THROTTLE(0.5, "[real] 硬护栏急停: V%zu 与 V%zu 实测足迹逼近"
                                           "(<%.2fm)。预测层疑似漏判,查 logger 车间距/coord_flag。",
                                           i, j, m);
                    }
                }
            }
        }
        return estop;
    }

    // 曲率限速(规划侧运动学):弯道允许速度 v≤√(a_lat/κ)。用 path_s 附近三点位置估 κ(Menger,
    // 对动捕/cusp 比航向差稳);限到 [creep, +∞)避免曲率尖点把车停死。返回该车此刻的速度上限。
    double curvatureSpeed(const VehicleAgent& v) const {
        if (cfg_.lat_accel_max <= 0.0 || v.track.empty()) return 1e9;  // 关闭
        const double L = v.track.length(), ds = 0.05;
        const double s = std::min(std::max(v.path_s, 0.0), L);
        const auto A = v.track.poseAtS(std::max(0.0, s - ds));
        const auto B = v.track.poseAtS(s);
        const auto C = v.track.poseAtS(std::min(L, s + ds));
        const double abx=B.x-A.x, aby=B.y-A.y, acx=C.x-A.x, acy=C.y-A.y;
        const double lab=std::hypot(abx,aby), lbc=std::hypot(C.x-B.x,C.y-B.y), lac=std::hypot(acx,acy);
        if (lab<1e-4||lbc<1e-4||lac<1e-4) return 1e9;
        const double kappa = 2.0*std::fabs(abx*acy - aby*acx)/(lab*lbc*lac);  // Menger κ
        if (kappa < 1e-3) return 1e9;
        const double vc = std::sqrt(cfg_.lat_accel_max / kappa);
        const double v_floor = cfg_.nominal_speed * cfg_.creep_ratio;          // 弯再急也不低于 creep,不停死
        return std::max(vc, v_floor);
    }

    // 发 /traj_i(路径,换任务才重发) + /coord_speed_i(带符号实时速度,每拍)
    void publishRealOutputs(double dt) {
        const double now = ros::Time::now().toSec();
        const std::vector<bool> estop = realHardGuard();  // 第二层:真实足迹逼近→急停
        for (size_t i = 0; i < agents_.size(); ++i) {
            VehicleAgent& v = agents_[i];
            // 注:几何 /traj 已由 publishHorizon(滚动时域时间参数化轨迹)发布,这里不再发 /traj。
            // 速度幅值=协调动作档,再被曲率限速卡住(规划侧运动学:弯道降速)。方向=路径段(倒车负)。STOP→0。
            double mag = rule_engine_->speedForAction(v.action);
            mag = std::min(mag, curvatureSpeed(v));   // 曲率限速(lat_accel_max,0=关)
            // 方向:当前段倒车,或【前方一小段即将进入倒车段】→ 负(倒车)。后者关键:realAdvance 的
            // path_s 单调只增,前进逼近 FORWARD→REVERSE 的 cusp 时,path_s 越不过 cusp(前进会冲偏、
            // 投影卡在 cusp)→ 若只看 typeAtS(path_s) 永远 FORWARD → 车冲过该倒车处不倒(sim 积分不暴露)。
            // 故前瞻 0.10m:逼近 cusp 即提前给负速度,车减速→cusp 停→倒入倒车段→path_s 越过,死锁解开。
            double dir = 1.0;
            if (!v.track.empty()) {
                const double look_ahead = 0.10;  // m
                const bool rev = v.track.typeAtS(v.path_s) == WpType::REVERSE ||
                                 v.track.typeAtS(std::min(v.path_s + look_ahead, v.track.length()))
                                     == WpType::REVERSE;
                if (rev) dir = -1.0;
            }
            double target = dir * mag;
            // 动捕看门狗:该车位姿失联(>0.5s)→ 强制目标速度 0。否则控制器拿陈旧位姿
            // 盲走、底盘又无超时 → 跑飞。只压这一辆的输出速度,协调逻辑/其它车不受影响
            //(协调端看它 path_s 不动,自会让旁车等它,安全)。
            // real_pose_timeout ≤ 0 → 关闭动捕失联停(与 emerg_margin=0、cmd_timeout≤0 统一约定)
            const bool stale = cfg_.real_pose_timeout > 0.0 &&
                               (now - rb_last_seen_[i]) > cfg_.real_pose_timeout;
            const char* flag = "OK";
            bool instant_zero = false;
            if (rb_estop_) {          // 操作员空格急停:最高优先级,瞬时 0(不走斜坡,要快)
                target = 0.0; instant_zero = true; flag = "EKEY";
            } else if (estop[i]) {    // 第二层硬护栏:真实足迹逼近 → 急停
                target = 0.0;
                flag = "ESTOP";
            } else if (stale) {       // 动捕失联 → 强制 0,防控制器拿陈旧位姿盲走跑飞
                target = 0.0;
                flag = "STALE";
                ROS_WARN_THROTTLE(1.0, "[real] 车 %zu 动捕失联 %.1fs → 强制STOP",
                                  i, now - rb_last_seen_[i]);
            }
            // 加速度限制(与 sim 一致):按 max_accel/max_decel 斜坡逼近目标,而非阶跃。
            // 这样真车速度剖面 = 仿真,停在协调按 max_decel 算的停止线处;cusp 处平滑过 0 反向。
            // 注:硬护栏/STALE 走斜坡降速(避免甩动,距离近时本就低速);操作员急停 EKEY 例外,瞬时归 0。
            if (instant_zero) rb_cmd_speed_[i] = 0.0;
            else rb_cmd_speed_[i] = limitedSpeed(rb_cmd_speed_[i], target, dt);
            std_msgs::Float64 sp;
            sp.data = rb_cmd_speed_[i];
            speed_pubs_[i].publish(sp);

            // 只读协调状态。格式 "mode,action,blk,brk,wait,slot->tgt,flag,seg,s/len" 供 logger 落列。
            // seg=当前路径段方向(FWD/REV,来自 typeAtS(path_s));s/len=路径进度——排查"该倒车却前进"。
            const char* seg = (dir < 0.0) ? "REV" : "FWD";
            const double len = v.track.empty() ? 0.0 : v.track.length();
            char buf[140];
            std::snprintf(buf, sizeof(buf), "%s,%s,%d,%d,%.1f,%d->%d,%s,%s,%.2f/%.2f",
                          modeName(v.mode), actionName(v.action), v.blocker_id,
                          v.deadlock_breaker ? 1 : 0, v.wait_time,
                          v.current_slot, v.target_slot, flag, seg, v.path_s, len);
            std_msgs::String st;
            st.data = buf;
            state_pubs_[i].publish(st);
            // 在动时节流打印:进度+段向+协调速度,实时看"倒车段是否被识别、coord_speed 是否变负"。
            if (v.mode == VehicleMode::ACTIVE)
                ROS_INFO_THROTTLE(1.0, "[real] V%zu s=%.2f/%.2f seg=%s coord_speed=%+.2f action=%s",
                                  i, v.path_s, len, seg, rb_cmd_speed_[i], actionName(v.action));
        }
    }

    void publishTraj(int id, const forklift_planner::multi_vehicle::PathTrack& tr) {
        sandbox_msgs::Trajectory msg;
        msg.target = id;
        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();
        const double len = tr.length();
        std::string segs; bool prev_rev = false; double seg_start = 0.0;  // 诊断:倒车段 s 区间
        for (double s = 0.0; s <= len + 1e-9; s += 0.02) {
            const double ss = std::min(s, len);
            const auto p = tr.poseAtS(ss);
            const bool rev = (tr.typeAtS(ss) == WpType::REVERSE);
            sandbox_msgs::TrajectoryPoint tp;
            tp.x = p.x; tp.y = p.y; tp.yaw = p.theta;
            tp.velocity = rev ? -1.0 : 1.0;  // 方向标记;速度幅值走 /coord_speed_i
            tp.time = 0.0;
            msg.points.push_back(tp);
            if (rev && !prev_rev) seg_start = ss;
            if (!rev && prev_rev) segs += "[" + std::to_string(seg_start).substr(0,4) + "," +
                                            std::to_string(ss).substr(0,4) + "] ";
            prev_rev = rev;
        }
        if (prev_rev) segs += "[" + std::to_string(seg_start).substr(0,4) + "," +
                                  std::to_string(len).substr(0,4) + "]";
        ROS_WARN("[real] V%d 新路径 len=%.2f  倒车段 s=%s", id, len,
                 segs.empty() ? "无(全程前进)" : segs.c_str());
        traj_pubs_[id].publish(msg);
    }

    ros::NodeHandle nh_;
    ros::Timer timer_;

    MapParam mp_;
    PlannerParam pp_;
    forklift_planner::multi_vehicle::MultiVehicleConfig cfg_;

    std::unique_ptr<ForkliftMap> map_;
    std::unique_ptr<PathGenerator> generator_;
    std::unique_ptr<forklift_planner::multi_vehicle::TaskAllocator> allocator_;
    std::unique_ptr<forklift_planner::multi_vehicle::RuleEngine> rule_engine_;
    bool one_shot_ = true;   // A方案 demo:8车各跑一程 A→B 到达即停,不持续巡逻(可被 ~one_shot 覆盖)
    std::unique_ptr<forklift_planner::multi_vehicle::MarkerPublisher> marker_pub_;
    std::unique_ptr<forklift_planner::multi_vehicle::TrafficResourceMap> resource_map_;
    std::vector<VehicleAgent> agents_;
    int target_only_ = -1;  // realbridge debug: -1 = all vehicles, otherwise control only this id

    // ── 实车模式(real_mode)I/O ──────────────────────────────────────────────
    ros::Subscriber object_sub_;                       // /object 动捕位姿(mm)
    ros::Publisher start_marker_pub_;                  // 起始摆位标记(足迹框+ID+朝向,latched)
    std::vector<ros::Publisher> traj_pubs_, speed_pubs_;// /traj_i(路径) + /coord_speed_i(实时速度)
    std::vector<ros::Publisher> state_pubs_;            // /coord_state_i(只读调试:停车原因)
    ros::Publisher horizon_marker_pub_;                 // 推演 5s 轨迹的 RViz 可视化(LINE_STRIP/车)
    ros::Subscriber start_sub_, estop_sub_;             // /rb_start(Enter开跑) /estop(空格急停切换)
    bool rb_started_ = false;                           // 摆位完成、按Enter后才推进
    bool rb_estop_ = false;                             // 操作员急停:true=全车瞬时停
    bool rb_estop_prev_ = false;                        // 上一拍急停态(检测按下/解除边沿,one_shot急停用)
    
    double rb_horizon_ = 10.0;                           // AD 滚动时域:每次推演/发布的未来时长(s)
    double rb_horizon_refresh_period_ = 2.0;
    int rb_horizon_refresh_ = 20;                        // 每多少拍重新推演刷新一次(5拍=0.5s)
    bool rb_one_shot_traj_ = false;                      // 一次性整条轨迹模式(默认):start后推演全程发一次latch
    
    double rb_full_horizon_ = 180.0;                    // 一次性模式:全程推演上限时长(s),尾部静止点会裁掉
    bool one_shot_published_ = false;                   // 一次性轨迹是否已全部发完(防重复发)
    std::vector<bool> one_shot_done_;                   // 各车一次性轨迹是否已发(动捕晚到的车就位后补发)
    std::vector<int> rb_logged_gen_;                    // 已打印倒车段诊断的 path_gen(每车,防重复刷屏)
    std::vector<double> real_x_, real_y_, real_yaw_;   // 各车真实位姿(已 /1000 转米)
    std::vector<std::deque<geometry_msgs::Point>> real_trails_; // RViz 实车走过的真实轨迹(ns=real_trail)
    std::vector<bool> real_pose_ok_;                   // 动捕是否已收到该车
    std::vector<double> rb_prev_path_s_;               // 上一拍 path_s(局部投影+差分估速度)
    std::vector<double> rb_cmd_speed_;                 // 上一拍发出的速度命令(带符号),斜坡限速用
    std::vector<double> rb_last_seen_;                 // 各车动捕最后到达时刻(s),看门狗用
    // 动捕失联超时、到点容差 → 已提为 ROS 参数 cfg_.real_pose_timeout / real_arrive_tol,
    // 现场可不重编译直接调。到点容差需 > PP 的 2cm 硬停容差,否则 PP 停在 2cm 短处而 path_s
    // 到不了 length → 永不 DWELL → 不触发下一个任务(卡死);默认 5cm。
    std::vector<int> rb_published_gen_, rb_track_gen_; // 已发布的 path_gen / 上次见到的 path_gen
    std::vector<VehicleMode> last_logged_mode_;
    std::vector<VehicleAction> last_logged_action_;
    std::vector<std::string> last_logged_reason_;
    std::vector<int> last_logged_blocker_;
    std::vector<int> last_logged_task_count_;
    std::vector<ros::Time> last_status_log_time_;
    std::vector<ros::Time> last_diag_time_;  // TEMPORARY: [DIAG stuck] throttle
    unsigned long long tick_count_ = 0;
    double sim_time_ = 0.0;

    // 无头批处理(快速回归)统计。确定性仿真:同种子同代码必得同结果,故脱离 RViz、
    // 不按实时狂跑 N 拍即可在几秒内覆盖数小时仿真,直接数碰撞。
    unsigned long long hard_guard_events_ = 0;        // 硬护栏触发(碰撞)累计次数
    unsigned long long first_guard_tick_ = 0;         // 首次碰撞所在 tick(0=从未)
    std::set<std::pair<int, int>> hard_guard_pairs_;  // 涉及碰撞的车对
    unsigned long long deadlock_ticks_ = 0;           // 检测到持续死锁环的拍数累计
    unsigned long long deadlock_recoveries_ = 0;      // 成功重规划脱困次数
    bool sim_mode_ = false;                           // 前瞻仿真中:屏蔽计数/日志副作用
    std::map<int, int> predict_holds_;                // 车id→连续"预测性错峰扣车"次数(防极端饥饿)
    bool deadlock_logged_ = false;                    // 首次死锁是否已详打
    std::set<std::set<int>> dumped_clusters_;         // 已详打过的死锁簇(按成员集去重,编目用)
    std::map<int, double> last_replan_t_;             // 车id→上次重规划 sim_t(冷却用)

public:
    // 一辆车的紧凑状态行(诊断用,信息尽量全)。
    std::string vehLine(const VehicleAgent& v) const {
        char buf[256];
        snprintf(buf, sizeof(buf),
                 "  V%d mode=%d act=%d reason=%s blk=%d brkr=%d task=%d slot=%d->%d "
                 "s=%.3f/%.3f rem=%.3f spd=%.3f wait=%.1f gen=%d",
                 v.id, (int)v.mode, (int)v.action, v.reason.c_str(), v.blocker_id,
                 (int)v.deadlock_breaker, v.task_count, v.current_slot,
                 v.target_slot, v.path_s, v.track.length(), v.remainingS(),
                 v.current_speed, v.wait_time, v.path_gen);
        return buf;
    }
    std::string fleetSnapshot() const {
        std::string s = "tick=" + std::to_string(tick_count_);
        for (const VehicleAgent& v : agents_) s += "\n" + vehLine(v);
        return s;
    }
    VehicleAgent* agentById(int id) {
        for (VehicleAgent& v : agents_)
            if (v.id == id) return &v;
        return nullptr;
    }
    // 打印某对车的冲突几何(委托 RuleEngine,拿到 se/sx/same_dir/committed/owner/following)。
    void dumpPair(int ia, int ib) {
        VehicleAgent* a = agentById(ia);
        VehicleAgent* b = agentById(ib);
        if (a && b) rule_engine_->debugDumpConflict(*a, *b);
    }

    // 死锁看门狗(C-第1步):跟 blocker_id 等待图找一个「持续死锁环」——环内每辆都
    // ACTIVE+STOP 且 wait_time≥min_wait(确为持续、非瞬时)。返回环成员 id(按链序),无则空。
    std::vector<int> findDeadlockCycle(double min_wait) const {
        auto idxOf = [&](int id) -> int {
            for (size_t i = 0; i < agents_.size(); ++i)
                if (agents_[i].id == id) return static_cast<int>(i);
            return -1;
        };
        for (const VehicleAgent& v0 : agents_) {
            if (v0.mode != VehicleMode::ACTIVE) continue;
            std::vector<int> path;
            int cur = v0.id;
            while (cur >= 0) {
                auto it = std::find(path.begin(), path.end(), cur);
                if (it != path.end()) {  // 链绕回 → 环 = [it, end)
                    std::vector<int> cyc(it, path.end());
                    bool all_stuck = cyc.size() >= 2;
                    for (int cid : cyc) {
                        int k = idxOf(cid);
                        if (k < 0 || agents_[k].wait_time < min_wait) { all_stuck = false; break; }
                    }
                    return all_stuck ? cyc : std::vector<int>{};
                }
                path.push_back(cur);
                int k = idxOf(cur);
                if (k < 0 || agents_[k].mode != VehicleMode::ACTIVE ||
                    agents_[k].action != VehicleAction::STOP)
                    break;
                cur = agents_[k].blocker_id;
            }
        }
        return {};
    }

    // 死锁恢复(C):检测「所有」持续环的成员 → 选其中等待最久(最该救)且不在冷却期的车,
    // 从当前位姿重规划到空库位脱困。不倒车、不强推。冷却防止反复重规划同一辆(churn)。
    void runDeadlockRecovery() {
        constexpr double kDeadlockWait = 25.0;  // 持续卡 >此秒数才算真死锁
        constexpr double kCooldown = 8.0;       // 同一辆两次重规划的最小间隔(给它时间驶离)
        const std::set<int> members = findDeadlockMembers(kDeadlockWait);
        if (members.empty()) return;
        ++deadlock_ticks_;
        // 选等待最久、且不在冷却期的成员当受害车。
        int victim = -1;
        double worst_wait = -1.0;
        for (int id : members) {
            const VehicleAgent* a = agentById_c(id);
            if (!a) continue;
            auto it = last_replan_t_.find(id);
            if (it != last_replan_t_.end() && sim_time_ - it->second < kCooldown) continue;
            if (a->wait_time > worst_wait) { worst_wait = a->wait_time; victim = id; }
        }
        if (victim < 0) return;  // 全在冷却 → 等下一拍
        if (!deadlock_logged_) {
            deadlock_logged_ = true;
            std::string ms;
            for (int id : members) ms += "V" + std::to_string(id) + " ";
            ROS_ERROR("[DEADLOCK] @tick=%llu sim_t=%.1fs 环成员=[%s] 首个受害车=V%d",
                      tick_count_, sim_time_, ms.c_str(), victim);
        }
        // 编目:每种不同的死锁簇(按成员集去重)各全面 dump 一次,catalog 出"到底几种特例"。
        if (dumped_clusters_.size() < 15 && dumped_clusters_.insert(members).second) {
            ROS_ERROR("[CLUSTER-CATALOG] 第 %zu 种 @tick=%llu sim_t=%.1fs",
                      dumped_clusters_.size(), tick_count_, sim_time_);
            dumpDeadlockCluster(members);
        }
        // 重规划脱困整套停用(enable_deadlock_recovery=false)。看门狗保留
        // 为纯检测(上方 deadlock_ticks 计数 + 首环日志),不再执行任何重规划/脱困动作。
        if (!cfg_.enable_deadlock_recovery) return;
        VehicleAgent* v = agentById(victim);
        if (!v) return;
        // 冷却按「尝试」记,而非仅「成功」记:换货位失败(当时清不出逃逸位)时,也冷却 8s 再试,
        // 别每 5 拍就对一堆候选位重算路径(实测那是 24.8 万条 clothoid 警告 + 大量 CPU 空转的根因)。
        last_replan_t_[victim] = sim_time_;
        if (allocator_->replanFromPose(*v, agents_)) {
            ++deadlock_recoveries_;
        }
    }

    const VehicleAgent* agentById_c(int id) const {
        for (const VehicleAgent& v : agents_) if (v.id == id) return &v;
        return nullptr;
    }

    // 一次性全面 dump 首个持续死锁簇:每个成员的路径要点 + 簇内两两冲突几何(same_dir 决定
    // 对向/同向 → 判定单向环流能否治)。只打一次,只读。用于源头修复的精确诊断。
    void dumpDeadlockCluster(const std::set<int>& members) {
        ROS_ERROR("[CLUSTER-DUMP] ===== 死锁簇 %zu 车,逐车路径 + 两两冲突几何 =====",
                  members.size());
        // 1) 每个成员:槽位、当前位姿/进度、路径起讫 + 等弧长 11 点折线。
        for (int id : members) {
            const VehicleAgent* a = agentById_c(id);
            if (!a || a->track.empty()) continue;
            const double L = a->track.length();
            const RoughWp cur = a->track.poseAtS(a->path_s);
            const RoughWp p0 = a->track.poseAtS(0.0);
            const RoughWp pe = a->track.poseAtS(L);
            ROS_ERROR("[CLUSTER-DUMP] V%d slot=%d->%d mode=%d act=%d reason=%s blk=%d "
                      "s=%.3f/%.3f cur(%.2f,%.2f,%.0fdeg) start(%.2f,%.2f) end(%.2f,%.2f)",
                      id, a->current_slot, a->target_slot, (int)a->mode, (int)a->action,
                      a->reason.c_str(), a->blocker_id, a->path_s, L,
                      cur.x, cur.y, cur.theta * 180.0 / M_PI, p0.x, p0.y, pe.x, pe.y);
            std::string poly;
            for (int k = 0; k <= 10; ++k) {
                const RoughWp p = a->track.poseAtS(L * k / 10.0);
                char buf[48];
                snprintf(buf, sizeof(buf), "(%.2f,%.2f%s)", p.x, p.y,
                         a->track.typeAtS(L * k / 10.0) == WpType::REVERSE ? "R" : "");
                poly += buf;
                if (k < 10) poly += "->";
            }
            ROS_ERROR("[CLUSTER-DUMP]   V%d path: %s", id, poly.c_str());
        }
        // 2) 簇内每对成员的冲突几何(same_dir / 物理坐标 / committed)。
        std::vector<int> ids(members.begin(), members.end());
        for (size_t i = 0; i < ids.size(); ++i) {
            for (size_t j = i + 1; j < ids.size(); ++j) {
                const VehicleAgent* a = agentById_c(ids[i]);
                const VehicleAgent* b = agentById_c(ids[j]);
                if (a && b) rule_engine_->debugDumpConflict(*a, *b);
            }
        }
        ROS_ERROR("[CLUSTER-DUMP] ===== 簇 dump 结束 =====");
    }

    // 持久 onset 文件:把关键现场同时写到 /tmp/forklift_onset.log(永不随 rosout 滚掉,我随时能读)。
    // 长测排错专用——只在出问题那一刻写,故文件小、不刷屏。
    void onsetLog(const std::string& s) {
        ROS_ERROR("%s", s.c_str());
        std::ofstream f("/tmp/forklift_onset.log", std::ios::app);
        if (f) f << s << "\n";
    }
    // 把碰撞/楔死前的全队历史(含 gen=path_gen:刚被 recovery 重规划过则 gen 跳变=churn 撞)
    // 写进持久文件,供事后根因。
    void onsetDumpHist(const std::string& header, const std::deque<std::string>& hist) {
        std::ofstream f("/tmp/forklift_onset.log", std::ios::app);
        if (!f) return;
        f << "\n========== " << header << " ==========\n";
        for (const std::string& snap : hist) f << snap << "\n";
        f.flush();
    }

    // 找「所有」持续死锁环的成员并集:对每辆车跟 blocker 链,若绕回自身则其环成员全部入集。
    std::set<int> findDeadlockMembers(double min_wait) const {
        auto idxOf = [&](int id) -> int {
            for (size_t i = 0; i < agents_.size(); ++i)
                if (agents_[i].id == id) return static_cast<int>(i);
            return -1;
        };
        std::set<int> members;
        for (const VehicleAgent& v0 : agents_) {
            if (v0.mode != VehicleMode::ACTIVE) continue;
            std::vector<int> path;
            int cur = v0.id;
            while (cur >= 0) {
                auto it = std::find(path.begin(), path.end(), cur);
                if (it != path.end()) {  // 绕回 → 环 = [it, end)
                    bool all_stuck = (path.end() - it) >= 2;
                    for (auto p = it; p != path.end(); ++p) {
                        int k = idxOf(*p);
                        if (k < 0 || agents_[k].wait_time < min_wait) { all_stuck = false; break; }
                    }
                    if (all_stuck) for (auto p = it; p != path.end(); ++p) members.insert(*p);
                    break;
                }
                path.push_back(cur);
                int k = idxOf(cur);
                if (k < 0 || agents_[k].mode != VehicleMode::ACTIVE ||
                    agents_[k].action != VehicleAction::STOP)
                    break;
                cur = agents_[k].blocker_id;
            }
        }
        return members;
    }

    // 批处理模式:紧凑循环跑 ticks 拍(跳过 marker)。维护近 N 拍环形历史;首次碰撞那拍
    // dump 全队历史+碰撞对几何,并对其后 kPost 拍逐拍详打;结尾 dump 永久楔死现场。
    bool runBatch(unsigned long long ticks) {
        const double dt = 1.0 / pp_.update_rate;
        std::ofstream("/tmp/forklift_onset.log", std::ios::trunc);  // 每次运行清空持久现场文件
        const unsigned long long progress = ticks / 10 ? ticks / 10 : 1;
        constexpr size_t kHist = 80;    // 碰撞前回看的拍数
        constexpr unsigned long long kPost = 150;  // 碰撞后逐拍详打的拍数
        std::deque<std::string> hist;
        bool first_dumped = false;
        bool wedge_dumped = false;
        bool multiwedge_dumped = false;
        unsigned long long verbose_until = 0;

        for (unsigned long long k = 0; k < ticks && ros::ok(); ++k) {
            ++tick_count_;
            sim_time_ += dt;
            updateDwellAndTasks(dt);
            rule_engine_->decide(agents_, dt);
            const unsigned long long guards_before = hard_guard_events_;
            advanceVehicles(dt);
            const bool new_collision = hard_guard_events_ > guards_before;

            hist.push_back(fleetSnapshot());
            if (hist.size() > kHist) hist.pop_front();

            if (new_collision && !first_dumped) {
                first_dumped = true;
                verbose_until = tick_count_ + kPost;
                std::string cp;
                for (const auto& q : hard_guard_pairs_)
                    cp += "V" + std::to_string(q.first) + "-V" +
                          std::to_string(q.second) + " ";
                onsetLog("[FIRST-COLLISION] @tick=" + std::to_string(tick_count_) +
                         " sim_t=" + std::to_string((long long)sim_time_) +
                         "s 涉及对=[" + cp + "]");
                // churn 信号:每个涉事车「距上次脱困重规划多久」——刚被重规划(Δt 很小)= 很可能
                // 是 recovery 把它重置成与他车重叠的新 track 撞出来的。
                for (const auto& q : hard_guard_pairs_) {
                    for (int id : {q.first, q.second}) {
                        auto it = last_replan_t_.find(id);
                        const double dt_re = (it == last_replan_t_.end())
                            ? -1.0 : (sim_time_ - it->second);
                        const VehicleAgent* a = agentById_c(id);
                        onsetLog("[FIRST-COLLISION]   V" + std::to_string(id) +
                                 " 距上次脱困=" + (dt_re < 0 ? std::string("从未") :
                                     std::to_string((long long)dt_re) + "s") +
                                 " gen=" + std::to_string(a ? a->path_gen : -1) +
                                 " slot=" + std::to_string(a ? a->current_slot : -1) +
                                 "->" + std::to_string(a ? a->target_slot : -1));
                    }
                }
                onsetDumpHist("FIRST-COLLISION 前 " + std::to_string(hist.size()) +
                              " 拍历史(看 gen 是否刚跳变=刚被脱困重置)", hist);
                ROS_WARN("[HIST] ====== 碰撞前 %zu 拍全队历史 ======", hist.size());
                for (const std::string& snap : hist) ROS_WARN("[HIST]\n%s", snap.c_str());
                ROS_WARN("[HIST] ====== 碰撞对冲突几何 ======");
                for (const auto& q : hard_guard_pairs_) dumpPair(q.first, q.second);
            }

            // 多车紧楔 onset(24h 杀手):≥3 车持续闭环(wait≥15s)首次出现 → 一次性 dump 到
            // 持久文件 + 历史。这是 B(40s前瞻)够不着、recovery 治不了的那类,要抓它怎么形成的。
            if (!multiwedge_dumped) {
                const std::set<int> mw3 = findDeadlockMembers(15.0);
                if (mw3.size() >= 3) {
                    multiwedge_dumped = true;
                    std::string ms;
                    for (int id : mw3) ms += "V" + std::to_string(id) + " ";
                    onsetLog("[MULTIWEDGE] @tick=" + std::to_string(tick_count_) +
                             " sim_t=" + std::to_string((long long)sim_time_) +
                             "s ≥3车紧楔首现 成员=[" + ms + "]");
                    onsetDumpHist("MULTIWEDGE 前 " + std::to_string(hist.size()) + " 拍历史", hist);
                    dumpDeadlockCluster(mw3);
                }
            }

            // 楔死现场一次性诊断:任一车 wait 首次超阈值 → 回放历史 + 全队 + 卡死车几何。
            if (!wedge_dumped) {
                int sid = -1; double mw = 0.0;
                for (const VehicleAgent& v : agents_)
                    if (v.wait_time > mw) { mw = v.wait_time; sid = v.id; }
                if (mw > 25.0 && sid >= 0) {
                    wedge_dumped = true;
                    ROS_ERROR("[FIRST-WEDGE] @tick=%llu sim_t=%.1fs 最久=V%d wait=%.1fs"
                              " —— 回放前 %zu 拍历史 + 卡死车几何 ===",
                              tick_count_, sim_time_, sid, mw, hist.size());
                    for (const std::string& snap : hist)
                        ROS_WARN("[WHIST]\n%s", snap.c_str());
                    for (const VehicleAgent& v : agents_)
                        if (v.id != sid) dumpPair(sid, v.id);
                }
            }

            if (tick_count_ <= verbose_until) {
                ROS_WARN("[POST]\n%s", fleetSnapshot().c_str());
                for (const auto& q : hard_guard_pairs_) dumpPair(q.first, q.second);
            }

            // 死锁看门狗(C):检测持续环 → 受害车从当前位姿重规划脱困。每 5 拍查一次。
            if (k % 5 == 0) runDeadlockRecovery();

            if ((k + 1) % progress == 0) {
                ROS_INFO("[batch] %llu/%llu ticks (sim_t=%.0fs) hard_guard=%llu",
                         k + 1, ticks, sim_time_, hard_guard_events_);
            }
        }

        // 结尾:找等待最久(永久楔死)的车,dump 它与所有其它车的冲突几何 + 全队快照。
        double max_wait = 0.0;
        int stuck_id = -1;
        for (const VehicleAgent& v : agents_)
            if (v.wait_time > max_wait) { max_wait = v.wait_time; stuck_id = v.id; }
        ROS_WARN("[END] ====== 结尾全队快照 ======\n%s", fleetSnapshot().c_str());
        if (stuck_id >= 0 && max_wait > 20.0) {
            ROS_WARN("[END] 永久楔死嫌疑=V%d (wait=%.1fs),其与各车冲突几何:",
                     stuck_id, max_wait);
            for (const VehicleAgent& v : agents_)
                if (v.id != stuck_id) dumpPair(stuck_id, v.id);
        }
        std::string pairs;
        for (const auto& p : hard_guard_pairs_)
            pairs += "V" + std::to_string(p.first) + "-V" +
                     std::to_string(p.second) + " ";
        ROS_WARN("[batch] ==== 汇总: ticks=%llu sim_t=%.0fs | 碰撞(hard_guard)事件=%llu "
                 "首次@tick=%llu 涉及对=[%s] | 死锁检出拍=%llu 重规划脱困=%llu | "
                 "最大wait=%.1fs(V%d) ====",
                 tick_count_, sim_time_, hard_guard_events_, first_guard_tick_,
                 pairs.c_str(), deadlock_ticks_, deadlock_recoveries_, max_wait,
                 stuck_id);
        return hard_guard_events_ > 0;
    }
    bool batchMode() const { return cfg_batch_ticks_ > 0; }
    unsigned long long batchTicks() const { return cfg_batch_ticks_; }

private:
    unsigned long long cfg_batch_ticks_ = 0;
};

int main(int argc, char** argv) {
    // 用环境 locale(通常 UTF-8)初始化 C/C++ 本地化,否则默认 "C" locale 会把日志里的
    // 中文打成 ???。一句即可,根治 rosconsole/printf 中文乱码。
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "multi_vehicle_patrol_node");
    MultiVehiclePatrolNode node;

    if (node.batchMode()) {
        // 无头快速回归:狂跑后退出。返回码 1=出现碰撞,0=干净(便于脚本判定)。
        const bool collided = node.runBatch(node.batchTicks());
        ros::shutdown();
        return collided ? 1 : 0;
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
