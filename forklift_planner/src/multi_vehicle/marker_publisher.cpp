#include "forklift_planner/multi_vehicle/marker_publisher.h"

#include <geometry_msgs/Point.h>

#include <cmath>
#include <sstream>

#include "forklift_planner/multi_vehicle/footprint.h"

namespace forklift_planner {
namespace multi_vehicle {

namespace {

geometry_msgs::Point pt3(double x, double y, double z = 0.08) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

std_msgs::ColorRGBA rgba(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

RoughWp displayPose(const VehicleAgent& v) {
    // 实车:显示真实 /object 位姿(实际在哪就画在哪,偏离路径多少看得见),而非投影点。
    if (v.real_pose_valid) {
        RoughWp p; p.x = v.real_x; p.y = v.real_y; p.theta = v.real_yaw;
        p.type = WpType::FORWARD;
        return p;
    }
    if (v.track.empty()) return {};
    if (v.mode == VehicleMode::DWELL) return v.track.poseAtS(v.track.length());
    return v.track.poseAtS(v.path_s);
}

}  // namespace

MarkerPublisher::MarkerPublisher(ros::NodeHandle& nh, const MapParam& mp,
                                 const PlannerParam& pp,
                                 const MultiVehicleConfig& cfg)
    : mp_(mp), pp_(pp), cfg_(cfg) {
    pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/forklift_planner/markers", 10);
}

void MarkerPublisher::addPathMarker(visualization_msgs::MarkerArray& arr,
                                    const VehicleAgent& v) const {
    constexpr int kPathPublishStride = 10;
    if (cfg_.show_paths && publish_seq_ % kPathPublishStride != 0) {
        return;
    }

    visualization_msgs::Marker m;
    m.header.frame_id = pp_.frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = "multi_patrol_path";
    m.id = v.id;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = cfg_.show_paths ? visualization_msgs::Marker::ADD
                               : visualization_msgs::Marker::DELETE;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.016;
    m.color = v.color;
    if (cfg_.show_paths) {
        for (const RoughWp& p : v.track.path()) {
            m.points.push_back(pt3(p.x, p.y, 0.045));
        }
    }
    arr.markers.push_back(m);
}

void MarkerPublisher::addBodyMarker(visualization_msgs::MarkerArray& arr,
                                    const VehicleAgent& v) const {
    // 路径点是后轴参考；车身方块画在车身几何中心。
    const RoughWp p = bodyCenterPose(displayPose(v), mp_);
    const double c = std::cos(p.theta);
    const double s = std::sin(p.theta);
    const double L = mp_.vehicle_length;
    const double W = mp_.vehicle_width;

    // RViz 只改变外观，不改变规划/碰撞 footprint：
    // 外包络仍是 L×W；后半段是车身，前半段分成两根叉臂，便于区分车头。
    const double body_len = L * 0.62;
    const double fork_len = L - body_len;
    const double fork_w = W * 0.22;
    const double body_x = -0.5 * L + 0.5 * body_len;
    const double fork_x = 0.5 * L - 0.5 * fork_len;
    const double fork_y = 0.5 * W - 0.5 * fork_w;

    auto addPart = [&](int id, double local_x, double local_y,
                       double sx, double sy) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "multi_patrol_body";
        m.id = id;
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = p.x + c * local_x - s * local_y;
        m.pose.position.y = p.y + s * local_x + c * local_y;
        m.pose.position.z = 0.035;
        m.pose.orientation.z = std::sin(p.theta * 0.5);
        m.pose.orientation.w = std::cos(p.theta * 0.5);
        m.scale.x = sx;
        m.scale.y = sy;
        m.scale.z = 0.050;
        m.color = v.color;
        arr.markers.push_back(m);
    };

    addPart(v.id, body_x, 0.0, body_len, W);
    addPart(1000 + v.id * 2, fork_x, fork_y, fork_len, fork_w);
    addPart(1001 + v.id * 2, fork_x, -fork_y, fork_len, fork_w);
}

void MarkerPublisher::addArrowMarker(visualization_msgs::MarkerArray& arr,
                                     const VehicleAgent& v) const {
    const RoughWp p = bodyCenterPose(displayPose(v), mp_);
    visualization_msgs::Marker m;
    m.header.frame_id = pp_.frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = "multi_patrol_arrow";
    m.id = v.id;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.010;
    m.scale.y = 0.022;
    m.scale.z = 0.0;
    m.color = v.color;

    const double half = mp_.vehicle_length * 0.55;
    const double dx = std::cos(p.theta) * half;
    const double dy = std::sin(p.theta) * half;
    m.points.push_back(pt3(p.x - dx * 0.5, p.y - dy * 0.5, 0.070));
    m.points.push_back(pt3(p.x + dx, p.y + dy, 0.070));
    arr.markers.push_back(m);
}

void MarkerPublisher::addLabelMarker(visualization_msgs::MarkerArray& arr,
                                     const VehicleAgent& v) const {
    const RoughWp p = bodyCenterPose(displayPose(v), mp_);
    visualization_msgs::Marker m;
    m.header.frame_id = pp_.frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = "multi_patrol_label";
    m.id = v.id;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = p.x;
    m.pose.position.y = p.y;
    m.pose.position.z = 0.160;
    m.pose.orientation.w = 1.0;
    m.scale.z = 0.070;
    m.color = v.color;
    m.text = "V" + std::to_string(v.id) + " " + actionName(v.action);
    if (v.mode == VehicleMode::DWELL) {
        m.text += " DWELL";
    } else if (v.loaded) {
        m.text += " L";
    } else {
        m.text += " E";
    }
    if (v.mode == VehicleMode::ACTIVE) {
        std::ostringstream ss;
        ss.setf(std::ios::fixed);
        ss.precision(2);
        ss << " v=" << v.current_speed;
        if (!v.reason.empty()) {
            ss << " " << v.reason;
        }
        m.text += ss.str();
    }
    arr.markers.push_back(m);
}

void MarkerPublisher::addConflictMarkers(
    visualization_msgs::MarkerArray& arr,
    const std::vector<ConflictMarker>& conflicts) const {
    if (!cfg_.show_prediction_conflicts) {
        for (int id = 0; id < last_conflict_marker_count_; ++id) {
            visualization_msgs::Marker m;
            m.header.frame_id = pp_.frame_id;
            m.header.stamp = ros::Time::now();
            m.ns = "multi_patrol_conflict";
            m.id = id;
            m.action = visualization_msgs::Marker::DELETE;
            arr.markers.push_back(m);
        }
        last_conflict_marker_count_ = 0;
        return;
    }
    int id = 0;
    for (const ConflictMarker& c : conflicts) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "multi_patrol_conflict";
        m.id = id++;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = c.x;
        m.pose.position.y = c.y;
        m.pose.position.z = 0.095;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.060;
        m.scale.y = 0.060;
        m.scale.z = 0.020;
        m.color = rgba(1.0f, 0.0f, 0.0f, 0.70f);
        arr.markers.push_back(m);
    }
    for (int stale = id; stale < last_conflict_marker_count_; ++stale) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "multi_patrol_conflict";
        m.id = stale;
        m.action = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(m);
    }
    last_conflict_marker_count_ = id;
}

void MarkerPublisher::addOriginAxes(visualization_msgs::MarkerArray& arr) const {
    // 地图原点 (0,0) 与 X/Y 正方向。实车标定时:动捕 world 原点应与此重合、
    // 某车放已知槽位时 /object÷1000 应≈该槽坐标,车头朝 +X(红轴)时 yaw≈0。
    constexpr double L = 0.5;   // 轴长 0.5m(地图 ~2.5×4.5,够看又不挡)
    const double z = 0.075;     // 抬到与车身箭头同高(z=0.07),否则被地图平面盖住看不见
    auto axis = [&](int id, double ex, double ey, const std_msgs::ColorRGBA& col) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "map_origin_axes";
        m.id = id;
        m.type = visualization_msgs::Marker::ARROW;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.022;   // 杆径(加粗,醒目)
        m.scale.y = 0.05;    // 箭头径
        m.scale.z = 0.08;    // 箭头长
        m.color = col;
        m.points.push_back(pt3(0.0, 0.0, z));
        m.points.push_back(pt3(ex * L, ey * L, z));
        arr.markers.push_back(m);
    };
    auto label = [&](int id, double x, double y, const std::string& txt,
                     const std_msgs::ColorRGBA& col) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "map_origin_axes";
        m.id = id;
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = 0.14;
        m.pose.orientation.w = 1.0;
        m.scale.z = 0.10;
        m.color = col;
        m.text = txt;
        arr.markers.push_back(m);
    };
    axis(0, 1.0, 0.0, rgba(0.95f, 0.1f, 0.1f));   // +X 红
    axis(1, 0.0, 1.0, rgba(0.1f, 0.9f, 0.1f));    // +Y 绿
    label(2, L + 0.06, 0.0, "+X", rgba(0.95f, 0.1f, 0.1f));
    label(3, 0.0, L + 0.06, "+Y", rgba(0.1f, 0.9f, 0.1f));
    label(4, -0.06, -0.06, "O(0,0)", rgba(1.0f, 1.0f, 1.0f));
}

void MarkerPublisher::publish(
    const std::vector<VehicleAgent>& vehicles,
    const std::vector<ConflictMarker>& conflicts) const {
    ++publish_seq_;
    visualization_msgs::MarkerArray arr;
    addOriginAxes(arr);  // 地图原点+XY正方向(标定核对用)
    for (const VehicleAgent& v : vehicles) {
        addPathMarker(arr, v);
        if (v.mode == VehicleMode::NEED_TASK || v.track.empty()) continue;
        addBodyMarker(arr, v);
        addArrowMarker(arr, v);
        addLabelMarker(arr, v);
    }
    addConflictMarkers(arr, conflicts);
    pub_.publish(arr);
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
