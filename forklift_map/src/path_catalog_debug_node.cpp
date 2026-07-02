#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/Point.h"
#include "forklift_map/forklift_map.h"
#include "forklift_map/map_param.h"
#include "forklift_map/map_types.h"
#include "forklift_planner/path_generator.h"
#include "forklift_planner/planner_param.h"
#include "std_msgs/ColorRGBA.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

std_msgs::ColorRGBA rgba(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

geometry_msgs::Point point(double x, double y, double z = 0.0) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

visualization_msgs::Marker baseMarker(const std::string& frame,
                                      const std::string& ns,
                                      int id) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = ros::Time(0);
    m.ns = ns;
    m.id = id;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    return m;
}

void addText(visualization_msgs::MarkerArray& arr,
             const std::string& frame,
             const std::string& ns,
             int id,
             double x,
             double y,
             double z,
             double height,
             const std::string& text,
             const std_msgs::ColorRGBA& color) {
    auto m = baseMarker(frame, ns, id);
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.scale.z = height;
    m.text = text;
    m.color = color;
    arr.markers.push_back(m);
}

void addArrow(visualization_msgs::MarkerArray& arr,
              const std::string& frame,
              const std::string& ns,
              int id,
              double x,
              double y,
              double theta,
              const std_msgs::ColorRGBA& color) {
    auto m = baseMarker(frame, ns, id);
    m.type = visualization_msgs::Marker::ARROW;
    m.points.push_back(point(x, y, 0.09));
    m.points.push_back(point(x + 0.22 * std::cos(theta),
                             y + 0.22 * std::sin(theta), 0.09));
    m.scale.x = 0.018;
    m.scale.y = 0.045;
    m.scale.z = 0.055;
    m.color = color;
    arr.markers.push_back(m);
}

void addSphere(visualization_msgs::MarkerArray& arr,
               const std::string& frame,
               const std::string& ns,
               int id,
               double x,
               double y,
               const std_msgs::ColorRGBA& color) {
    auto m = baseMarker(frame, ns, id);
    m.type = visualization_msgs::Marker::SPHERE;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.08;
    m.scale.x = 0.09;
    m.scale.y = 0.09;
    m.scale.z = 0.05;
    m.color = color;
    arr.markers.push_back(m);
}

void addPath(visualization_msgs::MarkerArray& arr,
             const std::string& frame,
             const std::string& ns,
             int id,
             const RoughPath& path,
             const std_msgs::ColorRGBA& color,
             double z_offset) {
    auto m = baseMarker(frame, ns, id);
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.scale.x = 0.010;
    m.color = color;
    m.points.reserve(path.size());
    for (const RoughWp& wp : path) {
        m.points.push_back(point(wp.x, wp.y, z_offset));
    }
    arr.markers.push_back(m);
}

double pathLength(const RoughPath& path) {
    double len = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        len += std::hypot(path[i].x - path[i - 1].x,
                          path[i].y - path[i - 1].y);
    }
    return len;
}

std::string idsToString(const std::vector<int>& ids) {
    std::ostringstream out;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (i > 0) out << ", ";
        out << ids[i];
    }
    return out.str();
}

Slot makeVirtualStart(const std::string& name,
                      int id,
                      int row_id,
                      double x,
                      double y,
                      double theta) {
    Slot s;
    s.id = id;
    s.row_id = row_id;
    s.col = -1;
    s.cx = x;
    s.cy = y;
    s.pre_dock_x = x;
    s.pre_dock_y = y;
    s.dock_theta = theta;
    s.occupied = false;
    ROS_INFO("[path_catalog] %s virtual start: id=%d row=%d x=%.4f y=%.4f yaw=%.1fdeg",
             name.c_str(), id, row_id, x, y, theta * 180.0 / kPi);
    return s;
}

double midpoint(double a, double b) {
    return 0.5 * (a + b);
}

double normAngle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}

double angleLerp(double a, double b, double u) {
    return normAngle(a + normAngle(b - a) * u);
}

RoughWp poseAtS(const RoughPath& path, double query_s) {
    if (path.empty()) return {0.0, 0.0, 0.0, WpType::FORWARD};
    if (path.size() == 1 || query_s <= 0.0) return path.front();

    double acc = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        const RoughWp& a = path[i - 1];
        const RoughWp& b = path[i];
        const double seg = std::hypot(b.x - a.x, b.y - a.y);
        if (seg <= 1e-9) continue;
        if (acc + seg >= query_s) {
            const double u = std::max(0.0, std::min(1.0, (query_s - acc) / seg));
            return {a.x + (b.x - a.x) * u,
                    a.y + (b.y - a.y) * u,
                    angleLerp(a.theta, b.theta, u),
                    u < 0.5 ? a.type : b.type};
        }
        acc += seg;
    }
    return path.back();
}

std::string uppercase(std::string s) {
    for (char& ch : s) {
        ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }
    return s;
}

}  // namespace

class PathCatalogDebugNode {
public:
    PathCatalogDebugNode() : nh_("~") {
        ros::NodeHandle param_nh;
        mp_ = MapParam::fromROSParam(param_nh);
        pp_ = PlannerParam::fromROSParam(param_nh);
        map_ = std::make_unique<ForkliftMap>(mp_);
        generator_ = std::make_unique<PathGenerator>(mp_, pp_);

        use_exact_midpoints_ = true;
        nh_.param("use_exact_midpoints", use_exact_midpoints_,
                  use_exact_midpoints_);
        nh_.param("a1_x", a1_x_, 1.25);
        nh_.param("a1_y", a1_y_, 4.38);
        nh_.param("a1_yaw", a1_yaw_, -kPi * 0.5);
        nh_.param("a2_x", a2_x_, 1.25);
        nh_.param("a2_y", a2_y_, 0.12);
        nh_.param("a2_yaw", a2_yaw_, kPi * 0.5);
        nh_.param("depot", depot_name_, depot_name_);
        nh_.param("target_slot", target_slot_, target_slot_);
        nh_.param("animate", animate_, animate_);
        nh_.param("animation_period", animation_period_, animation_period_);
        nh_.param("animation_duration", animation_duration_, animation_duration_);

        if (use_exact_midpoints_ && map_->slots().size() > 61) {
            const Slot& b4 = map_->slots().at(4);
            const Slot& b5 = map_->slots().at(5);
            const Slot& b60 = map_->slots().at(60);
            const Slot& b61 = map_->slots().at(61);
            a1_x_ = midpoint(b4.cx, b5.cx);
            a1_y_ = midpoint(b4.cy, b5.cy);
            a2_x_ = midpoint(b60.cx, b61.cx);
            a2_y_ = midpoint(b60.cy, b61.cy);
            ROS_INFO("[path_catalog] midpoint check: B4(%.4f,%.4f), B5(%.4f,%.4f) -> A1(%.4f,%.4f)",
                     b4.cx, b4.cy, b5.cx, b5.cy, a1_x_, a1_y_);
            ROS_INFO("[path_catalog] midpoint check: B60(%.4f,%.4f), B61(%.4f,%.4f) -> A2(%.4f,%.4f)",
                     b60.cx, b60.cy, b61.cx, b61.cy, a2_x_, a2_y_);
        }

        pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_map/markers", 1, true);

        publish();
        if (!selected_path_.empty()) {
            animation_start_ = ros::Time::now();
            publishVehicleMarker(poseAtS(selected_path_, 0.0));
            if (animate_) {
                timer_ = nh_.createTimer(ros::Duration(animation_period_),
                                         &PathCatalogDebugNode::onAnimationTimer,
                                         this);
            }
        }
    }

private:
    void publish() {
        const Slot a1 = makeVirtualStart("A1", -101, 0, a1_x_, a1_y_, a1_yaw_);
        const Slot a2 = makeVirtualStart("A2", -102, 7, a2_x_, a2_y_, a2_yaw_);

        std::vector<int> a1_targets;
        std::vector<int> a2_targets;
        const double split_y = mp_.field_height * 0.5;
        for (const Slot& s : map_->slots()) {
            if (s.cy >= split_y) {
                a1_targets.push_back(s.id);
            } else {
                a2_targets.push_back(s.id);
            }
        }

        ROS_WARN("[path_catalog] A1 targets (%zu): [%s]",
                 a1_targets.size(), idsToString(a1_targets).c_str());
        ROS_WARN("[path_catalog] A2 targets (%zu): [%s]",
                 a2_targets.size(), idsToString(a2_targets).c_str());

        if (target_slot_ >= 0) {
            publishSinglePath(a1, a2, a1_targets, a2_targets);
            return;
        }

        visualization_msgs::MarkerArray arr;
        int id = 0;
        addDepotMarkers(arr, id, a1, "A1", rgba(0.1f, 0.65f, 1.0f, 1.0f));
        addDepotMarkers(arr, id, a2, "A2", rgba(1.0f, 0.55f, 0.05f, 1.0f));
        addTargetMarkers(arr, id, a1_targets, rgba(0.2f, 0.8f, 1.0f, 0.9f),
                         "A1_target");
        addTargetMarkers(arr, id, a2_targets, rgba(1.0f, 0.75f, 0.2f, 0.9f),
                         "A2_target");

        buildAndDrawPaths(arr, id, a1, a1_targets, "A1",
                          rgba(0.15f, 0.75f, 1.0f, 0.42f), 0.055);
        buildAndDrawPaths(arr, id, a2, a2_targets, "A2",
                          rgba(1.0f, 0.55f, 0.05f, 0.42f), 0.065);

        static_markers_ = arr;
        pub_.publish(arr);
        ROS_WARN("[path_catalog] published %zu markers on /forklift_map/markers",
                 arr.markers.size());
    }

    void publishSinglePath(const Slot& a1,
                           const Slot& a2,
                           const std::vector<int>& a1_targets,
                           const std::vector<int>& a2_targets) {
        if (target_slot_ < 0 ||
            target_slot_ >= static_cast<int>(map_->slots().size())) {
            ROS_ERROR("[path_catalog] target_slot=%d is out of range [0,%zu)",
                      target_slot_, map_->slots().size());
            return;
        }

        const std::string depot = uppercase(depot_name_);
        const bool use_a2 = depot == "A2";
        const Slot& src = use_a2 ? a2 : a1;
        const std::string label = use_a2 ? "A2" : "A1";
        const auto& allowed = use_a2 ? a2_targets : a1_targets;
        if (std::find(allowed.begin(), allowed.end(), target_slot_) ==
            allowed.end()) {
            ROS_WARN("[path_catalog] %s -> B%d is outside the current half-map target set; generating anyway for inspection",
                     label.c_str(), target_slot_);
        }

        const Slot& dst = map_->slots().at(static_cast<size_t>(target_slot_));
        PathGenerationInfo info;
        selected_path_ = generator_->generate(src, dst, &info);
        selected_label_ = label + "_to_B" + std::to_string(target_slot_);
        if (selected_path_.size() < 2) {
            ROS_ERROR("[path_catalog] %s failed: empty path", selected_label_.c_str());
            selected_path_.clear();
            return;
        }

        const std_msgs::ColorRGBA color =
            use_a2 ? rgba(1.0f, 0.55f, 0.05f, 1.0f)
                   : rgba(0.1f, 0.65f, 1.0f, 1.0f);
        const double len = pathLength(selected_path_);
        ROS_WARN("[path_catalog] single path %s row=%d col=%d wpts=%zu len=%.3f arc=%d animate=%d",
                 selected_label_.c_str(), dst.row_id, dst.col,
                 selected_path_.size(), len, info.used_arc_fallback ? 1 : 0,
                 animate_ ? 1 : 0);

        visualization_msgs::MarkerArray arr;
        int id = 0;
        addDepotMarkers(arr, id, src, label, color);
        addSphere(arr, pp_.frame_id, "single_target_point", id++, dst.cx, dst.cy,
                  rgba(1.0f, 1.0f, 1.0f, 1.0f));
        addText(arr, pp_.frame_id, "single_target_label", id++, dst.cx, dst.cy,
                0.19, 0.075, "B" + std::to_string(target_slot_),
                rgba(1.0f, 1.0f, 1.0f, 1.0f));
        addPath(arr, pp_.frame_id, "single_path", id++, selected_path_, color,
                0.075);
        addText(arr, pp_.frame_id, "single_path_label", id++,
                midpoint(src.cx, dst.cx), midpoint(src.cy, dst.cy), 0.22,
                0.07, selected_label_, color);
        static_markers_ = arr;
        pub_.publish(arr);
        ROS_WARN("[path_catalog] published single path markers on /forklift_map/markers");
    }

    void addDepotMarkers(visualization_msgs::MarkerArray& arr,
                         int& id,
                         const Slot& depot,
                         const std::string& label,
                         const std_msgs::ColorRGBA& color) const {
        addSphere(arr, pp_.frame_id, "virtual_depot", id++, depot.cx, depot.cy,
                  color);
        addArrow(arr, pp_.frame_id, "virtual_depot_heading", id++, depot.cx,
                 depot.cy, depot.dock_theta, color);
        addText(arr, pp_.frame_id, "virtual_depot_label", id++, depot.cx,
                depot.cy, 0.18, 0.10, label, color);
    }

    void addTargetMarkers(visualization_msgs::MarkerArray& arr,
                          int& id,
                          const std::vector<int>& targets,
                          const std_msgs::ColorRGBA& color,
                          const std::string& ns) const {
        for (int target : targets) {
            const Slot& s = map_->slots().at(static_cast<size_t>(target));
            addText(arr, pp_.frame_id, ns, id++, s.cx, s.cy, 0.16, 0.055,
                    "B" + std::to_string(target), color);
        }
    }

    void buildAndDrawPaths(visualization_msgs::MarkerArray& arr,
                           int& id,
                           const Slot& depot,
                           const std::vector<int>& targets,
                           const std::string& depot_label,
                           const std_msgs::ColorRGBA& color,
                           double z_offset) const {
        int ok = 0;
        int failed = 0;
        for (int target : targets) {
            PathGenerationInfo info;
            const Slot& dst = map_->slots().at(static_cast<size_t>(target));
            RoughPath path = generator_->generate(depot, dst, &info);
            if (path.size() < 2) {
                ++failed;
                ROS_ERROR("[path_catalog] %s -> B%d failed: empty path",
                          depot_label.c_str(), target);
                continue;
            }
            ++ok;
            addPath(arr, pp_.frame_id, depot_label + "_to_B", id++, path, color,
                    z_offset);
            ROS_INFO("[path_catalog] %s -> B%d row=%d col=%d wpts=%zu len=%.3f arc=%d",
                     depot_label.c_str(), target, dst.row_id, dst.col,
                     path.size(), pathLength(path),
                     info.used_arc_fallback ? 1 : 0);
        }
        ROS_WARN("[path_catalog] %s generated %d/%zu paths, failed=%d",
                 depot_label.c_str(), ok, targets.size(), failed);
    }

    void onAnimationTimer(const ros::TimerEvent&) {
        if (selected_path_.empty()) return;
        const double len = pathLength(selected_path_);
        if (len <= 1e-9) return;
        const double elapsed =
            std::max(0.0, (ros::Time::now() - animation_start_).toSec());
        const double period = std::max(0.5, animation_duration_);
        const double phase = std::fmod(elapsed, period) / period;
        publishVehicleMarker(poseAtS(selected_path_, len * phase));
    }

    void publishVehicleMarker(const RoughWp& ref) {
        visualization_msgs::MarkerArray arr = static_markers_;

        RoughWp center = ref;
        center.x += mp_.rear_axle_to_center * std::cos(ref.theta);
        center.y += mp_.rear_axle_to_center * std::sin(ref.theta);

        auto body = baseMarker(pp_.frame_id, "single_vehicle_body", 0);
        body.type = visualization_msgs::Marker::CUBE;
        body.pose.position.x = center.x;
        body.pose.position.y = center.y;
        body.pose.position.z = 0.13;
        body.pose.orientation.z = std::sin(center.theta * 0.5);
        body.pose.orientation.w = std::cos(center.theta * 0.5);
        body.scale.x = mp_.vehicle_length;
        body.scale.y = mp_.vehicle_width;
        body.scale.z = 0.045;
        body.color = ref.type == WpType::REVERSE
            ? rgba(1.0f, 0.25f, 0.25f, 0.85f)
            : rgba(0.2f, 1.0f, 0.35f, 0.85f);
        arr.markers.push_back(body);

        auto arrow = baseMarker(pp_.frame_id, "single_vehicle_heading", 1);
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.points.push_back(point(center.x, center.y, 0.18));
        arrow.points.push_back(point(center.x + 0.18 * std::cos(center.theta),
                                     center.y + 0.18 * std::sin(center.theta), 0.18));
        arrow.scale.x = 0.018;
        arrow.scale.y = 0.045;
        arrow.scale.z = 0.055;
        arrow.color = rgba(1.0f, 1.0f, 1.0f, 1.0f);
        arr.markers.push_back(arrow);

        pub_.publish(arr);
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Timer timer_;
    visualization_msgs::MarkerArray static_markers_;
    MapParam mp_;
    PlannerParam pp_;
    std::unique_ptr<ForkliftMap> map_;
    std::unique_ptr<PathGenerator> generator_;

    bool use_exact_midpoints_ = true;
    double a1_x_ = 1.25;
    double a1_y_ = 4.38;
    double a1_yaw_ = -kPi * 0.5;
    double a2_x_ = 1.25;
    double a2_y_ = 0.12;
    double a2_yaw_ = kPi * 0.5;

    std::string depot_name_ = "A1";
    int target_slot_ = -1;
    bool animate_ = true;
    double animation_period_ = 0.05;
    double animation_duration_ = 8.0;
    ros::Time animation_start_;
    RoughPath selected_path_;
    std::string selected_label_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_catalog_debug_node");
    PathCatalogDebugNode node;
    ros::spin();
    return 0;
}
