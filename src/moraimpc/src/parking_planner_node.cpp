// Parking Planner — 콘 → 주차 박스 추론 + Hybrid A* 경로 생성
//
// 입력: /cone_detector/cones (PoseArray, map frame)
//       /localization/ego_status (GPS/IMU ESKF 현재 위치)
// 출력: /parking_planner/goal           (PoseStamped)
//       /parking_planner/path           (nav_msgs/Path)
//       /parking_planner/obstacle_grid  (nav_msgs/OccupancyGrid)
//       /parking_planner/box            (visualization_msgs/Marker, 직사각형)
//
// 콘 → 주차박스: PCA로 주축 → bounding box → 입구(콘 가장 적은 변) → goal pose

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <std_srvs/Empty.h>

#include "moraimpc/hybrid_astar.hpp"

#include <jsoncpp/json/json.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>

using namespace moraimpc;

class ParkingPlanner {
public:
    ParkingPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
        // 파라미터
        pnh.param("min_cones",       min_cones_,       4);
        pnh.param("inflation_r",     inflation_r_,     1.25);
        pnh.param("grid_res",        grid_res_,        0.25);
        pnh.param("grid_size",       grid_size_,       60.0);
        pnh.param("box_pad",         box_pad_,         0.30);
        pnh.param("plan_period",     plan_period_,     1.0);
        pnh.param<std::string>("map_frame", map_frame_, "map");
        pnh.param<std::string>("output_path_file", output_path_file_, "");  // 비어있으면 저장 X
        pnh.param("densify_step",    densify_step_,    0.20);  // path 보간 간격 [m]

        // Hybrid A* config
        HAStarConfig cfg;
        pnh.param("plan_step",       cfg.step_len,        0.7);
        pnh.param("plan_max_iter",   cfg.max_iter,        20000);
        pnh.param("plan_xy_res",     cfg.xy_res,          0.5);
        pnh.param("plan_safety",     cfg.safety,          0.30);
        pnh.param("plan_reverse_pen", cfg.reverse_penalty, 1.5);
        pnh.param("plan_switch_pen",  cfg.switch_penalty,  2.0);
        planner_.reset(new HybridAStar(cfg));

        // Sub/Pub
        sub_cones_ = nh.subscribe("/cone_detector/cones",      1, &ParkingPlanner::conesCb, this);
        sub_ego_   = nh.subscribe("/localization/ego_status",   5, &ParkingPlanner::egoCb,   this);
        // RViz "2D Goal Pose" 클릭 → /move_base_simple/goal publish
        sub_manual_goal_ = nh.subscribe("/move_base_simple/goal", 1,
                                        &ParkingPlanner::manualGoalCb, this);
        pub_goal_  = pnh.advertise<geometry_msgs::PoseStamped>("goal", 1, true);
        pub_path_  = pnh.advertise<nav_msgs::Path>("path", 1, true);
        pub_grid_  = pnh.advertise<nav_msgs::OccupancyGrid>("obstacle_grid", 1, true);
        pub_box_   = pnh.advertise<visualization_msgs::Marker>("box", 1, true);

        timer_     = nh.createTimer(ros::Duration(plan_period_), &ParkingPlanner::tick, this);

        ROS_INFO("[ParkingPlanner] inflation=%.2fm grid=%.0fm/%.2fm step=%.2fm",
                 inflation_r_, grid_size_, grid_res_, cfg.step_len);
    }

private:
    struct Cone { double x, y; };
    struct Box {
        double cx, cy;     // 중심
        double yaw;        // 주축 각도 (radian)
        double L, W;       // 길이/폭
        int    entry_side; // 0=+x변, 1=+y변, 2=-x변, 3=-y변 (주축 기준)
        bool   valid = false;
    };

    void conesCb(const geometry_msgs::PoseArray::ConstPtr& msg) {
        cones_.clear();
        cones_.reserve(msg->poses.size());
        for (const auto& p : msg->poses) {
            cones_.push_back(Cone{p.position.x, p.position.y});
        }
        last_cones_t_ = ros::Time::now();
    }

    void egoCb(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
        ego_x_ = msg->position.x;
        ego_y_ = msg->position.y;
        ego_yaw_ = msg->heading * M_PI / 180.0;
        ego_ready_ = true;
    }

    void manualGoalCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // RViz "2D Goal Pose" 클릭 → 즉시 Hybrid A* 실행 (콘 무시)
        if (!ego_ready_) {
            ROS_WARN("[ParkingPlanner] manual goal 수신했으나 ego pose 미수신");
            return;
        }
        double yaw = std::atan2(2.0 * (msg->pose.orientation.w * msg->pose.orientation.z),
                                 1.0 - 2.0 * msg->pose.orientation.z * msg->pose.orientation.z);
        HState goal{msg->pose.position.x, msg->pose.position.y, yaw};
        HState start{ego_x_, ego_y_, ego_yaw_};

        // Goal publish (시각화)
        publishGoal(goal);

        // 그리드: 콘 있으면 inflation, 없으면 빈 grid
        ObstacleGrid grid;
        buildGrid(cones_, grid);
        publishGrid(grid);

        ROS_INFO("[ParkingPlanner] Manual goal 수신: (%.1f, %.1f, %.1f°). 검색 시작...",
                 goal.x, goal.y, goal.yaw * 180.0 / M_PI);
        ros::WallTime t0 = ros::WallTime::now();
        std::vector<PathPoint> path;
        bool ok = planner_->plan(start, goal, grid, path);
        double ms = (ros::WallTime::now() - t0).toSec() * 1000.0;

        if (!ok) {
            ROS_WARN("[ParkingPlanner] Manual: HybridA* 실패 iter=%d (%.0fms)",
                     planner_->last_iter(), ms);
            return;
        }
        int gswitches = 0;
        for (size_t i = 1; i < path.size(); ++i) {
            if (path[i].gear != path[i-1].gear) gswitches++;
        }
        ROS_INFO("[ParkingPlanner] Manual: HybridA* OK %zu pts, %d D/R switches, iter=%d (%.0fms)",
                 path.size(), gswitches, planner_->last_iter(), ms);
        publishPath(path);
        exportMixedJson(path, output_path_file_);
    }

    Box estimateBox(const std::vector<Cone>& cs) {
        Box box;
        if ((int)cs.size() < min_cones_) return box;

        // 1) 중심 + PCA 주축
        Eigen::Vector2d mu(0, 0);
        for (const auto& c : cs) { mu(0) += c.x; mu(1) += c.y; }
        mu /= cs.size();

        Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
        for (const auto& c : cs) {
            Eigen::Vector2d d(c.x - mu(0), c.y - mu(1));
            C += d * d.transpose();
        }
        C /= cs.size();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C);
        // 큰 고유값에 해당하는 벡터 = 주축
        Eigen::Vector2d ax_major = es.eigenvectors().col(1);
        Eigen::Vector2d ax_minor = es.eigenvectors().col(0);
        double yaw = std::atan2(ax_major(1), ax_major(0));

        // 2) 각 콘을 (u=주축, v=부축) 좌표로 투영
        double umin = 1e9, umax = -1e9, vmin = 1e9, vmax = -1e9;
        std::vector<double> us, vs;
        us.reserve(cs.size()); vs.reserve(cs.size());
        for (const auto& c : cs) {
            double dx = c.x - mu(0), dy = c.y - mu(1);
            double u = ax_major(0) * dx + ax_major(1) * dy;
            double v = ax_minor(0) * dx + ax_minor(1) * dy;
            us.push_back(u); vs.push_back(v);
            umin = std::min(umin, u); umax = std::max(umax, u);
            vmin = std::min(vmin, v); vmax = std::max(vmax, v);
        }

        double L = (umax - umin) + 2 * box_pad_;
        double W = (vmax - vmin) + 2 * box_pad_;
        // 박스 중심: (u_avg, v_avg) 좌표를 다시 world로
        double u_c = 0.5 * (umin + umax);
        double v_c = 0.5 * (vmin + vmax);
        double cx = mu(0) + ax_major(0) * u_c + ax_minor(0) * v_c;
        double cy = mu(1) + ax_major(1) * u_c + ax_minor(1) * v_c;

        // 3) 입구 변 검출 — 4 변 각각에 가까운 콘 수
        // 변 거리 임계: 0.5m
        double thresh = 0.5;
        int cnt[4] = {0, 0, 0, 0};
        for (size_t i = 0; i < cs.size(); ++i) {
            double u = us[i] - u_c, v = vs[i] - v_c;
            // 각 변에 대한 부호거리 (박스 내부 기준 -, 변 위 ≈ 0)
            double du_p = (L * 0.5) - u;     // +x 변 (u=+L/2)
            double du_m = u - (-L * 0.5);    // -x 변
            double dv_p = (W * 0.5) - v;
            double dv_m = v - (-W * 0.5);
            if (std::abs(du_p) < thresh) cnt[0]++;
            if (std::abs(dv_p) < thresh) cnt[1]++;
            if (std::abs(du_m) < thresh) cnt[2]++;
            if (std::abs(dv_m) < thresh) cnt[3]++;
        }
        int entry = 0;
        for (int k = 1; k < 4; ++k) {
            if (cnt[k] < cnt[entry]) entry = k;
        }

        box.cx = cx; box.cy = cy; box.yaw = yaw;
        box.L = L; box.W = W;
        box.entry_side = entry;
        box.valid = true;
        return box;
    }

    // 박스 + 입구 정보 → goal pose
    // 후진 주차: 차량 전방이 입구를 향하게 (즉 yaw = 입구 방향) — 차가 후진하면서 박스 안으로
    HState goalFromBox(const Box& b) {
        // 입구 방향 단위벡터 (박스 중심에서 입구 변 방향)
        double cs = std::cos(b.yaw), sn = std::sin(b.yaw);
        double ex_u = 0, ex_v = 0;
        switch (b.entry_side) {
            case 0: ex_u = +1.0; break;  // +u 변
            case 1: ex_v = +1.0; break;  // +v 변
            case 2: ex_u = -1.0; break;
            case 3: ex_v = -1.0; break;
        }
        // 입구 방향 벡터를 world로 회전
        double dx = cs * ex_u + (-sn) * ex_v;
        double dy = sn * ex_u +  cs   * ex_v;
        double yaw = std::atan2(dy, dx);  // 차량 전방 = 입구 방향 (후진 주차)
        HState g;
        g.x = b.cx;   // 박스 중심
        g.y = b.cy;
        g.yaw = yaw;
        return g;
    }

    void buildGrid(const std::vector<Cone>& cs, ObstacleGrid& g) {
        // 차량 중심 grid_size_ × grid_size_, 해상도 grid_res_
        int W = static_cast<int>(grid_size_ / grid_res_);
        int H = W;
        double ox = ego_x_ - 0.5 * grid_size_;
        double oy = ego_y_ - 0.5 * grid_size_;
        g = ObstacleGrid(ox, oy, grid_res_, W, H);
        for (const auto& c : cs) {
            g.inflateCircle(c.x, c.y, inflation_r_);
        }
    }

    void publishGrid(const ObstacleGrid& g) {
        nav_msgs::OccupancyGrid msg;
        msg.header.frame_id = map_frame_;
        msg.header.stamp = ros::Time::now();
        msg.info.resolution = g.res();
        msg.info.width  = g.width();
        msg.info.height = g.height();
        msg.info.origin.position.x = g.originX();
        msg.info.origin.position.y = g.originY();
        msg.info.origin.orientation.w = 1.0;
        msg.data.resize(g.width() * g.height());
        const auto& d = g.data();
        for (size_t i = 0; i < d.size(); ++i) msg.data[i] = d[i] ? 100 : 0;
        pub_grid_.publish(msg);
    }

    void publishGoal(const HState& g) {
        geometry_msgs::PoseStamped p;
        p.header.frame_id = map_frame_;
        p.header.stamp = ros::Time::now();
        p.pose.position.x = g.x;
        p.pose.position.y = g.y;
        p.pose.position.z = 0.05;
        p.pose.orientation.z = std::sin(g.yaw * 0.5);
        p.pose.orientation.w = std::cos(g.yaw * 0.5);
        pub_goal_.publish(p);
    }

    void publishBox(const Box& b) {
        visualization_msgs::Marker m;
        m.header.frame_id = map_frame_;
        m.header.stamp = ros::Time::now();
        m.ns = "parking_box"; m.id = 0;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.10;  // line width
        m.color.r = 0.2f; m.color.g = 1.0f; m.color.b = 0.4f; m.color.a = 0.95f;
        // 4 corners (주축 yaw 회전)
        double cs = std::cos(b.yaw), sn = std::sin(b.yaw);
        double hL = b.L * 0.5, hW = b.W * 0.5;
        double corners_u[5] = {+hL, +hL, -hL, -hL, +hL};
        double corners_v[5] = {+hW, -hW, -hW, +hW, +hW};
        for (int i = 0; i < 5; ++i) {
            geometry_msgs::Point pt;
            pt.x = b.cx + cs * corners_u[i] - sn * corners_v[i];
            pt.y = b.cy + sn * corners_u[i] + cs * corners_v[i];
            pt.z = 0.05;
            m.points.push_back(pt);
        }
        pub_box_.publish(m);
    }

    // Hybrid A* 경로 → mixed.json 형식 (path_follower 호환)
    // - 0.20m 간격으로 densify (linear interp)
    // - gear: +1 → "D", -1 → "R"
    // - heading: yaw [rad]
    void exportMixedJson(const std::vector<PathPoint>& path, const std::string& fpath) {
        if (path.size() < 2 || fpath.empty()) return;

        Json::Value root;
        Json::Value wps(Json::arrayValue);

        auto wrapPi = [](double a){ while (a>M_PI) a -= 2*M_PI; while (a<-M_PI) a += 2*M_PI; return a; };

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            const PathPoint& a = path[i];
            const PathPoint& b = path[i+1];
            double dx = b.x - a.x, dy = b.y - a.y;
            double seg = std::hypot(dx, dy);
            if (seg < 1e-6) continue;
            int n = std::max(1, (int)std::round(seg / densify_step_));
            // gear는 b.gear (이 segment 진행 gear)
            const char* gear_str = (b.gear < 0) ? "R" : "D";
            for (int k = 0; k < n; ++k) {
                double t = static_cast<double>(k) / n;
                double x = a.x + t * dx;
                double y = a.y + t * dy;
                // yaw: a→b 슬러프 (wrap)
                double dyaw = wrapPi(b.yaw - a.yaw);
                double yaw = a.yaw + t * dyaw;
                Json::Value w;
                w["x"] = x; w["y"] = y;
                w["heading"] = yaw;
                w["gear"] = gear_str;
                wps.append(w);
            }
        }
        // 마지막 점 추가
        const PathPoint& last = path.back();
        Json::Value w_last;
        w_last["x"] = last.x; w_last["y"] = last.y;
        w_last["heading"] = last.yaw;
        w_last["gear"] = (last.gear < 0) ? "R" : "D";
        wps.append(w_last);

        root["waypoints"] = wps;

        std::ofstream ofs(fpath);
        if (!ofs) {
            ROS_WARN("[ParkingPlanner] mixed.json 저장 실패: %s", fpath.c_str());
            return;
        }
        Json::StreamWriterBuilder b;
        b["indentation"] = "  ";
        std::unique_ptr<Json::StreamWriter> writer(b.newStreamWriter());
        writer->write(root, &ofs);
        ROS_INFO("[ParkingPlanner] 경로 저장: %s (%d waypoints)", fpath.c_str(), (int)wps.size());
    }

    void publishPath(const std::vector<PathPoint>& path) {
        nav_msgs::Path msg;
        msg.header.frame_id = map_frame_;
        msg.header.stamp = ros::Time::now();
        msg.poses.reserve(path.size());
        for (const auto& p : path) {
            geometry_msgs::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            ps.pose.position.z = 0.05;
            ps.pose.orientation.z = std::sin(p.yaw * 0.5);
            ps.pose.orientation.w = std::cos(p.yaw * 0.5);
            msg.poses.push_back(ps);
        }
        pub_path_.publish(msg);
    }

    void tick(const ros::TimerEvent&) {
        // 매 tick 상태 로그 (1Hz throttle)
        ROS_INFO_THROTTLE(1.0,
            "[ParkingPlanner] tick: ego=%d cones=%zu min_required=%d",
            (int)ego_ready_, cones_.size(), min_cones_);
        if (!ego_ready_ || cones_.empty()) return;

        // 1) 박스 추정 (raw)
        Box box_new = estimateBox(cones_);
        if (!box_new.valid) {
            ROS_WARN_THROTTLE(2.0, "[ParkingPlanner] cones=%zu (min %d 필요)",
                              cones_.size(), min_cones_);
            return;
        }
        // EMA 안정화 (콘 누적 + 박스 평활화)
        if (!box_smoothed_.valid) {
            box_smoothed_ = box_new;
        } else {
            const double a = 0.3;  // 0=완전 smooth, 1=raw
            box_smoothed_.cx  = (1 - a) * box_smoothed_.cx + a * box_new.cx;
            box_smoothed_.cy  = (1 - a) * box_smoothed_.cy + a * box_new.cy;
            box_smoothed_.L   = (1 - a) * box_smoothed_.L  + a * box_new.L;
            box_smoothed_.W   = (1 - a) * box_smoothed_.W  + a * box_new.W;
            // yaw wrap
            double dy = box_new.yaw - box_smoothed_.yaw;
            while (dy >  M_PI/2) dy -= M_PI;       // 박스 yaw는 ±90° 모호 → π주기
            while (dy < -M_PI/2) dy += M_PI;
            box_smoothed_.yaw += a * dy;
            // entry는 다수결 (최신값 사용)
            box_smoothed_.entry_side = box_new.entry_side;
        }
        Box box = box_smoothed_;
        publishBox(box);

        // 2) Goal pose
        HState goal = goalFromBox(box);
        publishGoal(goal);

        // 3) Obstacle grid
        ObstacleGrid grid;
        buildGrid(cones_, grid);
        publishGrid(grid);

        // 4) Hybrid A* plan
        HState start{ego_x_, ego_y_, ego_yaw_};
        std::vector<PathPoint> path;
        ros::WallTime t0 = ros::WallTime::now();
        bool ok = planner_->plan(start, goal, grid, path);
        double ms = (ros::WallTime::now() - t0).toSec() * 1000.0;

        if (!ok) {
            ROS_WARN("[ParkingPlanner] HybridA* 실패 iter=%d (%.0fms) — start=(%.1f,%.1f,%.1f°) goal=(%.1f,%.1f,%.1f°)",
                     planner_->last_iter(), ms,
                     start.x, start.y, start.yaw * 180.0 / M_PI,
                     goal.x, goal.y, goal.yaw * 180.0 / M_PI);
            return;
        }

        // gear 전환 통계
        int gswitches = 0;
        for (size_t i = 1; i < path.size(); ++i) {
            if (path[i].gear != path[i-1].gear) gswitches++;
        }
        ROS_INFO("[ParkingPlanner] HybridA* OK: %zu pts, %d D/R switches, iter=%d (%.0fms)",
                 path.size(), gswitches, planner_->last_iter(), ms);

        publishPath(path);
        exportMixedJson(path, output_path_file_);
    }

    // 파라미터
    int    min_cones_;
    double inflation_r_;
    double grid_res_, grid_size_;
    double box_pad_;
    double plan_period_;
    double densify_step_;
    std::string map_frame_;
    std::string output_path_file_;

    // 상태
    std::vector<Cone> cones_;
    ros::Time last_cones_t_;
    double ego_x_ = 0, ego_y_ = 0, ego_yaw_ = 0;
    bool   ego_ready_ = false;

    std::unique_ptr<HybridAStar> planner_;
    Box box_smoothed_;

    ros::Subscriber sub_cones_, sub_ego_, sub_manual_goal_;
    ros::Publisher  pub_goal_, pub_path_, pub_grid_, pub_box_;
    ros::Timer      timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "parking_planner_node");
    ros::NodeHandle nh, pnh("~");
    ParkingPlanner pl(nh, pnh);
    ros::spin();
    return 0;
}
