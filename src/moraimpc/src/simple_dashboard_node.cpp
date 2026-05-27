// simple_dashboard_node.cpp — 최소 OpenCV 대시보드
//   path(mixed) + path_avoid + 차량 trail + NPC 표시 + cte/hdg/v 텍스트
//   focus: 단순·확실 GUI 표시 (mpc_dashboard_node 안 뜨는 문제 우회)
//
// 토픽: /Ego_topic, /Object_topic, /mpc_performance
// 파라미터: ~path_file, ~avoid_path_file, ~window_size (default 700px)

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/ObjectStatusList.h>
#include <opencv2/opencv.hpp>
#include <jsoncpp/json/json.h>
#include <atomic>
#include <fstream>
#include <mutex>
#include <vector>
#include <deque>
#include <cmath>

struct Pt { double x, y; double kappa = 0.0; };

class SimpleDashboard {
public:
    SimpleDashboard(ros::NodeHandle& nh) {
        std::string path_file, avoid_file;
        nh.param<std::string>("path_file", path_file, "");
        nh.param<std::string>("avoid_path_file", avoid_file, "");
        nh.param<int>("window_size", win_size_, 700);
        nh.param<double>("view_margin_m", view_margin_, 30.0);

        loadPath(path_file, path_);
        loadPath(avoid_file, avoid_);

        ego_sub_ = nh.subscribe("/Ego_topic", 1, &SimpleDashboard::egoCb, this);
        obj_sub_ = nh.subscribe("/Object_topic", 1, &SimpleDashboard::objCb, this);
        perf_sub_ = nh.subscribe("/mpc_performance", 1, &SimpleDashboard::perfCb, this);

        ROS_INFO("[simple_dashboard] path=%zu avoid=%zu win=%dpx",
                 path_.size(), avoid_.size(), win_size_);
        cv::namedWindow(kWin_, cv::WINDOW_AUTOSIZE);
        cv::moveWindow(kWin_, 50, 50);
    }

    void spin() {
        ros::Rate rate(20);
        while (ros::ok()) {
            ros::spinOnce();
            render();
            cv::waitKey(1);
            rate.sleep();
        }
    }

private:
    void loadPath(const std::string& f, std::vector<Pt>& out) {
        if (f.empty()) return;
        std::ifstream in(f);
        if (!in.is_open()) { ROS_WARN("[simple_dashboard] cannot open %s", f.c_str()); return; }
        Json::Value root; in >> root;
        const auto& wps = root.isMember("waypoints") ? root["waypoints"] : root;
        for (const auto& w : wps) out.push_back({w["x"].asDouble(), w["y"].asDouble(), 0.0});
        // 곡률 계산 (3점 중심차분)
        for (size_t i = 1; i+1 < out.size(); ++i) {
            double h1 = std::atan2(out[i].y - out[i-1].y, out[i].x - out[i-1].x);
            double h2 = std::atan2(out[i+1].y - out[i].y, out[i+1].x - out[i].x);
            double dh = std::atan2(std::sin(h2-h1), std::cos(h2-h1));
            double ds = std::hypot(out[i+1].x - out[i-1].x, out[i+1].y - out[i-1].y);
            out[i].kappa = (ds > 1e-6) ? dh/ds : 0.0;
        }
    }
    // 곡률 → 색상 매핑 (BGR). R=∞(직선) → 청록, R=20m → 빨강
    cv::Scalar kappaColor(double kappa) {
        double R = (std::abs(kappa) > 1e-5) ? 1.0/std::abs(kappa) : 1e4;
        // R 100m+ 청록, R 50m 노랑, R 20m 빨강
        double t = std::clamp((100.0 - R) / 80.0, 0.0, 1.0);  // 0 = 직선, 1 = sharp
        int r = (int)(255 * t);
        int g = (int)(255 * (1.0 - t) * 0.7 + 80);
        int b = (int)(220 * (1.0 - t));
        return cv::Scalar(b, g, r);
    }
    void egoCb(const morai_msgs::EgoVehicleStatus::ConstPtr& m) {
        std::lock_guard<std::mutex> lk(m_);
        ego_x_ = m->position.x; ego_y_ = m->position.y; ego_yaw_ = m->heading * M_PI/180.0;
        trail_.push_back({ego_x_, ego_y_});
        if (trail_.size() > 800) trail_.pop_front();
    }
    void objCb(const morai_msgs::ObjectStatusList::ConstPtr& m) {
        std::lock_guard<std::mutex> lk(m_);
        npcs_.clear();
        for (const auto& o : m->npc_list) npcs_.push_back({o.position.x, o.position.y});
    }
    void perfCb(const std_msgs::Float32MultiArray::ConstPtr& m) {
        std::lock_guard<std::mutex> lk(m_);
        if (m->data.size() >= 5) {
            cte_ = m->data[2]; hdg_ = m->data[3]; v_ = m->data[4];
        }
    }
    cv::Point worldToImg(double wx, double wy) {
        double cx = ego_x_, cy = ego_y_;
        double scale = win_size_ / (2.0 * view_margin_);
        int ix = (int)((wx - cx) * scale + win_size_/2.0);
        int iy = (int)(win_size_/2.0 - (wy - cy) * scale);
        return cv::Point(ix, iy);
    }
    void render() {
        std::lock_guard<std::mutex> lk(m_);
        cv::Mat img(win_size_, win_size_, CV_8UC3, cv::Scalar(20, 20, 20));

        // 그리드
        for (int g=-30; g<=30; g+=10) {
            cv::line(img, worldToImg(ego_x_+g, ego_y_-30), worldToImg(ego_x_+g, ego_y_+30), cv::Scalar(40,40,40), 1);
            cv::line(img, worldToImg(ego_x_-30, ego_y_+g), worldToImg(ego_x_+30, ego_y_+g), cv::Scalar(40,40,40), 1);
        }

        // mixed path: 곡률 기반 색상 (직선=청록, sharp=빨강)
        drawPathColored(img, path_, 2);
        // avoid path (green 단색, mixed 위에 덮어그림)
        drawPath(img, avoid_, cv::Scalar(50, 220, 80), 3);
        // trail (gray)
        for (size_t i=1; i<trail_.size(); ++i)
            cv::line(img, worldToImg(trail_[i-1].x, trail_[i-1].y), worldToImg(trail_[i].x, trail_[i].y), cv::Scalar(150,150,150), 1);
        // npcs (red)
        for (const auto& n : npcs_) {
            auto p = worldToImg(n.x, n.y);
            cv::circle(img, p, 8, cv::Scalar(50, 50, 220), -1);
            cv::circle(img, p, 10, cv::Scalar(50, 50, 220), 2);
        }
        // ego (yellow arrow)
        cv::Point ep = worldToImg(ego_x_, ego_y_);
        cv::circle(img, ep, 7, cv::Scalar(50, 220, 220), -1);
        double dx = std::cos(ego_yaw_) * 5.0, dy = std::sin(ego_yaw_) * 5.0;
        cv::Point head = worldToImg(ego_x_ + dx, ego_y_ + dy);
        cv::arrowedLine(img, ep, head, cv::Scalar(80, 255, 255), 2);

        // 텍스트 HUD
        char buf[128];
        snprintf(buf, sizeof(buf), "ego (%.1f, %.1f) yaw=%+5.1f deg", ego_x_, ego_y_, ego_yaw_ * 180.0/M_PI);
        cv::putText(img, buf, {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255,255,255), 1);
        snprintf(buf, sizeof(buf), "cte=%+5.2f m  hdg=%+5.1f deg  v=%5.1f km/h", cte_, hdg_, v_);
        cv::putText(img, buf, {10, 50}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(80,220,220), 1);
        snprintf(buf, sizeof(buf), "NPC=%zu", npcs_.size());
        cv::putText(img, buf, {10, 75}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(80,80,220), 1);

        // legend
        cv::putText(img, "path color: CYAN=straight  YELLOW=R~50m  RED=R<30m (sharp)",
                    {10, win_size_-30}, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(160,160,160), 1);
        cv::putText(img, "GREEN=avoid_path  RED-DOT=NPC  YELLOW=ego",
                    {10, win_size_-15}, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(160,160,160), 1);

        cv::imshow(kWin_, img);
    }
    void drawPath(cv::Mat& img, const std::vector<Pt>& path, cv::Scalar color, int thick) {
        for (size_t i=1; i<path.size(); ++i) {
            cv::Point p1 = worldToImg(path[i-1].x, path[i-1].y);
            cv::Point p2 = worldToImg(path[i].x, path[i].y);
            if (p1.x < -50 || p1.x > win_size_+50 || p1.y < -50 || p1.y > win_size_+50) continue;
            cv::line(img, p1, p2, color, thick);
        }
    }
    void drawPathColored(cv::Mat& img, const std::vector<Pt>& path, int thick) {
        for (size_t i=1; i<path.size(); ++i) {
            cv::Point p1 = worldToImg(path[i-1].x, path[i-1].y);
            cv::Point p2 = worldToImg(path[i].x, path[i].y);
            if (p1.x < -50 || p1.x > win_size_+50 || p1.y < -50 || p1.y > win_size_+50) continue;
            cv::line(img, p1, p2, kappaColor(path[i].kappa), thick);
        }
    }

    ros::Subscriber ego_sub_, obj_sub_, perf_sub_;
    std::mutex m_;
    std::vector<Pt> path_, avoid_;
    std::deque<Pt> trail_;
    std::vector<Pt> npcs_;
    double ego_x_=0, ego_y_=0, ego_yaw_=0;
    double cte_=0, hdg_=0, v_=0;
    int win_size_=700;
    double view_margin_=30.0;
    const std::string kWin_ = "MORAI MPC Dashboard";
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_dashboard");
    ros::NodeHandle nh("~");
    SimpleDashboard d(nh);
    d.spin();
    return 0;
}
