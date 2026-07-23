// mpc_dashboard_node.cpp — OpenCV 기반 C++ 실시간 대시보드
//   - 좌측 큰 맵 (D=파랑, R=빨강, 차량 화살표 + 트레일)
//   - 우측 상단: 추종율% + 헤딩일치율% + 기어/컨트롤러/저속 배지
//   - 우측 하단: CTE / HDG / Speed 시계열
//
// 토픽:
//   /mpc_performance (std_msgs/Float32MultiArray, [near_dist, solve_ms, cte, hdg_deg, v_kmh, max_kappa, tgt_kmh])
//   /mpc_status      (std_msgs/String)
//   /localization/ego_status (morai_msgs/EgoVehicleStatus)
//
// 파라미터:
//   ~path_file  (string, default="")
//   ~window_m   (double, default=18.0)
//   ~hdg_pct_thresh_deg (double, default=5.0)

#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/ObjectStatusList.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <jsoncpp/json/json.h>

#include <atomic>
#include <cmath>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace {

// NASA HUD 스타일 — 다크/시안/앰버 팔레트, 그리드, 모노 텔레메트리
constexpr int    kW          = 1280;
constexpr int    kH          = 920;
constexpr int    kHeaderH    = 30;
constexpr int    kMapX0      = 10,  kMapY0 = kHeaderH + 10;
constexpr int    kMapW       = 760, kMapH = 870;
constexpr int    kPanelX0    = 790, kPanelY0 = kHeaderH + 10;
constexpr int    kPanelW     = 480;
constexpr int    kStatsH     = 230;
constexpr int    kGraphH     = 200;
constexpr int    kGraphGap   = 10;

// 색상 팔레트 (BGR) — 깊은 우주색 + HUD 시안/앰버
const cv::Scalar kBg        (12, 8, 4);          // 거의 검정 (deep space)
const cv::Scalar kBgPanel   (28, 22, 14);        // 패널 약간 밝은 다크
const cv::Scalar kBgMap     (24, 16, 8);         // 맵 배경
const cv::Scalar kGrid      (60, 50, 40);        // 그리드 (어두운 시안)
const cv::Scalar kGridMaj   (90, 80, 60);        // 메이저 그리드
const cv::Scalar kBorder    (140, 120, 80);      // 박스 테두리 (콜드 시안)
const cv::Scalar kBorderHi  (255, 200, 100);     // 액센트 테두리 (HUD 시안)
const cv::Scalar kFg        (220, 220, 220);     // 일반 텍스트
const cv::Scalar kFgDim     (130, 130, 130);     // 흐린 텍스트
const cv::Scalar kCyan      (255, 220, 0);       // 시안 (BGR=0,220,255 → ROS BGR로 0xFFDC00? 사실 BGR(255,220,0) = 시안 #00DCFF)
const cv::Scalar kAmber     (0, 200, 255);       // 앰버 (#FFC800 in BGR ≈ amber)
const cv::Scalar kGreen     (100, 255, 60);      // HUD green
const cv::Scalar kRed       (60, 60, 255);       // critical red
const cv::Scalar kAccent    (255, 200, 80);      // 보조 시안
const cv::Scalar kPurple    (180, 100, 255);     // NMPC 표시
const cv::Scalar kYellow    (40, 220, 240);      // warning

cv::Scalar pctColor(double p) {
    if (p >= 90) return kGreen;
    if (p >= 70) return kAmber;
    return kRed;
}

}  // namespace

class Dashboard {
public:
    Dashboard(ros::NodeHandle& nh) : nh_(nh) {
        nh_.param<std::string>("path_file",  path_file_, "");
        if (path_file_.empty()) {
            nh_.param<std::string>("/path_follower_node/path_file", path_file_, "");
        }
        nh_.param<double>("window_m", window_m_, 18.0);
        nh_.param<double>("hdg_pct_thresh_deg", hdg_pct_thresh_, 5.0);
        nh_.param<bool>("follow_vehicle", follow_vehicle_, false);  // 기본: path 전체 보임

        loadPath();

        sub_perf_   = nh_.subscribe("/mpc_performance", 1, &Dashboard::cbPerf, this);
        sub_status_ = nh_.subscribe("/mpc_status",      1, &Dashboard::cbStatus, this);
        sub_ego_    = nh_.subscribe("/localization/ego_status", 1, &Dashboard::cbEgo, this);
        sub_obj_    = nh_.subscribe("/Object_topic",    1, &Dashboard::cbObj, this);
        sub_avoid_  = nh_.subscribe("/avoidance_offset", 1, &Dashboard::cbAvoid, this);
        sub_livepath_ = nh_.subscribe("/avoid_path",     1, &Dashboard::cbLivePath, this);  // planner live 경로(차가 실제 추종)
        sub_mode_   = nh_.subscribe("/avoid_mode",       1, &Dashboard::cbMode, this);  // 현재 모드(추월/회피/직진)

        cv::namedWindow(kWin_, cv::WINDOW_AUTOSIZE);
        cv::moveWindow(kWin_, 0, 0);   // WSLg: 좌상단 고정
        // startWindowThread() 제거 — WSLg/GTK에서 별도 GUI 스레드가 메인 imshow 갱신 막아 검은화면
        cv::Mat init(kH, kW, CV_8UC3, kBg);
        cv::imshow(kWin_, init);
        cv::waitKey(30);
    }

    void spin() {
        // 단순 spinOnce + waitKey 패턴 (AsyncSpinner 제거 → GUI 응답성 향상)
        // startWindowThread() ctor에서 호출됨 → main waitKey는 짧게(1ms) 호출하여 X event polling
        ros::Rate r(20.0);
        while (ros::ok()) {
            ros::spinOnce();
            render();
            int k = cv::waitKey(30);   // WSLg: 이벤트펌핑+repaint 보장 (1ms는 갱신누락→검은화면)
            if (k == 27 || k == 'q') { ros::shutdown(); break; }
            r.sleep();
        }
        cv::destroyAllWindows();
    }

private:
    static constexpr const char* kWin_ = "MPC Dashboard (C++)";

    // ── 콜백 ──────────────────────────────────────────────────
    void cbPerf(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() < 7) return;
        std::lock_guard<std::mutex> lk(mu_);
        if (t0_ == 0.0) t0_ = ros::Time::now().toSec();
        double t = ros::Time::now().toSec() - t0_;

        double cte    = msg->data[2];
        double hdg    = msg->data[3];
        double v_kmh  = msg->data[4];
        double tgt    = msg->data[6];

        t_buf_.push_back(t);
        cte_buf_.push_back(cte);
        hdg_buf_.push_back(hdg);
        v_buf_.push_back(v_kmh);
        tgt_buf_.push_back(tgt);
        const size_t kBuf = 1500;
        while (t_buf_.size() > kBuf) {
            t_buf_.pop_front(); cte_buf_.pop_front();
            hdg_buf_.pop_front(); v_buf_.pop_front(); tgt_buf_.pop_front();
        }

        // 누적 통계 (RMSE/Mode용)
        total_++;
        cte_sq_ += cte * cte;
        hdg_sq_ += hdg * hdg;

        // 윈도우드 퍼센트 (최근 N틱; 커브 지나면 회복) — 0/1 플래그를 sum과 함께 deque에 보관
        double a = std::abs(cte);
        int8_t f10 = (a < 0.10) ? 1 : 0;
        int8_t f20 = (a < 0.20) ? 1 : 0;
        int8_t f30 = (a < 0.30) ? 1 : 0;
        int8_t f50 = (a < 0.50) ? 1 : 0;
        int8_t fhd = (std::abs(hdg) < hdg_pct_thresh_) ? 1 : 0;
        win_f10_.push_back(f10); sum_w10_ += f10;
        win_f20_.push_back(f20); sum_w20_ += f20;
        win_f30_.push_back(f30); sum_w30_ += f30;
        win_f50_.push_back(f50); sum_w50_ += f50;
        win_fhd_.push_back(fhd); sum_whd_ += fhd;
        const size_t kWin = 60;    // 3초 (20Hz) — 마지막 정렬 즉각 반영
        if (win_f10_.size() > kWin) { sum_w10_ -= win_f10_.front(); win_f10_.pop_front(); }
        if (win_f20_.size() > kWin) { sum_w20_ -= win_f20_.front(); win_f20_.pop_front(); }
        if (win_f30_.size() > kWin) { sum_w30_ -= win_f30_.front(); win_f30_.pop_front(); }
        if (win_f50_.size() > kWin) { sum_w50_ -= win_f50_.front(); win_f50_.pop_front(); }
        if (win_fhd_.size() > kWin) { sum_whd_ -= win_fhd_.front(); win_fhd_.pop_front(); }
    }

    void cbMode(const std_msgs::String::ConstPtr& msg) {
        std::lock_guard<std::mutex> lk(mu_);
        avoid_mode_ = msg->data;
    }

    void cbStatus(const std_msgs::String::ConstPtr& msg) {
        std::lock_guard<std::mutex> lk(mu_);
        cur_mode_ = msg->data;
        if (cur_mode_.rfind("NMPC", 0) == 0) { ctrl_ = "NMPC"; nmpc_n_++; }
        else if (cur_mode_.rfind("RECOV", 0) == 0) { ctrl_ = "RECOV"; recov_n_++; }
        else { ctrl_ = "LTV"; }
        std::string g = (cur_mode_.size() >= 2 &&
                         cur_mode_.substr(cur_mode_.size() - 2) == "_R") ? "R" : "D";
        if (g != cur_gear_) { gear_switch_n_++; cur_gear_ = g; }
        is_low_speed_ = (cur_mode_ == "NMPC_LO");
    }

    void cbEgo(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
        std::lock_guard<std::mutex> lk(mu_);
        ego_x_   = msg->position.x;
        ego_y_   = msg->position.y;
        ego_yaw_ = msg->heading * M_PI / 180.0;
        ego_v_   = msg->velocity.x;
        ego_rcvd_ = true;
        trail_.emplace_back(ego_x_, ego_y_);
        if (trail_.size() > 4000) trail_.pop_front();
    }

    // ── 패스 로드 ─────────────────────────────────────────────
    void loadPath() {
        if (path_file_.empty()) {
            ROS_WARN("[dashboard_cpp] path_file 없음 — 글로벌 패스 비활성");
            return;
        }
        std::ifstream f(path_file_);
        if (!f.is_open()) { ROS_WARN("[dashboard_cpp] path_file 열기 실패: %s", path_file_.c_str()); return; }
        Json::Value root;
        f >> root;
        const auto& wps = root["waypoints"];
        path_min_x_ =  1e18; path_max_x_ = -1e18;
        path_min_y_ =  1e18; path_max_y_ = -1e18;
        for (auto& w : wps) {
            std::string g = w.get("gear", "D").asString();
            double x = w["x"].asDouble();
            double y = w["y"].asDouble();
            if (g == "R") { wp_r_x_.push_back(x); wp_r_y_.push_back(y); }
            else          { wp_d_x_.push_back(x); wp_d_y_.push_back(y); }
            path_min_x_ = std::min(path_min_x_, x); path_max_x_ = std::max(path_max_x_, x);
            path_min_y_ = std::min(path_min_y_, y); path_max_y_ = std::max(path_max_y_, y);
        }
        ROS_INFO("[dashboard_cpp] 패스 로드: %zu D + %zu R 점, x[%.1f..%.1f] y[%.1f..%.1f]",
                 wp_d_x_.size(), wp_r_x_.size(),
                 path_min_x_, path_max_x_, path_min_y_, path_max_y_);
    }

    // ── 렌더링 진입점 ────────────────────────────────────────
    void render() {
        cv::Mat img(kH, kW, CV_8UC3, kBg);
        {
            std::lock_guard<std::mutex> lk(mu_);
            drawHeader(img);
            drawMap(img);
            drawAvoidMode(img);
            drawStatsPanel(img);
            drawGraph(img, "[ CTE ]  LATERAL ERROR", t_buf_, cte_buf_, kCyan,
                      kPanelX0, kPanelY0 + kStatsH + kGraphGap, kPanelW, kGraphH,
                      true, 0.30);
            drawGraph(img, "[ HDG ]  HEADING ERROR", t_buf_, hdg_buf_, kAmber,
                      kPanelX0, kPanelY0 + kStatsH + kGraphGap*2 + kGraphH, kPanelW, kGraphH,
                      true, 5.0);
            drawSpeedGraph(img, kPanelX0, kPanelY0 + kStatsH + kGraphGap*3 + kGraphH*2, kPanelW, kGraphH);
        }
        cv::imshow(kWin_, img);
    }

    // ── planner 모드 배너 (추월/회피/순항) — 지도 좌상단, 색상 구분 ──
    void drawAvoidMode(cv::Mat& img) {
        std::string m = avoid_mode_.empty() ? std::string("CRUISE") : avoid_mode_;
        cv::Scalar col;
        if      (m.rfind("OVERTAKE:BOOST", 0) == 0) col = cv::Scalar(60, 90, 255);   // 빨강: 급가속 추월
        else if (m.rfind("OVERTAKE", 0)      == 0)  col = cv::Scalar(60, 200, 255);  // 주황: 차선변경 진입
        else if (m.rfind("AVOID", 0)         == 0)  col = cv::Scalar(230, 200, 60);  // 시안: 회피
        else                                        col = cv::Scalar(120, 200, 120); // 녹: 순항
        int x = 16, y = kHeaderH + 30;
        double sc = 0.85;
        cv::Size sz = cv::getTextSize(m, cv::FONT_HERSHEY_DUPLEX, sc, 2, nullptr);
        cv::rectangle(img, cv::Rect(x - 10, y - sz.height - 12, sz.width + 20, sz.height + 22),
                      cv::Scalar(20, 24, 30), cv::FILLED);
        cv::rectangle(img, cv::Rect(x - 10, y - sz.height - 12, sz.width + 20, sz.height + 22), col, 2);
        cv::rectangle(img, cv::Rect(x - 10, y - sz.height - 12, 6, sz.height + 22), col, cv::FILLED);
        putText(img, m, x + 6, y, sc, col, cv::FONT_HERSHEY_DUPLEX, 2);
    }

    // ── HUD 헤더 배너 ────────────────────────────────────────
    void drawHeader(cv::Mat& img) {
        cv::rectangle(img, cv::Rect(0, 0, kW, kHeaderH), kBgPanel, cv::FILLED);
        cv::line(img, cv::Point(0, kHeaderH-1), cv::Point(kW, kHeaderH-1), kBorder, 1);
        // 좌측 system tag
        putText(img, "MORAI MPC TRACKING SYSTEM", 12, 20, 0.55, kCyan, cv::FONT_HERSHEY_DUPLEX, 1);
        // 가운데 status (현재 mode)
        std::string st = "MODE: " + (cur_mode_.empty() ? std::string("--") : cur_mode_);
        cv::Size sz = cv::getTextSize(st, cv::FONT_HERSHEY_DUPLEX, 0.55, 1, nullptr);
        putText(img, st, kW/2 - sz.width/2, 20, 0.55, kAmber, cv::FONT_HERSHEY_DUPLEX, 1);
        // 우측 elapsed time + clock
        char buf[64];
        double elapsed = t_buf_.empty() ? 0.0 : t_buf_.back();
        std::snprintf(buf, sizeof(buf), "T+ %05.1f s   |   %s", elapsed,
                     std::to_string((int)ros::Time::now().toSec()).c_str());
        putText(img, buf, kW - 270, 20, 0.5, kFgDim, cv::FONT_HERSHEY_DUPLEX, 1);
    }

    // ── 맵 (좌측) — HUD 그리드 + 컴파스 + 스케일 ───────────
    void drawMap(cv::Mat& img) {
        cv::Rect r(kMapX0, kMapY0, kMapW, kMapH);
        cv::rectangle(img, r, kBgMap, cv::FILLED);
        // 코너 마커 (HUD 스타일)
        int cornL = 16;
        cv::line(img, cv::Point(kMapX0, kMapY0), cv::Point(kMapX0+cornL, kMapY0), kBorderHi, 2);
        cv::line(img, cv::Point(kMapX0, kMapY0), cv::Point(kMapX0, kMapY0+cornL), kBorderHi, 2);
        cv::line(img, cv::Point(kMapX0+kMapW, kMapY0), cv::Point(kMapX0+kMapW-cornL, kMapY0), kBorderHi, 2);
        cv::line(img, cv::Point(kMapX0+kMapW, kMapY0), cv::Point(kMapX0+kMapW, kMapY0+cornL), kBorderHi, 2);
        cv::line(img, cv::Point(kMapX0, kMapY0+kMapH), cv::Point(kMapX0+cornL, kMapY0+kMapH), kBorderHi, 2);
        cv::line(img, cv::Point(kMapX0, kMapY0+kMapH), cv::Point(kMapX0, kMapY0+kMapH-cornL), kBorderHi, 2);
        cv::line(img, cv::Point(kMapX0+kMapW, kMapY0+kMapH), cv::Point(kMapX0+kMapW-cornL, kMapY0+kMapH), kBorderHi, 2);
        cv::line(img, cv::Point(kMapX0+kMapW, kMapY0+kMapH), cv::Point(kMapX0+kMapW, kMapY0+kMapH-cornL), kBorderHi, 2);
        // 얇은 테두리
        cv::rectangle(img, r, kBorder, 1);
        putText(img, "[ MAP ]  GLOBAL TRAJECTORY",
                kMapX0 + 12, kMapY0 + 22, 0.55, kCyan, cv::FONT_HERSHEY_DUPLEX);

        // 맵 좌표계 결정:
        //  (a) follow_vehicle == false: path 전체에 fit (path 보임 + 차량 마커)
        //  (b) follow_vehicle == true : 차량 중심 ±window_m
        bool have_path_bounds = (path_max_x_ > path_min_x_);
        double cx, cy, scale;
        if (follow_vehicle_ && ego_rcvd_) {
            cx = ego_x_; cy = ego_y_;
            scale = std::min(kMapW, kMapH) / (2.0 * window_m_);
        } else if (have_path_bounds) {
            // path 전체 fit (15% 마진)
            cx = (path_min_x_ + path_max_x_) * 0.5;
            cy = (path_min_y_ + path_max_y_) * 0.5;
            double span_x = path_max_x_ - path_min_x_;
            double span_y = path_max_y_ - path_min_y_;
            double span = std::max(span_x, span_y) * 1.15;
            if (span < 5.0) span = 5.0;
            scale = std::min(kMapW, kMapH) / span;
        } else if (ego_rcvd_) {
            cx = ego_x_; cy = ego_y_;
            scale = std::min(kMapW, kMapH) / (2.0 * window_m_);
        } else {
            putText(img, "(waiting for /localization/ego_status)",
                    kMapX0 + 8, kMapY0 + kMapH - 12, 0.5, kFgDim);
            return;
        }
        auto W2I = [&](double wx, double wy) -> cv::Point {
            int ix = kMapX0 + kMapW / 2 + (int)std::round((wx - cx) * scale);
            int iy = kMapY0 + kMapH / 2 - (int)std::round((wy - cy) * scale);
            return cv::Point(ix, iy);
        };

        auto inMap = [&](const cv::Point& p) {
            return p.x >= kMapX0 && p.x < kMapX0 + kMapW &&
                   p.y >= kMapY0 && p.y < kMapY0 + kMapH;
        };

        // ── HUD 그리드 (월드 좌표 기준 10m 단위) ────────────
        double world_per_px = 1.0 / scale;
        double grid_step_m = 10.0;
        // 화면 외곽이 보이는 월드 좌표 범위
        double wL = cx - (kMapW/2.0) * world_per_px;
        double wR = cx + (kMapW/2.0) * world_per_px;
        double wB = cy - (kMapH/2.0) * world_per_px;
        double wT = cy + (kMapH/2.0) * world_per_px;
        // 수직 그리드 라인 (x = N*10m)
        double gx0 = std::floor(wL / grid_step_m) * grid_step_m;
        for (double gx = gx0; gx <= wR; gx += grid_step_m) {
            cv::Point p1 = W2I(gx, wB), p2 = W2I(gx, wT);
            bool maj = std::fmod(std::round(gx), 50.0) == 0;
            cv::line(img, p1, p2, maj ? kGridMaj : kGrid, 1, cv::LINE_AA);
            // 좌표 라벨 (메이저만)
            if (maj && p1.x > kMapX0+10 && p1.x < kMapX0+kMapW-30) {
                char b[16]; std::snprintf(b, sizeof(b), "%.0f", gx);
                putText(img, b, p1.x + 2, kMapY0 + kMapH - 6, 0.35, kFgDim);
            }
        }
        // 수평 그리드 라인 (y = N*10m)
        double gy0 = std::floor(wB / grid_step_m) * grid_step_m;
        for (double gy = gy0; gy <= wT; gy += grid_step_m) {
            cv::Point p1 = W2I(wL, gy), p2 = W2I(wR, gy);
            bool maj = std::fmod(std::round(gy), 50.0) == 0;
            cv::line(img, p1, p2, maj ? kGridMaj : kGrid, 1, cv::LINE_AA);
            if (maj && p1.y > kMapY0+30 && p1.y < kMapY0+kMapH-10) {
                char b[16]; std::snprintf(b, sizeof(b), "%.0f", gy);
                putText(img, b, kMapX0 + 6, p1.y - 3, 0.35, kFgDim);
            }
        }

        // ── LIVE 경로 우선: planner가 발행한 실제 추종경로 하나만 표시 (static과 이중표시 제거) ──
        bool have_live = !live_x_.empty() && (ros::Time::now() - live_stamp_).toSec() < 1.0;
        int path_dot_r = follow_vehicle_ ? 2 : 1;
        if (have_live) {
            const cv::Scalar kLive(70, 255, 120);   // bright green — 차가 실제 따라가는 경로
            for (size_t i = 1; i < live_x_.size(); ++i) {
                cv::Point p1 = W2I(live_x_[i-1], live_y_[i-1]);
                cv::Point p2 = W2I(live_x_[i],   live_y_[i]);
                if (inMap(p1) || inMap(p2)) cv::line(img, p1, p2, kLive, 2, cv::LINE_AA);
            }
            for (size_t i = 0; i < live_x_.size(); i += 2) {
                cv::Point p = W2I(live_x_[i], live_y_[i]);
                if (inMap(p)) cv::circle(img, p, path_dot_r, kLive, cv::FILLED);
            }
        } else {
        // D 경로 — 시안 (close-up에서 더 잘 보이게) — live 없을때만(planner off)
        const cv::Scalar kPathD(220, 170, 40);   // brighter cyan
        const cv::Scalar kPathR(50, 170, 240);   // brighter amber
        // 차량 중심 모드에서는 점 키움
        for (size_t i = 0; i < wp_d_x_.size(); ++i) {
            cv::Point p = W2I(wp_d_x_[i], wp_d_y_[i]);
            if (inMap(p)) cv::circle(img, p, path_dot_r, kPathD, cv::FILLED);
        }
        for (size_t i = 0; i < wp_r_x_.size(); ++i) {
            cv::Point p = W2I(wp_r_x_[i], wp_r_y_[i]);
            if (inMap(p)) cv::circle(img, p, path_dot_r, kPathR, cv::FILLED);
        }
        // close-up에서 path를 line으로 연결 (각 segment 내부)
        if (follow_vehicle_) {
            // D 점들 segment 단위로 연결 (인접 점 거리 1m 이하면 line)
            for (size_t i = 1; i < wp_d_x_.size(); ++i) {
                cv::Point p1 = W2I(wp_d_x_[i-1], wp_d_y_[i-1]);
                cv::Point p2 = W2I(wp_d_x_[i],   wp_d_y_[i]);
                double seg = std::hypot(wp_d_x_[i]-wp_d_x_[i-1], wp_d_y_[i]-wp_d_y_[i-1]);
                if (seg < 1.0 && (inMap(p1) || inMap(p2)))
                    cv::line(img, p1, p2, kPathD, 1, cv::LINE_AA);
            }
            for (size_t i = 1; i < wp_r_x_.size(); ++i) {
                cv::Point p1 = W2I(wp_r_x_[i-1], wp_r_y_[i-1]);
                cv::Point p2 = W2I(wp_r_x_[i],   wp_r_y_[i]);
                double seg = std::hypot(wp_r_x_[i]-wp_r_x_[i-1], wp_r_y_[i]-wp_r_y_[i-1]);
                if (seg < 1.0 && (inMap(p1) || inMap(p2)))
                    cv::line(img, p1, p2, kPathR, 1, cv::LINE_AA);
            }
        }
        }  // end else (static 경로 — live 없을때만)

        // ── 트레일 (그라디언트 + 글로우 효과) ────────────────
        // 오래된 점 → 어둡고 얇음, 최근 점 → 밝고 두꺼움 (깃발 효과)
        const cv::Scalar kTrailHot(80, 255, 255);   // 노랑 hot (BGR=80,255,255)
        const cv::Scalar kTrailMid(120, 240, 100);  // 라임
        const cv::Scalar kTrailCold(160, 100, 50);  // 어두운 청록
        size_t tn = trail_.size();
        for (size_t i = 1; i < tn; ++i) {
            cv::Point p1 = W2I(trail_[i-1].first, trail_[i-1].second);
            cv::Point p2 = W2I(trail_[i].first, trail_[i].second);
            if (!inMap(p1) && !inMap(p2)) continue;
            double age = (double)(tn - i) / std::max((size_t)1, tn);
            cv::Scalar col;
            int thick;
            if (age < 0.15)      { col = kTrailHot; thick = 4; }
            else if (age < 0.5)  { col = kTrailMid; thick = 3; }
            else                 { col = kTrailCold; thick = 2; }
            // glow: 두꺼운 외곽 (어둡게) + 얇은 중심 (밝게)
            cv::line(img, p1, p2, cv::Scalar(col[0]/3, col[1]/3, col[2]/3), thick + 2, cv::LINE_AA);
            cv::line(img, p1, p2, col, thick, cv::LINE_AA);
        }

        // ── 장애물 (NPC/보행자) 그리기 ──────────────────────────
        // (render가 이미 mu_ lock 잡고 호출 — 재귀 lock 금지)
        {
            for (const auto& ob : obstacles_) {
                cv::Point pc = W2I(ob.x, ob.y);
                if (!inMap(pc)) continue;
                double yaw_rad = ob.heading * M_PI / 180.0;
                double cs = std::cos(yaw_rad), sn = std::sin(yaw_rad);
                double hl = std::max(ob.sx, 1.0) * 0.5;   // half length
                double hw = std::max(ob.sy, 1.0) * 0.5;   // half width
                // 4 corners (world coord)
                std::vector<cv::Point> corners;
                for (auto [dx, dy] : std::vector<std::pair<double,double>>{
                        {hl,  hw}, {hl, -hw}, {-hl, -hw}, {-hl, hw}}) {
                    double wx = ob.x + cs * dx - sn * dy;
                    double wy = ob.y + sn * dx + cs * dy;
                    corners.push_back(W2I(wx, wy));
                }
                // 진한 빨강 채우기 + 흰색 테두리
                cv::Scalar fill(40, 40, 220);   // BGR (red)
                cv::Scalar edge(240, 240, 240);
                std::vector<std::vector<cv::Point>> polys = {corners};
                cv::fillPoly(img, polys, fill, cv::LINE_AA);
                cv::polylines(img, polys, true, edge, 2, cv::LINE_AA);
                // heading 화살표
                double hx = ob.x + cs * hl, hy = ob.y + sn * hl;
                cv::line(img, pc, W2I(hx, hy), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            }
        }

        // ── 회피 경로 (reference path + avoid_offset 만큼 lateral shift) ─
        // live 경로 표시중이면 이 가짜(offset shift) 경로는 그리지 않음 — 이중선 제거
        if (!have_live && std::abs(avoid_offset_) > 0.05 && wp_d_x_.size() > 5) {
            // wp_d 좌표를 path direction 기준 lateral 이동 (좌측+)
            std::vector<cv::Point> avoid_pts;
            for (size_t i = 1; i < wp_d_x_.size(); ++i) {
                double dx = wp_d_x_[i] - wp_d_x_[i-1];
                double dy = wp_d_y_[i] - wp_d_y_[i-1];
                double len = std::hypot(dx, dy);
                if (len < 1e-3) continue;
                // 좌측 단위 벡터 (path 진행방향에서 90° ccw 회전)
                double nx = -dy / len, ny = dx / len;
                double sx = wp_d_x_[i] + avoid_offset_ * nx;
                double sy = wp_d_y_[i] + avoid_offset_ * ny;
                cv::Point p = W2I(sx, sy);
                if (inMap(p)) avoid_pts.push_back(p);
            }
            for (size_t i = 1; i < avoid_pts.size(); ++i) {
                cv::line(img, avoid_pts[i-1], avoid_pts[i],
                         cv::Scalar(0, 255, 255), 3, cv::LINE_AA);  // 밝은 노랑 (BGR)
            }
        }

        if (ego_rcvd_) {
            cv::Point pego = W2I(ego_x_, ego_y_);
            // 거리 동심원 (5/10/15m)
            for (double r_m : {5.0, 10.0, 15.0}) {
                int r_px = (int)std::round(r_m * scale);
                if (r_px > 5 && r_px < kMapW)
                    cv::circle(img, pego, r_px, kGrid, 1, cv::LINE_AA);
            }

            // ── 차량 마커 (커진 화살표 + 글로우 + 외곽 박스) ─────
            double car_L = 4.0;   // m (차량 길이 표현)
            double car_W = 2.0;   // m (차량 폭)
            double cy_yaw = ego_yaw_;
            double cs = std::cos(cy_yaw), sn = std::sin(cy_yaw);
            // 4 corner of car body (rear-left, rear-right, front-right, front-left)
            std::vector<cv::Point> car_pts = {
                W2I(ego_x_ - car_L*0.4*cs - car_W*0.5*sn, ego_y_ - car_L*0.4*sn + car_W*0.5*cs),
                W2I(ego_x_ - car_L*0.4*cs + car_W*0.5*sn, ego_y_ - car_L*0.4*sn - car_W*0.5*cs),
                W2I(ego_x_ + car_L*0.6*cs + car_W*0.5*sn, ego_y_ + car_L*0.6*sn - car_W*0.5*cs),
                W2I(ego_x_ + car_L*0.6*cs - car_W*0.5*sn, ego_y_ + car_L*0.6*sn + car_W*0.5*cs)
            };
            // 차량 본체: 빨간색 채움 + 흰 외곽 (강조)
            const cv::Scalar kCarFill(80, 80, 240);    // 빨강 (BGR)
            const cv::Scalar kCarEdge(255, 255, 255);  // 흰 외곽
            const cv::Scalar kCarFront(0, 255, 255);   // 앞쪽 강조 (노랑)
            cv::fillConvexPoly(img, car_pts.data(), 4, kCarFill, cv::LINE_AA);
            std::vector<std::vector<cv::Point>> contours = {car_pts};
            cv::polylines(img, contours, true, kCarEdge, 2, cv::LINE_AA);
            // 앞쪽 line 강조 (노랑)
            cv::line(img, car_pts[2], car_pts[3], kCarFront, 3, cv::LINE_AA);

            // 헤딩 벡터 (긴 노랑 화살표 - 글로우)
            double Lvec = 6.0;
            cv::Point phead = W2I(ego_x_ + Lvec * cs, ego_y_ + Lvec * sn);
            cv::arrowedLine(img, pego, phead, cv::Scalar(40, 40, 40), 5, cv::LINE_AA, 0, 0.30);  // shadow
            cv::arrowedLine(img, pego, phead, kCarFront, 3, cv::LINE_AA, 0, 0.32);

            // 중심점 (이중원: 외곽 흰, 내부 빨강)
            cv::circle(img, pego, 6, kCarEdge, cv::FILLED, cv::LINE_AA);
            cv::circle(img, pego, 4, kCarFill, cv::FILLED, cv::LINE_AA);
        }

        // ── 장애물 시각화 (/Object_topic) ─────────────────────
        {
            // render()가 이미 mu_ 보유 → 재lock 금지(non-recursive mutex 이중lock=데드락→검은화면)
            for (const auto& o : obstacles_) {
                cv::Point poc = W2I(o.x, o.y);
                if (!inMap(poc)) continue;
                const double r_m  = 0.5 * std::hypot(std::max(o.sx, 1.0),
                                                     std::max(o.sy, 1.0));
                const int    r_px = std::max(6, (int)std::round(r_m * scale));
                // safe margin band (얇은 외곽 — 1.5m default)
                const int sm_px = std::max(r_px + 4,
                                           (int)std::round((r_m + 1.5) * scale));
                cv::circle(img, poc, sm_px, cv::Scalar(0, 200, 255), 1, cv::LINE_AA);  // 노랑 margin
                cv::circle(img, poc, r_px,  cv::Scalar(0,  80, 230), 2, cv::LINE_AA);  // 주황 외곽
                cv::circle(img, poc, std::max(3, r_px/3),
                           cv::Scalar(0, 0, 230), cv::FILLED, cv::LINE_AA);             // 빨강 중심
                // heading 화살표 (2m)
                const double hx = o.x + 2.0 * std::cos(o.heading);
                const double hy = o.y + 2.0 * std::sin(o.heading);
                cv::Point phd = W2I(hx, hy);
                cv::arrowedLine(img, poc, phd, cv::Scalar(0, 80, 230), 1,
                                cv::LINE_AA, 0, 0.35);
                // 라벨 OBS
                putText(img, "OBS", poc.x + r_px + 2, poc.y - r_px - 2,
                        0.4, cv::Scalar(0, 80, 230));
            }
        }

        // 컴파스 N (좌상단)
        int cx_c = kMapX0 + kMapW - 50, cy_c = kMapY0 + 60;
        cv::circle(img, cv::Point(cx_c, cy_c), 22, kBorder, 1, cv::LINE_AA);
        cv::circle(img, cv::Point(cx_c, cy_c), 22, kBgPanel, cv::FILLED);
        cv::circle(img, cv::Point(cx_c, cy_c), 22, kBorder, 1, cv::LINE_AA);
        cv::line(img, cv::Point(cx_c, cy_c-18), cv::Point(cx_c, cy_c+10), kAmber, 1);
        putText(img, "N", cx_c - 5, cy_c - 22, 0.45, kAmber, cv::FONT_HERSHEY_DUPLEX);

        // 스케일 바 (좌하단)
        int sb_x = kMapX0 + 14, sb_y = kMapY0 + kMapH - 18;
        int sb_len = (int)std::round(10.0 * scale);  // 10m
        cv::line(img, cv::Point(sb_x, sb_y), cv::Point(sb_x + sb_len, sb_y), kFg, 2);
        cv::line(img, cv::Point(sb_x, sb_y - 4), cv::Point(sb_x, sb_y + 4), kFg, 1);
        cv::line(img, cv::Point(sb_x + sb_len, sb_y - 4), cv::Point(sb_x + sb_len, sb_y + 4), kFg, 1);
        putText(img, "10 m", sb_x, sb_y - 8, 0.4, kFgDim);

        // ego 좌표 readout (우하단)
        if (ego_rcvd_) {
            char buf[80];
            std::snprintf(buf, sizeof(buf), "POS  X %+8.1f  Y %+8.1f   YAW %+6.1f°",
                          ego_x_, ego_y_, ego_yaw_ * 180.0 / M_PI);
            putText(img, buf, kMapX0 + 130, kMapY0 + kMapH - 10, 0.42, kFgDim, cv::FONT_HERSHEY_DUPLEX);
        }
    }

    // ── 우측 상단 텔레메트리 패널 ──────────────────────────
    void drawStatsPanel(cv::Mat& img) {
        cv::Rect r(kPanelX0, kPanelY0, kPanelW, kStatsH);
        cv::rectangle(img, r, kBgPanel, cv::FILLED);
        // HUD 코너 마커
        int cl = 12;
        cv::line(img, cv::Point(kPanelX0, kPanelY0), cv::Point(kPanelX0+cl, kPanelY0), kBorderHi, 2);
        cv::line(img, cv::Point(kPanelX0, kPanelY0), cv::Point(kPanelX0, kPanelY0+cl), kBorderHi, 2);
        cv::line(img, cv::Point(kPanelX0+kPanelW, kPanelY0), cv::Point(kPanelX0+kPanelW-cl, kPanelY0), kBorderHi, 2);
        cv::line(img, cv::Point(kPanelX0+kPanelW, kPanelY0), cv::Point(kPanelX0+kPanelW, kPanelY0+cl), kBorderHi, 2);
        cv::line(img, cv::Point(kPanelX0, kPanelY0+kStatsH), cv::Point(kPanelX0+cl, kPanelY0+kStatsH), kBorderHi, 2);
        cv::line(img, cv::Point(kPanelX0, kPanelY0+kStatsH), cv::Point(kPanelX0, kPanelY0+kStatsH-cl), kBorderHi, 2);
        cv::line(img, cv::Point(kPanelX0+kPanelW, kPanelY0+kStatsH), cv::Point(kPanelX0+kPanelW-cl, kPanelY0+kStatsH), kBorderHi, 2);
        cv::line(img, cv::Point(kPanelX0+kPanelW, kPanelY0+kStatsH), cv::Point(kPanelX0+kPanelW, kPanelY0+kStatsH-cl), kBorderHi, 2);
        cv::rectangle(img, r, kBorder, 1);
        // 패널 타이틀
        putText(img, "[ TELEMETRY ]", kPanelX0 + 12, kPanelY0 + 18, 0.5, kCyan, cv::FONT_HERSHEY_DUPLEX);

        // 윈도우드 pct (커브 통과 후 자동 회복)
        size_t wn = win_f20_.size();
        double pct_cte = wn ? (100.0 * sum_w20_ / wn) : 0.0;
        double pct_hdg = wn ? (100.0 * sum_whd_ / wn) : 0.0;

        // ─ 상단 큰 게이지 (CTE % / HDG %) — HUD 스타일 ─────
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%3.0f", pct_cte);
        cv::putText(img, buf, cv::Point(kPanelX0 + 18, kPanelY0 + 70),
                    cv::FONT_HERSHEY_DUPLEX, 1.7, pctColor(pct_cte), 2);
        putText(img, "%", kPanelX0 + 100, kPanelY0 + 60, 0.6, pctColor(pct_cte), cv::FONT_HERSHEY_DUPLEX);
        putText(img, "CTE <= 20 cm", kPanelX0 + 18, kPanelY0 + 88, 0.42, kFgDim, cv::FONT_HERSHEY_DUPLEX);
        putText(img, "[ 10s window ]", kPanelX0 + 18, kPanelY0 + 102, 0.35, kBorder);

        std::snprintf(buf, sizeof(buf), "%3.0f", pct_hdg);
        cv::putText(img, buf, cv::Point(kPanelX0 + 150, kPanelY0 + 70),
                    cv::FONT_HERSHEY_DUPLEX, 1.7, pctColor(pct_hdg), 2);
        putText(img, "%", kPanelX0 + 232, kPanelY0 + 60, 0.6, pctColor(pct_hdg), cv::FONT_HERSHEY_DUPLEX);
        std::snprintf(buf, sizeof(buf), "HDG <= %.0f deg", hdg_pct_thresh_);
        putText(img, buf, kPanelX0 + 150, kPanelY0 + 88, 0.42, kFgDim, cv::FONT_HERSHEY_DUPLEX);
        putText(img, "[ 10s window ]", kPanelX0 + 150, kPanelY0 + 102, 0.35, kBorder);

        // ─ 우상단 GEAR / CTRL 박스 (HUD 스타일) ───────────
        int gx = kPanelX0 + kPanelW - 130;
        int gy = kPanelY0 + 30;
        cv::Rect gear_r(gx, gy, 118, 78);
        cv::rectangle(img, gear_r, kBgMap, cv::FILLED);
        cv::Scalar gear_col = (cur_gear_ == "R") ? kAmber : kCyan;
        cv::rectangle(img, gear_r, gear_col, 2);
        cv::putText(img, cur_gear_, cv::Point(gx + 38, gy + 52),
                    cv::FONT_HERSHEY_DUPLEX, 1.8, gear_col, 2);
        std::string ctrl_label = "CTRL: " + ctrl_;
        putText(img, ctrl_label, gx + 6, gy + 70, 0.4, kFgDim, cv::FONT_HERSHEY_DUPLEX);
        // controller 표시 LED
        cv::Scalar ctrl_col = (ctrl_ == "NMPC") ? kPurple :
                              (ctrl_ == "RECOV") ? kAmber : kCyan;
        cv::circle(img, cv::Point(gx + 110, gy + 12), 4, ctrl_col, cv::FILLED);

        // 저속 배지
        if (is_low_speed_) {
            cv::Rect lb(gx, gy + 84, 118, 22);
            cv::rectangle(img, lb, cv::Scalar(40, 30, 60), cv::FILLED);
            cv::rectangle(img, lb, kPurple, 1);
            cv::putText(img, "LOW SPEED", cv::Point(gx + 12, gy + 100),
                        cv::FONT_HERSHEY_DUPLEX, 0.45, kPurple, 1);
        }

        // 통계 텍스트 (좌하단 영역)
        double cte_rmse = total_ ? std::sqrt(cte_sq_ / total_) : 0.0;
        double hdg_rmse = total_ ? std::sqrt(hdg_sq_ / total_) : 0.0;
        double elapsed  = t_buf_.empty() ? 0.0 : t_buf_.back();

        // ─ HUD 텔레메트리 라인 (모노 헬베티카 스타일) ─────
        int sx = kPanelX0 + 18;
        int sy = kPanelY0 + 130;
        char line[128];
        auto putln = [&](const char* lbl, const char* val, const cv::Scalar& vcol = kFg) {
            putText(img, lbl, sx, sy, 0.4, kFgDim, cv::FONT_HERSHEY_DUPLEX);
            putText(img, val, sx + 130, sy, 0.4, vcol, cv::FONT_HERSHEY_DUPLEX);
            sy += 16;
        };
        char val[64];
        std::snprintf(val, sizeof(val), "%5.1f / %5.1f / %5.1f",
                      wn ? 100.0 * sum_w10_ / wn : 0.0,
                      wn ? 100.0 * sum_w30_ / wn : 0.0,
                      wn ? 100.0 * sum_w50_ / wn : 0.0);
        putln("CTE  10/30/50cm %", val);
        std::snprintf(val, sizeof(val), "%6.3f m   %5.1f deg", cte_rmse, hdg_rmse);
        putln("RMSE  CTE / HDG  ", val);
        std::snprintf(val, sizeof(val), "%4d / %4d / %4d", gear_switch_n_, nmpc_n_, recov_n_);
        putln("SWITCH / NMPC / RECOV", val);
        std::snprintf(val, sizeof(val), "%6.1f s", elapsed);
        putln("MISSION TIME", val, kCyan);
    }

    // ── 시계열 그래프 (HUD 스타일) ─────────────────────────
    void drawGraph(cv::Mat& img, const std::string& title,
                   const std::deque<double>& tb, const std::deque<double>& yb,
                   const cv::Scalar& color,
                   int x0, int y0, int w, int h,
                   bool center0, double lim_min) {
        cv::Rect r(x0, y0, w, h);
        cv::rectangle(img, r, kBgMap, cv::FILLED);
        // HUD 코너
        int cl = 8;
        cv::line(img, cv::Point(x0, y0), cv::Point(x0+cl, y0), kBorderHi, 2);
        cv::line(img, cv::Point(x0, y0), cv::Point(x0, y0+cl), kBorderHi, 2);
        cv::line(img, cv::Point(x0+w, y0+h), cv::Point(x0+w-cl, y0+h), kBorderHi, 2);
        cv::line(img, cv::Point(x0+w, y0+h), cv::Point(x0+w, y0+h-cl), kBorderHi, 2);
        cv::rectangle(img, r, kBorder, 1);
        putText(img, title, x0 + 10, y0 + 16, 0.42, kCyan, cv::FONT_HERSHEY_DUPLEX);

        if (tb.size() < 2) return;
        // 최근 60s 윈도우
        double t_end = tb.back();
        double t_start = std::max(0.0, t_end - 60.0);
        // y 범위
        double y_max = lim_min;
        for (size_t i = 0; i < yb.size(); ++i)
            y_max = std::max(y_max, std::abs(yb[i]));
        y_max *= 1.2;
        double y_min = center0 ? -y_max : 0.0;
        if (!center0) y_max = std::max(lim_min, y_max);

        auto X = [&](double t) {
            double rng = std::max(1e-3, t_end - t_start);
            return x0 + 30 + (int)std::round((t - t_start) / rng * (w - 40));
        };
        auto Y = [&](double y) {
            double rng = std::max(1e-3, y_max - y_min);
            return y0 + h - 12 - (int)std::round((y - y_min) / rng * (h - 30));
        };

        // 0 라인
        cv::line(img, cv::Point(X(t_start), Y(0)), cv::Point(X(t_end), Y(0)),
                 cv::Scalar(180,180,180), 1, cv::LINE_AA);
        // 임계 라인 (CTE 0.2, HDG 임계)
        if (title.find("CTE") != std::string::npos) {
            cv::line(img, cv::Point(X(t_start), Y(0.2)), cv::Point(X(t_end), Y(0.2)), kYellow, 1);
            cv::line(img, cv::Point(X(t_start), Y(-0.2)), cv::Point(X(t_end), Y(-0.2)), kYellow, 1);
        } else if (title.find("HDG") != std::string::npos) {
            cv::line(img, cv::Point(X(t_start), Y(hdg_pct_thresh_)),
                     cv::Point(X(t_end), Y(hdg_pct_thresh_)), kYellow, 1);
            cv::line(img, cv::Point(X(t_start), Y(-hdg_pct_thresh_)),
                     cv::Point(X(t_end), Y(-hdg_pct_thresh_)), kYellow, 1);
        }

        // 시계열
        std::vector<cv::Point> pts;
        pts.reserve(tb.size());
        for (size_t i = 0; i < tb.size(); ++i) {
            if (tb[i] < t_start) continue;
            pts.emplace_back(X(tb[i]), Y(yb[i]));
        }
        if (pts.size() >= 2)
            cv::polylines(img, pts, false, color, 2, cv::LINE_AA);

        // y축 눈금
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.2f", y_max);
        putText(img, buf, x0 + 4, y0 + 28, 0.4, kFgDim);
        std::snprintf(buf, sizeof(buf), "%.2f", y_min);
        putText(img, buf, x0 + 4, y0 + h - 12, 0.4, kFgDim);
    }

    void drawSpeedGraph(cv::Mat& img, int x0, int y0, int w, int h) {
        cv::Rect r(x0, y0, w, h);
        cv::rectangle(img, r, kBgMap, cv::FILLED);
        int cl = 8;
        cv::line(img, cv::Point(x0, y0), cv::Point(x0+cl, y0), kBorderHi, 2);
        cv::line(img, cv::Point(x0, y0), cv::Point(x0, y0+cl), kBorderHi, 2);
        cv::line(img, cv::Point(x0+w, y0+h), cv::Point(x0+w-cl, y0+h), kBorderHi, 2);
        cv::line(img, cv::Point(x0+w, y0+h), cv::Point(x0+w, y0+h-cl), kBorderHi, 2);
        cv::rectangle(img, r, kBorder, 1);
        putText(img, "[ VEL ]  ACTUAL=cyan  TARGET=amber", x0 + 10, y0 + 16, 0.42, kCyan, cv::FONT_HERSHEY_DUPLEX);
        if (t_buf_.size() < 2) return;

        double t_end = t_buf_.back();
        double t_start = std::max(0.0, t_end - 60.0);
        double y_max = 5.0;
        for (auto& v : v_buf_)   y_max = std::max(y_max, std::abs(v));
        for (auto& v : tgt_buf_) y_max = std::max(y_max, std::abs(v));
        y_max *= 1.15;
        double y_min = -std::max(2.0, y_max * 0.2);  // 후진 음수도 보이도록

        auto X = [&](double t) {
            double rng = std::max(1e-3, t_end - t_start);
            return x0 + 30 + (int)std::round((t - t_start) / rng * (w - 40));
        };
        auto Y = [&](double y) {
            double rng = std::max(1e-3, y_max - y_min);
            return y0 + h - 12 - (int)std::round((y - y_min) / rng * (h - 30));
        };
        cv::line(img, cv::Point(X(t_start), Y(0)), cv::Point(X(t_end), Y(0)),
                 cv::Scalar(180,180,180), 1, cv::LINE_AA);

        std::vector<cv::Point> p_v, p_t;
        for (size_t i = 0; i < t_buf_.size(); ++i) {
            if (t_buf_[i] < t_start) continue;
            p_v.emplace_back(X(t_buf_[i]), Y(v_buf_[i]));
            p_t.emplace_back(X(t_buf_[i]), Y(tgt_buf_[i]));
        }
        if (p_t.size() >= 2) cv::polylines(img, p_t, false, kAmber, 1, cv::LINE_AA);
        if (p_v.size() >= 2) cv::polylines(img, p_v, false, kCyan,  2, cv::LINE_AA);

        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.0f", y_max);
        putText(img, buf, x0 + 4, y0 + 28, 0.4, kFgDim);
        std::snprintf(buf, sizeof(buf), "%.0f", y_min);
        putText(img, buf, x0 + 4, y0 + h - 12, 0.4, kFgDim);
    }

    static void putText(cv::Mat& img, const std::string& s, int x, int y,
                        double scale, const cv::Scalar& color,
                        int font = cv::FONT_HERSHEY_SIMPLEX, int thickness = 1) {
        cv::putText(img, s, cv::Point(x, y), font, scale, color, thickness, cv::LINE_AA);
    }

    // ── 장애물 / 회피 콜백 ─────────────────────────────────────
    void cbObj(const morai_msgs::ObjectStatusList::ConstPtr& msg) {
        std::lock_guard<std::mutex> lk(mu_);
        obstacles_.clear();
        auto add = [&](const auto& list) {
            for (const auto& o : list) {
                Obstacle ob;
                ob.x = o.position.x; ob.y = o.position.y;
                ob.sx = o.size.x;    ob.sy = o.size.y;
                ob.heading = o.heading;
                obstacles_.push_back(ob);
            }
        };
        add(msg->npc_list);
        add(msg->pedestrian_list);
        add(msg->obstacle_list);
    }
    void cbAvoid(const std_msgs::Float64::ConstPtr& msg) {
        std::lock_guard<std::mutex> lk(mu_);
        avoid_offset_ = msg->data;
    }
    // planner가 실제 발행하는 live 경로 — 차가 이걸 따라감. 대시보드도 이거 하나만 표시.
    void cbLivePath(const nav_msgs::Path::ConstPtr& msg) {
        std::lock_guard<std::mutex> lk(mu_);
        live_x_.clear(); live_y_.clear();
        for (const auto& ps : msg->poses) {
            live_x_.push_back(ps.pose.position.x);
            live_y_.push_back(ps.pose.position.y);
        }
        live_stamp_ = ros::Time::now();
    }
    struct Obstacle { double x, y, sx, sy, heading; };

    // ── 데이터 ────────────────────────────────────────────────
    ros::NodeHandle nh_;
    ros::Subscriber sub_perf_, sub_status_, sub_ego_, sub_obj_, sub_avoid_, sub_livepath_, sub_mode_;
    std::vector<Obstacle> obstacles_;
    std::string avoid_mode_ = "직진";   // planner 현재 모드(추월/회피/직진)
    double avoid_offset_ = 0.0;
    std::vector<double> live_x_, live_y_;   // planner live 경로 (차 실제 추종경로)
    ros::Time live_stamp_;
    std::string path_file_;
    double window_m_ = 18.0;
    double hdg_pct_thresh_ = 5.0;

    std::mutex mu_;
    std::deque<double> t_buf_, cte_buf_, hdg_buf_, v_buf_, tgt_buf_;
    int total_ = 0;
    double cte_sq_ = 0.0, hdg_sq_ = 0.0;
    // 윈도우드 pct: 최근 N틱 sum/N (커브 통과 후 회복)
    std::deque<int8_t> win_f10_, win_f20_, win_f30_, win_f50_, win_fhd_;
    int sum_w10_ = 0, sum_w20_ = 0, sum_w30_ = 0, sum_w50_ = 0, sum_whd_ = 0;
    int gear_switch_n_ = 0, recov_n_ = 0, nmpc_n_ = 0;
    std::string cur_mode_ = "—";
    std::string cur_gear_ = "D";
    std::string ctrl_     = "LTV";
    bool   is_low_speed_ = false;
    double t0_ = 0.0;

    bool   ego_rcvd_ = false;
    double ego_x_ = 0.0, ego_y_ = 0.0, ego_yaw_ = 0.0, ego_v_ = 0.0;
    std::deque<std::pair<double,double>> trail_;

    std::vector<double> wp_d_x_, wp_d_y_, wp_r_x_, wp_r_y_;
    double path_min_x_ = 0, path_max_x_ = 0, path_min_y_ = 0, path_max_y_ = 0;
    bool   follow_vehicle_ = false;   // false=path 전체 fit, true=차량 중심
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_dashboard_cpp");
    ros::NodeHandle nh("~");
    Dashboard dash(nh);
    dash.spin();
    return 0;
}
