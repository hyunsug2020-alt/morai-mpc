#include "moraimpc/path_follower.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>

#include <jsoncpp/json/json.h>

namespace moraimpc {

static double wrapAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ── 생성자 ─────────────────────────────────────────────────────────
PathFollower::PathFollower(ros::NodeHandle& nh) {
    std::string path_file;
    double target_vel = 20.0;
    nh.param<std::string>("path_file",   path_file,   "/tmp/waypoints.json");
    nh.param<double>     ("target_vel",  target_vel,  20.0);

    cfg_.target_vel = target_vel / 3.6;   // km/h → m/s
    nh.param<std::string>("log_file", log_file_, "/tmp/mpc_log.json");
    log_t0_ = ros::Time::now();
    log_recs_.reserve(20000);
    loadPath(path_file);

    model_  = std::make_unique<LTVModel>(cfg_);
    cost_   = std::make_unique<LTVCost>(cfg_);
    solver_ = std::make_unique<LTVSolver>(cfg_);

    ego_sub_    = nh.subscribe("/Ego_topic",       1, &PathFollower::egoCallback,  this);
    ctrl_pub_   = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0",       1);
    perf_pub_   = nh.advertise<std_msgs::Float32MultiArray>("/mpc_performance", 1);
    status_pub_ = nh.advertise<std_msgs::String>   ("/mpc_status",       1);

    timer_ = nh.createTimer(ros::Duration(cfg_.Ts), &PathFollower::controlLoop, this);
    prev_cmd_time_ = ros::Time::now();

    ROS_INFO("[PathFollower] LTV-MPC 시작 — 목표속도: %.1f km/h, 웨이포인트: %zu개",
             target_vel, wp_x_.size());
    ROS_INFO("[PathFollower] 주행 기록 → %s", log_file_.c_str());
}

// ── 소멸자 ─────────────────────────────────────────────────────────
PathFollower::~PathFollower() {
    flushLog();
    // 주행 종료 후 분석 이미지 자동 생성
    if (!log_file_.empty()) {
        std::string script = log_file_;
        // logs/mpc_log.json -> scripts/analyze_mpc.py 경로 추정
        auto pos = script.rfind("/logs/");
        if (pos != std::string::npos) {
            std::string pkg_dir = script.substr(0, pos);
            std::string analyze = pkg_dir + "/scripts/analyze_mpc.py";
            std::string cmd = "python3 " + analyze + " " + log_file_ + " &";
            std::system(cmd.c_str());
            ROS_INFO("[PathFollower] 분석 이미지 생성 중 -> logs/mpc_analysis.png");
        }
    }
}

// ── 주행 기록 저장 ──────────────────────────────────────────────────
void PathFollower::flushLog() {
    if (log_recs_.empty()) return;
    Json::Value root(Json::objectValue);
    root["total_records"] = static_cast<int>(log_recs_.size());
    Json::Value arr(Json::arrayValue);
    for (auto& r : log_recs_) arr.append(r);
    root["records"] = arr;

    std::ofstream ofs(log_file_);
    Json::StreamWriterBuilder wb;
    wb["indentation"] = " ";
    ofs << Json::writeString(wb, root);
    ROS_INFO("[PathFollower] 기록 저장 완료: %d ticks -> %s",
             static_cast<int>(log_recs_.size()), log_file_.c_str());
}

// ── 경로 로드 ───────────────────────────────────────────────────────
void PathFollower::loadPath(const std::string& file) {
    std::ifstream ifs(file);
    Json::Value root;
    Json::Reader reader;
    if (!reader.parse(ifs, root)) {
        ROS_ERROR("[PathFollower] 경로 파일 파싱 실패: %s", file.c_str());
        return;
    }

    const Json::Value wps = root["waypoints"];
    int n = static_cast<int>(wps.size());
    wp_x_.resize(n); wp_y_.resize(n); wp_h_.resize(n); wp_k_.resize(n, 0.0);

    for (int i = 0; i < n; ++i) {
        wp_x_[i] = wps[i]["x"].asDouble();
        wp_y_[i] = wps[i]["y"].asDouble();
        // heading 필드는 무시 — 좌표계 불일치로 잘못된 값이 저장될 수 있음
        // 실제 경로 접선각을 위치 좌표에서 직접 계산
    }

    // ── 경로 접선각: 위치 좌표 기반 기하학적 계산 ──────────────────
    // JSON heading 필드와 atan2 간의 좌표계 불일치를 완전히 우회
    if (n >= 2) {
        wp_h_[0] = std::atan2(wp_y_[1]   - wp_y_[0],   wp_x_[1]   - wp_x_[0]);
        for (int i = 1; i < n - 1; ++i) {
            wp_h_[i] = std::atan2(wp_y_[i+1] - wp_y_[i-1],
                                  wp_x_[i+1] - wp_x_[i-1]);
        }
        wp_h_[n-1] = std::atan2(wp_y_[n-1] - wp_y_[n-2],
                                 wp_x_[n-1] - wp_x_[n-2]);
    }

    // 참조 곡률 (중앙 차분)
    for (int i = 1; i < n - 1; ++i) {
        double dh = wrapAngle(wp_h_[i+1] - wp_h_[i-1]);
        double ds = std::hypot(wp_x_[i+1] - wp_x_[i-1],
                               wp_y_[i+1] - wp_y_[i-1]);
        wp_k_[i] = (ds > 1e-6) ? (dh / ds) : 0.0;
    }
    wp_k_[0]     = wp_k_[1];
    wp_k_[n - 1] = wp_k_[n - 2];

    // 웨이포인트 평균 간격
    if (n >= 2) {
        double total = 0.0;
        for (int i = 1; i < n; ++i)
            total += std::hypot(wp_x_[i] - wp_x_[i-1], wp_y_[i] - wp_y_[i-1]);
        wp_spacing_ = total / (n - 1);
    }
    ROS_INFO("[PathFollower] 경로 로드: %d포인트, 평균간격 %.3fm", n, wp_spacing_);
}

// ── Ego 콜백 ───────────────────────────────────────────────────────
void PathFollower::egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
    cur_x_   = msg->position.x;
    cur_y_   = msg->position.y;
    cur_yaw_ = wrapAngle(msg->heading * M_PI / 180.0);
    cur_v_   = std::hypot(msg->velocity.x, msg->velocity.y);
    ego_rcvd_ = true;
}

// ── 최근접 탐색: forward-filter + jump suppression ─────────────────
// local_path_pub_cpp.cpp 패턴 기반
PathFollower::NearResult PathFollower::findNearest() {
    const int n   = static_cast<int>(wp_x_.size());
    const double hx = std::cos(cur_yaw_);
    const double hy = std::sin(cur_yaw_);

    double min_d  = std::numeric_limits<double>::max();
    int    closest = nearest_idx_;

    // ── Step1: 윈도우 탐색 (초기화 후) ────────────────────────────
    if (search_init_) {
        int w_s = std::max(0,     nearest_idx_ - 10);
        int w_e = std::min(n - 1, nearest_idx_ + kSearchWindow);
        for (int i = w_s; i <= w_e; ++i) {
            double dx   = wp_x_[i] - cur_x_;
            double dy   = wp_y_[i] - cur_y_;
            double dot  = dx * hx + dy * hy;
            if (dot < kDotThreshold) continue;   // 후방 점 제외
            double dist = std::hypot(dx, dy);
            if (dist < min_d) { min_d = dist; closest = i; }
        }
    }

    // ── Step2: 전역 탐색 (미초기화 or dist > kRecovDist) ─────────
    if (!search_init_ || min_d > kRecovDist) {
        double best = std::numeric_limits<double>::max();
        int    best_idx = nearest_idx_;
        bool   found_fwd = false;
        for (int i = 0; i < n; ++i) {
            double dx   = wp_x_[i] - cur_x_;
            double dy   = wp_y_[i] - cur_y_;
            double dot  = dx * hx + dy * hy;
            double dist = std::hypot(dx, dy);
            if (dot > 0.0) {
                if (!found_fwd || dist < best) { best = dist; best_idx = i; found_fwd = true; }
            } else if (!found_fwd && dist < best) {
                best = dist; best_idx = i;
            }
        }
        closest = best_idx;
        min_d   = best;
        search_init_ = true;
        ROS_WARN_THROTTLE(2.0, "[PathFollower] 전역 재초기화 → idx=%d, dist=%.1fm",
                          closest, min_d);
    }

    // ── Step3: jump suppression ────────────────────────────────────
    {
        int delta = closest - nearest_idx_;
        if (delta < 0) {
            closest = nearest_idx_;                           // 역방향 점프 무조건 차단
        } else if (delta > kMaxIndexStep && min_d <= kRecovDist) {
            closest = nearest_idx_ + kMaxIndexStep;          // 과도한 전방 점프는 정상 추종만 제한
        }
    }
    nearest_idx_ = std::min(closest, n - 1);

    // ── 경로 접선 및 오차 계산 ─────────────────────────────────────
    int ni   = nearest_idx_;
    int ni1  = std::min(ni + 1, n - 1);
    double path_yaw = std::atan2(wp_y_[ni1] - wp_y_[ni],
                                  wp_x_[ni1] - wp_x_[ni]);
    double rx = cur_x_ - wp_x_[ni];
    double ry = cur_y_ - wp_y_[ni];
    double signed_cte  = -std::sin(path_yaw) * rx + std::cos(path_yaw) * ry;
    double heading_err =  wrapAngle(cur_yaw_ - path_yaw);

    return { nearest_idx_, min_d, signed_cte, heading_err, path_yaw };
}

// ── 조향 변화율 제한 ───────────────────────────────────────────────
double PathFollower::steerRateLimit(double steer_deg, double dt) {
    if (!cmd_init_) { prev_steer_ = steer_deg; cmd_init_ = true; return steer_deg; }
    double max_d = max_steer_rate_ * dt;
    steer_deg = std::clamp(steer_deg, prev_steer_ - max_d, prev_steer_ + max_d);
    return std::clamp(steer_deg, -max_steer_deg_, max_steer_deg_);
}

// ── 속도 시그모이드 ────────────────────────────────────────────────
double PathFollower::velocitySigmoid(double v_tgt, double dt) {
    if (!v_sig_init_) { v_sig_ = v_tgt; v_sig_init_ = true; return v_sig_; }
    double tau   = (v_tgt > v_sig_) ? sig_tau_up_ : sig_tau_down_;
    double alpha = 1.0 - std::exp(-dt / std::max(tau, 1e-3));
    v_sig_ += alpha * (v_tgt - v_sig_);
    return std::max(0.0, v_sig_);
}

// ── 제어 루프 ──────────────────────────────────────────────────────
void PathFollower::controlLoop(const ros::TimerEvent&) {
    if (!ego_rcvd_ || wp_x_.empty()) return;

    ros::Time now = ros::Time::now();
    double dt = (now - prev_cmd_time_).toSec();
    if (!std::isfinite(dt) || dt <= 1e-4) dt = cfg_.Ts;
    prev_cmd_time_ = now;

    auto t_start = ros::WallTime::now();

    // ── 1. 최근접 탐색 ─────────────────────────────────────────────
    NearResult near = findNearest();
    const int n = static_cast<int>(wp_x_.size());

    // ── 기본 로그 레코드 구성 ──────────────────────────────────────
    Json::Value rec(Json::objectValue);
    rec["t"]           = (now - log_t0_).toSec();
    rec["tick"]        = log_tick_;
    rec["x"]           = cur_x_;
    rec["y"]           = cur_y_;
    rec["yaw_deg"]     = cur_yaw_ * (180.0 / M_PI);
    rec["v_kmh"]       = cur_v_ * 3.6;
    rec["idx"]         = nearest_idx_;
    rec["n_wp"]        = n;
    rec["dist"]        = near.dist;
    rec["cte"]         = near.signed_cte;
    rec["hdg_err_deg"] = near.heading_err * (180.0 / M_PI);
    rec["path_yaw_deg"]= near.path_yaw    * (180.0 / M_PI);

    // ── 경로 종료 판단 ─────────────────────────────────────────────
    if (nearest_idx_ >= n - 2) {
        double d_end = std::hypot(wp_x_.back() - cur_x_, wp_y_.back() - cur_y_);
        if (d_end < 2.0) {
            ROS_INFO_ONCE("[PathFollower] 경로 종료 — 정지");
            rec["mode"]      = "STOP";
            rec["steer_cmd"] = 0.0;
            rec["v_cmd"]     = 0.0;
            log_recs_.push_back(rec);
            publishCmd(0.0, 0.0);
            return;
        }
    }

    double steer_deg = 0.0;
    double v_cmd     = cfg_.target_vel * 3.6;

    // ══════════════════════════════════════════════════════════════
    // 경로 이탈 복구 모드 (히스테리시스 적용)
    //   진입: dist > kRecovDist(0.8m) OR |hdg| > kRecovHdgThresh(25°)
    //   탈출: dist < kRecovDistExit(0.70m) 단독 기준
    //   히스테리시스로 MPC↔RECOV 고주파 전환(2.3회/s) 억제
    // ══════════════════════════════════════════════════════════════
    bool enter_recov = (near.dist > kRecovDist) ||
                       (std::abs(near.heading_err) > kRecovHdgThresh);
    // dist 단독 기준: AND hdg 조건 제거 (dist<0.35 AND hdg<12° 동시 만족 거의 불가능)
    bool exit_recov  = (near.dist < kRecovDistExit);

    // exit 우선: dist < kRecovDistExit이면 hdg가 커도 탈출 (enter else-if exit 버그 수정)
    if (exit_recov)        in_recov_ = false;
    else if (enter_recov)  in_recov_ = true;
    // 그 사이 구간은 이전 상태 유지 (히스테리시스)

    bool need_recov = in_recov_;

    if (need_recov) {
        // CTE 크기에 따라 lookahead 적응: 크게 이탈할수록 짧게 → 더 강하게 꺾음
        double la_dist = std::max(4.0, 10.0 - near.dist * 2.0);
        int lookahead = std::max(1, static_cast<int>(std::round(la_dist / wp_spacing_)));
        int target_idx = std::min(nearest_idx_ + lookahead, n - 1);

        double dx_to  = wp_x_[target_idx] - cur_x_;
        double dy_to  = wp_y_[target_idx] - cur_y_;
        double alpha  = wrapAngle(std::atan2(dy_to, dx_to) - cur_yaw_);
        steer_deg = std::clamp(alpha * (180.0 / M_PI), -max_steer_deg_, max_steer_deg_);
        double facing = std::max(0.0, std::cos(alpha));
        v_cmd = std::min(kRecovMaxVel,
                         std::max(5.0, cfg_.target_vel * 3.6 * kRecovDist
                                           / std::max(near.dist, kRecovDist)
                                       + cfg_.target_vel * 3.6 * 0.3 * facing));
        prev_steer_ = steer_deg;
        prev_was_recov_ = true;
        v_cmd = velocitySigmoid(v_cmd, dt);

        rec["mode"]        = need_recov && (std::abs(near.heading_err) > kRecovHdgThresh)
                                 && !(near.dist > kRecovDist) ? "HDG_RECOV" : "RECOV";
        rec["alpha_deg"]   = alpha * (180.0 / M_PI);
        rec["target_idx"]  = target_idx;
        rec["steer_cmd"]   = steer_deg;
        rec["v_cmd"]       = v_cmd;
        log_recs_.push_back(rec);

        publishCmd(v_cmd, steer_deg);
        ROS_WARN_THROTTLE(1.0,
            "[PathFollower][RECOV] idx=%d→tgt=%d dist=%.1fm hErr=%.1f° α=%.1f° steer=%.1f°",
            nearest_idx_, target_idx, near.dist,
            near.heading_err * (180.0 / M_PI), alpha * (180.0 / M_PI), steer_deg);

        if (++log_tick_ % 200 == 0) flushLog();
        return;
    }

    // ══════════════════════════════════════════════════════════════
    // 정상 LTV-MPC 추종
    // ══════════════════════════════════════════════════════════════

    // ── 2. Frenet 초기 상태 구성 ──────────────────────────────────
    double theta_ref = wp_h_[nearest_idx_];
    double kappa_ref = wp_k_[nearest_idx_];

    double dx_ref = cur_x_ - wp_x_[nearest_idx_];
    double dy_ref = cur_y_ - wp_y_[nearest_idx_];
    double dr     = -std::sin(theta_ref) * dx_ref + std::cos(theta_ref) * dy_ref;

    double delta_theta_raw = wrapAngle(cur_yaw_ - theta_ref);

    // ── Stanley 교정: MPC heading 목표에 CTE 비례 오프셋 추가 ──────
    // 목표: delta_theta_eff = raw_hErr - atan(k_s * CTE / v)
    //   CTE>0 (우측) → 교정항 음수 → delta_theta 더 음수 → MPC가 kappa>0 적용 → CTE 감소 ✓
    //   CTE<0 (좌측) → 교정항 양수 → delta_theta 더 양수 → MPC가 kappa<0 적용 → CTE 증가 ✓
    // 부호 주의: -atan(k * signed_cte / v)
    double stanley_corr = -std::atan2(k_stanley_ * near.signed_cte,
                                      std::max(cur_v_, 0.5));
    stanley_corr = std::clamp(stanley_corr, -0.52, 0.52);   // ±30° 상한
    double delta_theta = delta_theta_raw + stanley_corr;

    constexpr double kDeltaThetaMax = 0.524;   // ~30° (Stanley 포함해서 넓힘)
    bool kappa_reset = false;
    if (std::abs(delta_theta) > kDeltaThetaMax) {
        delta_theta = std::clamp(delta_theta, -kDeltaThetaMax, kDeltaThetaMax);
        kappa_reset = true;
        current_kappa_ = kappa_ref;  // 헤딩 포화 시 kappa도 참조값으로 리셋
    }
    double theta_aligned = theta_ref + delta_theta;

    Eigen::VectorXd x0(kNx);
    x0 << dr, theta_aligned, current_kappa_, theta_ref, kappa_ref;

    // ── 3. 배치 행렬 + 참조 곡률 입력 ────────────────────────────
    double v_curr = std::max(cur_v_, 0.5);
    std::vector<double> v_profile(cfg_.N);
    
    // 미래 곡률 기반 속도 프로파일 생성 (Velocity Planning)
    {
        double v_tgt = cfg_.target_vel;
        const double kCurveAlpha = 2.5; // 곡률에 따른 감속 계수
        int step_idx = nearest_idx_;
        for (int i = 0; i < cfg_.N; ++i) {
            double v_k = v_tgt;
            // 0.5초(약 10스텝) 앞의 곡률까지 고려하여 미리 감속
            int lookahead_steps = 10;
            double max_k = 0.0;
            for (int j = 0; j < lookahead_steps; ++j) {
                int ki = std::min(step_idx + j, n - 1);
                max_k = std::max(max_k, std::abs(wp_k_[ki]));
            }
            v_k = v_tgt / (1.0 + kCurveAlpha * max_k);
            v_profile[i] = std::max(1.5, v_k); // 최소 속도 보장
            
            // 예측 스텝당 이동 인덱스 계산
            int idx_step = std::max(1, static_cast<int>(std::round(v_profile[i] * cfg_.Ts / wp_spacing_)));
            step_idx = std::min(step_idx + idx_step, n - 1);
        }
    }

    Eigen::MatrixXd A_bar, B_bar, E_bar;
    model_->buildBatchMatrices(v_profile, A_bar, B_bar, E_bar);

    Eigen::VectorXd z_bar = Eigen::VectorXd::Zero(cfg_.N);
    int curr_step_idx = nearest_idx_;
    for (int i = 0; i < cfg_.N; ++i) {
        int idx_step = std::max(1, static_cast<int>(std::round(v_profile[i] * cfg_.Ts / wp_spacing_)));
        int ki  = std::min(curr_step_idx + idx_step, n - 1);
        z_bar(i) = (wp_k_[ki] - wp_k_[curr_step_idx]) / cfg_.Ts;
        curr_step_idx = ki;
    }

    // ── 4. QP 구성 & 풀기 ─────────────────────────────────────────
    Eigen::SparseMatrix<double> P;
    Eigen::VectorXd q_vec;
    cost_->buildQPObjective(x0, z_bar, A_bar, B_bar, E_bar, v_profile, P, q_vec);

    Eigen::SparseMatrix<double> A_cons(cfg_.N, cfg_.N);
    A_cons.setIdentity();
    Eigen::VectorXd l_cons = Eigen::VectorXd::Constant(cfg_.N, cfg_.u_min);
    Eigen::VectorXd u_cons = Eigen::VectorXd::Constant(cfg_.N, cfg_.u_max);

    Eigen::VectorXd solution;
    bool ok = solver_->solve(P, q_vec, A_cons, l_cons, u_cons, solution);
    double solve_ms = (ros::WallTime::now() - t_start).toSec() * 1000.0;

    double kappa_dot_sol = 0.0;
    if (!ok || solution.size() == 0) {
        ROS_WARN_THROTTLE(1.0, "[PathFollower] MPC 풀이 실패 dist=%.2f", near.dist);
        rec["mode"]    = "NORMAL";
        rec["mpc_ok"]  = false;
        rec["steer_cmd"] = 0.0;
        rec["v_cmd"]     = 0.0;
        log_recs_.push_back(rec);
        if (++log_tick_ % 200 == 0) flushLog();
        publishCmd(0.0, 0.0);
        return;
    }
    kappa_dot_sol = solution[0];

    // ── 5. kappa 초기화 및 상태 업데이트 ────────────────────────────
    if (prev_was_recov_) {
        current_kappa_ = kappa_ref;
        prev_was_recov_ = false;
    } else if (cmd_init_) {
        current_kappa_ = cfg_.kappa_gain * std::tan(prev_steer_ * M_PI / 180.0) / cfg_.L;
    }
    double kappa_before = current_kappa_;
    
    current_kappa_ += cfg_.kappa_gain * kappa_dot_sol * cfg_.Ts;
    current_kappa_  = std::clamp(current_kappa_, cfg_.kappa_gain * cfg_.kappa_min, 
                                                 cfg_.kappa_gain * cfg_.kappa_max);

    // ── 5b. 헤딩 오버슈트 방지 ────────────────────────────────────────
    {
        const double kHdgCapRad = 0.35;  // ~20°
        double hdg_abs = std::abs(near.heading_err);
        bool heading_toward_path = (near.signed_cte * near.heading_err < 0.0);
        if (hdg_abs > kHdgCapRad && heading_toward_path) {
            current_kappa_ *= 0.80;
        }
    }

    double cmd_kappa = current_kappa_ / cfg_.kappa_gain;
    steer_deg = std::atan(cmd_kappa * cfg_.L) * (180.0 / M_PI);
    double steer_after_kappa = steer_deg;

    // ── 6. 후처리 A: 속도 결정 (프로파일의 첫 번째 값 사용) ──────
    v_cmd = v_profile[0] * 3.6; // m/s -> km/h

    // ── 6. 후처리 B: 오버슈트 방지 ───────────────────────────────
    double steer_rad = steer_deg * M_PI / 180.0;
    if (std::abs(near.signed_cte) > overshoot_dist_ &&
        std::abs(near.signed_cte) < 0.6 &&
        (steer_rad * near.signed_cte) > 0.0) {
        steer_deg *= overshoot_damp_;
    }

    // ── 6. 후처리 C: 진동 감지 → 강력한 조향 댐핑 ────────────────
    bool osc_damped = false;
    if (has_prev_errors_) {
        bool cte_flip =
            (std::max(std::abs(near.signed_cte), std::abs(prev_cte_)) > osc_cte_db_) &&
            (near.signed_cte * prev_cte_ < 0.0);
        bool hdg_flip =
            (std::max(std::abs(near.heading_err), std::abs(prev_hdg_)) > osc_hdg_db_) &&
            (near.heading_err * prev_hdg_ < 0.0);
        if (cte_flip || hdg_flip) { 
            osc_damped = true; 
            steer_deg *= osc_damp_; // 진동 발생 시 조향 출력을 직접 댐핑
            current_kappa_ *= osc_damp_; // 상태값도 동기화
        }
    }

    // ── 6. 후처리 D: 경로 근접 시 추가 안정화 ───────────────────
    if (std::abs(near.signed_cte)  < near_cte_thresh_ &&
        std::abs(near.heading_err) < near_hdg_thresh_) {
        v_cmd *= near_v_scale_;
        steer_deg *= near_steer_damp_;
    }

    prev_cte_        = near.signed_cte;
    prev_hdg_        = near.heading_err;
    has_prev_errors_ = true;

    // ── 6. 후처리 E: 조향 변화율 제한 + 속도 시그모이드 ──────────
    double steer_before_ratelim = steer_deg;
    steer_deg   = steerRateLimit(steer_deg, dt);
    prev_steer_ = steer_deg;
    v_cmd       = velocitySigmoid(v_cmd, dt);

    // ── 로그 레코드 완성 ──────────────────────────────────────────
    rec["mode"]                = "NORMAL";
    rec["mpc_ok"]              = true;
    rec["theta_ref_deg"]       = theta_ref  * (180.0 / M_PI);
    rec["path_yaw_deg"]        = near.path_yaw * (180.0 / M_PI);   // 기하 접선 (중복 기록)
    rec["kappa_ref"]           = kappa_ref;
    rec["dr"]                  = dr;
    rec["delta_theta_raw_deg"] = delta_theta_raw * (180.0 / M_PI);
    rec["delta_theta_sat_deg"] = delta_theta     * (180.0 / M_PI);
    rec["kappa_reset"]         = kappa_reset;
    rec["kappa_before"]        = kappa_before;
    rec["kappa_dot_sol"]       = kappa_dot_sol;
    rec["current_kappa"]       = current_kappa_;
    rec["steer_kappa_deg"]     = steer_after_kappa;        // kappa→steer 직후
    rec["steer_pre_rl_deg"]    = steer_before_ratelim;     // rate limit 직전
    rec["steer_cmd"]           = steer_deg;                // MORAI 전송값 (양수=좌)
    rec["v_cmd"]               = v_cmd;
    rec["osc_damped"]          = osc_damped;
    rec["solve_ms"]            = solve_ms;
    rec["idx_per_step"]        = std::max(1, static_cast<int>(std::round(v_profile[0] * cfg_.Ts / wp_spacing_)));
    log_recs_.push_back(rec);

    publishCmd(v_cmd, steer_deg);

    // ── 퍼포먼스 퍼블리시 ─────────────────────────────────────────
    std_msgs::Float32MultiArray perf;
    perf.data = { static_cast<float>(near.dist),
                  static_cast<float>(solve_ms),
                  static_cast<float>(std::abs(near.signed_cte)) };
    perf_pub_.publish(perf);

    std_msgs::String status;
    status.data = "OPTIMAL";
    status_pub_.publish(status);

    ROS_INFO_THROTTLE(1.0,
        "[PathFollower] idx=%d/%d dist=%.2f CTE=%.2f hErr=%.1f° steer=%.1f° solve=%.1fms",
        nearest_idx_, n, near.dist, near.signed_cte,
        near.heading_err * (180.0 / M_PI), steer_deg, solve_ms);

    if (++log_tick_ % 200 == 0) flushLog();
}

// ── 명령 발행 ───────────────────────────────────────────────────────
void PathFollower::publishCmd(double vel_kmh, double steer_deg) {
    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 2;
    cmd.velocity = std::max(0.0, vel_kmh);
    cmd.steering = steer_deg;    // MORAI: 양수=좌회전 (실측 확인)
    ctrl_pub_.publish(cmd);
}

}  // namespace moraimpc

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower_node");
    ros::NodeHandle nh("~");

    // argv로도 path_file / target_vel 오버라이드 가능
    if (argc >= 2) nh.setParam("path_file",  std::string(argv[1]));
    if (argc >= 3) nh.setParam("target_vel", std::stod(argv[2]));

    moraimpc::PathFollower node(nh);
    ros::spin();
    return 0;
}
