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
    nh.param<double>("reverse_max_vel", reverse_max_vel_kmh_, 2.0);
    nh.param<double>("low_speed_thresh_kmh", low_speed_thresh_kmh_, 4.0);
    nh.param<double>("pre_gear_change_dist_m", pre_gear_change_dist_m_, 5.0);
    nh.param<std::string>("log_file", log_file_, "/tmp/mpc_log.json");
    log_t0_ = ros::Time::now();
    log_recs_.reserve(20000);
    loadPath(path_file);

    model_  = std::make_unique<LTVModel>(cfg_);
    cost_   = std::make_unique<LTVCost>(cfg_);
    solver_ = std::make_unique<LTVSolver>(cfg_);

    // NMPC (legacy 단순 버전)
    nmpc_cfg_.Ts = cfg_.Ts;
    nmpc_cfg_.L  = cfg_.L;
    nmpc_       = std::make_unique<NMPCController>(nmpc_cfg_);

    // RTI-NMPC (저속·후진 전용)
    rti_cfg_.Ts        = cfg_.Ts;
    rti_cfg_.wheelbase = cfg_.L;
    rti_cfg_.target_velocity = reverse_max_vel_kmh_ / 3.6;  // 매 tick 부호 갱신
    rti_nmpc_  = std::make_unique<RTINMPCController>(rti_cfg_);

    nh.param<bool>("avoidance_enabled", avoidance_enabled_, false);
    nh.param<bool>("force_nmpc", force_nmpc_, false);  // true면 항상 RTI-NMPC 사용 (고속 튜닝용)

    // NMPC stage 제약 (방안 B) — launch 파라미터
    nh.param<bool>  ("nmpc_obs_enable",      rti_cfg_.obs_enable,      false);
    nh.param<double>("nmpc_obs_safe_margin", rti_cfg_.obs_safe_margin, 1.5);
    nh.param<double>("nmpc_obs_active_dist", rti_cfg_.obs_active_dist, 30.0);
    nh.param<int>   ("nmpc_obs_max_count",   rti_cfg_.obs_max_count,   5);
    nh.param<int>   ("nmpc_obs_skip_first",  rti_cfg_.obs_skip_first,  1);
    nh.param<double>("nmpc_w_obs_slack_quad", rti_cfg_.w_obs_slack_quad, 1e5);
    nh.param<double>("nmpc_w_obs_slack_lin",  rti_cfg_.w_obs_slack_lin,  1e3);
    rti_nmpc_->setConfig(rti_cfg_);

    ego_sub_    = nh.subscribe("/Ego_topic",       1, &PathFollower::egoCallback,  this);
    // NMPC obs 활성 또는 LTV corridor 활성 시 /Object_topic 구독
    if (avoidance_enabled_ || rti_cfg_.obs_enable) {
        obj_sub_   = nh.subscribe("/Object_topic", 1, &PathFollower::objectCallback,  this);
        ROS_INFO("[PathFollower] 회피 모드 ON — /Object_topic 구독 (LTV corridor=%d, NMPC stage=%d)",
                 avoidance_enabled_ ? 1 : 0, rti_cfg_.obs_enable ? 1 : 0);
    }
    ctrl_pub_   = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0",       1);
    gear_srv_   = nh.serviceClient<morai_msgs::MoraiEventCmdSrv>("/Service_MoraiEventCmd");
    event_pub_  = nh.advertise<morai_msgs::EventInfo>("/InsnControl", 1);  // topic fallback
    perf_pub_   = nh.advertise<std_msgs::Float32MultiArray>("/mpc_performance", 1);
    status_pub_ = nh.advertise<std_msgs::String>   ("/mpc_status",       1);

    timer_ = nh.createTimer(ros::Duration(cfg_.Ts), &PathFollower::controlLoop, this);
    prev_cmd_time_ = ros::Time::now();

    ROS_INFO("[PathFollower] Mobility-Structure MPC 시작 — 목표속도: %.1f km/h", target_vel);
}

PathFollower::~PathFollower() {
    flushLog();
}

void PathFollower::flushLog() {
    if (log_recs_.empty()) return;
    Json::Value root(Json::objectValue);
    root["total_records"] = static_cast<int>(log_recs_.size());
    Json::Value arr(Json::arrayValue);
    for (auto& r : log_recs_) arr.append(r);
    root["records"] = arr;

    // Summary statistics
    double cte_sq = 0.0, hdg_sq = 0.0, vel_sq = 0.0;
    int cnt = 0;
    for (auto& r : log_recs_) {
        if (!r.isMember("cte")) continue;
        double c = r["cte"].asDouble();
        double h = r["hdg_err_deg"].asDouble();
        cte_sq += c * c;
        hdg_sq += h * h;
        if (r.isMember("vel_error")) {
            double ve = r["vel_error"].asDouble();
            vel_sq += ve * ve;
        }
        ++cnt;
    }
    if (cnt > 0) {
        Json::Value stats(Json::objectValue);
        stats["cte_rmse"]     = std::sqrt(cte_sq / cnt);
        stats["hdg_rmse_deg"] = std::sqrt(hdg_sq / cnt);
        stats["vel_err_rmse"] = std::sqrt(vel_sq / cnt);
        stats["num_records"]  = cnt;
        root["summary"] = stats;
    }

    std::ofstream ofs(log_file_);
    Json::StreamWriterBuilder wb; wb["indentation"] = " ";
    ofs << Json::writeString(wb, root);
}

void PathFollower::loadPath(const std::string& file) {
    std::ifstream ifs(file);
    Json::Value root; Json::Reader reader;
    if (!reader.parse(ifs, root)) return;
    const Json::Value wps = root["waypoints"];
    int n = static_cast<int>(wps.size());
    wp_x_.resize(n); wp_y_.resize(n); wp_h_.resize(n); wp_k_.resize(n, 0.0);
    wp_gear_.resize(n, 1);  // 기본값: 전진(D)
    for (int i = 0; i < n; ++i) {
        wp_x_[i] = wps[i]["x"].asDouble();
        wp_y_[i] = wps[i]["y"].asDouble();
        if (wps[i].isMember("gear")) {
            std::string g = wps[i]["gear"].asString();
            wp_gear_[i] = (g == "R" || g == "r") ? -1 : 1;
        }
    }
    // ── 기어 세그먼트 테이블 컴파일 (heading/곡률 계산보다 먼저) ────
    // 같은 gear 연속 구간을 [start_idx, end_idx_inclusive]로 묶음
    gear_segments_.clear();
    if (n > 0) {
        int s = 0;
        for (int i = 1; i < n; ++i) {
            if (wp_gear_[i] != wp_gear_[s]) { gear_segments_.emplace_back(s, i - 1); s = i; }
        }
        gear_segments_.emplace_back(s, n - 1);
        cur_gear_ = wp_gear_[0];
        cur_segment_ = 0;
    }
    // ── 헤딩 / 곡률 계산: segment 경계에서 절대 차분하지 않음 ────
    // (D 끝점이 R 첫 점과 차분되면 path_yaw가 180° 뒤집혀 차량이 발산함)
    for (auto& seg : gear_segments_) {
        int s = seg.first, e = seg.second;
        if (e <= s) {  // 단일 점 세그먼트
            wp_h_[s] = 0.0;
            continue;
        }
        wp_h_[s] = std::atan2(wp_y_[s+1] - wp_y_[s], wp_x_[s+1] - wp_x_[s]);
        for (int i = s + 1; i < e; ++i)
            wp_h_[i] = std::atan2(wp_y_[i+1] - wp_y_[i-1], wp_x_[i+1] - wp_x_[i-1]);
        wp_h_[e] = std::atan2(wp_y_[e] - wp_y_[e-1], wp_x_[e] - wp_x_[e-1]);
    }
    for (auto& seg : gear_segments_) {
        int s = seg.first, e = seg.second;
        for (int i = s; i <= e; ++i) {
            int lo = std::max(s, i - 1), hi = std::min(e, i + 1);
            if (lo == hi) { wp_k_[i] = 0.0; continue; }
            double dh = wrapAngle(wp_h_[hi] - wp_h_[lo]);
            double ds = std::hypot(wp_x_[hi] - wp_x_[lo], wp_y_[hi] - wp_y_[lo]);
            wp_k_[i] = (ds > 1e-6) ? (dh / ds) : 0.0;
        }
    }
    if (n >= 2) {
        double total = 0.0;
        for (int i = 1; i < n; ++i) total += std::hypot(wp_x_[i] - wp_x_[i-1], wp_y_[i] - wp_y_[i-1]);
        wp_spacing_ = total / (n - 1);
    }
    // ── 누적 path 거리 wp_s_ ────
    wp_s_.assign(n, 0.0);
    for (int i = 1; i < n; ++i) {
        wp_s_[i] = wp_s_[i-1] + std::hypot(wp_x_[i] - wp_x_[i-1], wp_y_[i] - wp_y_[i-1]);
    }
    vehicle_s_ = 0.0;

    // ── 회피 lateral offset 계산 (mixed.json 원본과 비교) ────
    wp_avoid_off_.assign(n, 0.0);
    // 같은 디렉토리의 mixed.json 시도 (avoid path만 사용 시 0)
    std::string mixed_file = file;
    size_t pos = mixed_file.find("mixed_avoid.json");
    if (pos != std::string::npos) {
        mixed_file.replace(pos, std::string("mixed_avoid.json").length(), "mixed.json");
        std::ifstream m_ifs(mixed_file);
        if (m_ifs.is_open()) {
            Json::Value m_root; Json::Reader m_reader;
            if (m_reader.parse(m_ifs, m_root)) {
                const Json::Value m_wps = m_root["waypoints"];
                int mn = std::min(n, (int)m_wps.size());
                for (int i = 0; i < mn; ++i) {
                    double mx = m_wps[i]["x"].asDouble();
                    double my = m_wps[i]["y"].asDouble();
                    wp_avoid_off_[i] = std::hypot(wp_x_[i] - mx, wp_y_[i] - my);
                }
                int n_avoid = 0; for (int i = 0; i < n; ++i) if (wp_avoid_off_[i] > 0.05) ++n_avoid;
                ROS_INFO("[PathFollower] 회피 영역: %d/%d wp (mixed.json 비교)", n_avoid, n);
            }
        }
    }

    ROS_INFO("[PathFollower] %d wp / %zu gear segments (start gear=%s) total_s=%.1fm",
             n, gear_segments_.size(), (cur_gear_ < 0) ? "R" : "D", n>0 ? wp_s_[n-1] : 0.0);
}

void PathFollower::avoidanceCallback(const std_msgs::Float64::ConstPtr& msg) {
    // 안전 cap: lane 폭 절반(±1.8m) 이내로만 허용
    double v = msg->data;
    avoidance_offset_ = std::max(-1.8, std::min(1.8, v));
}

void PathFollower::objectCallback(const morai_msgs::ObjectStatusList::ConstPtr& msg) {
    std::vector<Obstacle> obs;
    obs.reserve(msg->npc_list.size() + msg->pedestrian_list.size() + msg->obstacle_list.size());
    auto push = [&](const auto& list) {
        for (const auto& o : list) {
            Obstacle ob;
            ob.x  = o.position.x;
            ob.y  = o.position.y;
            ob.vx = o.velocity.x;
            ob.vy = o.velocity.y;
            ob.sx = std::max((double)o.size.x, 1.0);
            ob.sy = std::max((double)o.size.y, 1.0);
            obs.push_back(ob);
        }
    };
    push(msg->npc_list);
    push(msg->pedestrian_list);
    push(msg->obstacle_list);
    obstacles_ = std::move(obs);
}

void PathFollower::buildObstacleCorridor(const std::vector<double>& v_profile,
                                          std::vector<double>& d_min,
                                          std::vector<double>& d_max,
                                          double& v_scale) {
    // OSQP INF 대용 큰 값
    const double INF = 1.0e6;
    int N = cfg_.N;
    d_min.assign(N, -INF);
    d_max.assign(N,  INF);
    v_scale = 1.0;

    if (obstacles_.empty() || wp_x_.empty()) return;

    // path nearest_idx_ 기준 ego 진행 — wp_spacing 기준으로 path s 진행도
    // NPC를 path 위에 투영하기 위한 헬퍼: 가장 가까운 wp idx 찾기 (전방 ±100idx 안)
    auto projectToPath = [&](double ox, double oy)
        -> std::tuple<int, double, double> {
        // returns (best_idx, s_along_path_from_nearest, d_lateral_left_positive)
        int n = (int)wp_x_.size();
        int lo = std::max(0, nearest_idx_ - 5);
        int hi = std::min(n - 1, nearest_idx_ + (int)(cfg_.obs_s_window / wp_spacing_) + 20);
        int best = lo;
        double best_d2 = 1e18;
        for (int i = lo; i <= hi; ++i) {
            double dx = ox - wp_x_[i], dy = oy - wp_y_[i];
            double d2 = dx*dx + dy*dy;
            if (d2 < best_d2) { best_d2 = d2; best = i; }
        }
        // s_offset (path 진행거리, 부호 포함)
        double s_off = (best - nearest_idx_) * wp_spacing_;
        // d (좌측 +)
        int bi1 = std::min(best + 1, n - 1);
        double th = std::atan2(wp_y_[bi1] - wp_y_[best], wp_x_[bi1] - wp_x_[best]);
        double cs = std::cos(th), sn = std::sin(th);
        double rx = ox - wp_x_[best], ry = oy - wp_y_[best];
        double d  = -sn * rx + cs * ry;
        return {best, s_off, d};
    };

    double min_obs_s = 1.0e6;
    int active_count = 0;

    for (const auto& ob : obstacles_) {
        // NPC 현재 위치를 path-frenet에 투영
        auto [obs_idx, s_obs_now, d_obs_now] = projectToPath(ob.x, ob.y);

        // 1) 뒤쪽 NPC 무시 (path 진행거리 음수)
        if (s_obs_now < -2.0) continue;
        // 2) 너무 먼 NPC 무시
        if (s_obs_now > cfg_.obs_s_window) continue;
        // 3) path-d 너무 큰 NPC 무시 — 경로 위 NPC만 회피 (±2m + NPC half)
        double lat_half = std::max(ob.sy, 1.0) * 0.5 + cfg_.obs_lat_safety;
        if (std::abs(d_obs_now) > 2.0 + lat_half) continue;   // ±4→±2 축소

        active_count++;
        if (s_obs_now < min_obs_s) min_obs_s = s_obs_now;

        // NPC 미래 d_obs(k) — 등속 가정 (NPC v를 path s,d로 분해)
        // 단순화: path 진행 방향(현재 NPC 위치의 path heading) 기준 분해
        int bi1 = std::min(obs_idx + 1, (int)wp_x_.size() - 1);
        double th = std::atan2(wp_y_[bi1] - wp_y_[obs_idx], wp_x_[bi1] - wp_x_[obs_idx]);
        double cs = std::cos(th), sn = std::sin(th);
        double v_s = ob.vx * cs + ob.vy * sn;          // path 방향 속도
        double v_d = -ob.vx * sn + ob.vy * cs;         // lateral 속도

        // ego 도달 거리: 현재 속도 사용 (target_vel 가정은 멀리부터 무리하게 회피하게 함)
        double v_ego_pred = std::max(cur_v_, 2.0);
        double s_ego_k = 0.0;
        for (int k = 0; k < N; ++k) {
            double t_k = (k + 1) * cfg_.Ts;
            s_ego_k += v_ego_pred * cfg_.Ts;
            double s_obs_k = s_obs_now + v_s * t_k;
            double d_obs_k = d_obs_now + v_d * t_k;

            // 활성 조건: ego와 NPC의 s 차이가 obs_long_safety 안 (충돌 위험 step만)
            double long_half = std::max(ob.sx, 1.0) * 0.5 + cfg_.obs_long_safety;
            const double release_dist = 8.0;   // [m] 통과 후 점진 풀림 (15→8: 복귀 빠르게)
            // ego가 NPC를 통과한 후의 거리 (양수면 통과)
            double s_passed = s_ego_k - s_obs_k - long_half;
            if (s_passed > release_dist) continue;             // 10m 이상 통과 → 완전 비활성
            if (s_obs_k > s_ego_k + cfg_.obs_s_window) break;  // 너무 멀음

            // 통과 후 점진 풀림: lat_half를 점진적으로 0으로 감소
            double release_ratio = (s_passed > 0) ? std::min(1.0, s_passed / release_dist) : 0.0;
            double lat_eff = lat_half * (1.0 - release_ratio);

            // d corridor 좁히기 + lane 폭 cap (lane 밖 회피 방지)
            const double max_lane_offset = 1.3;   // [m] 한쪽 lane 회피 한계 (lane 폭 3m 기준)
            if (d_obs_k >= 0) {
                double new_max = d_obs_k - lat_eff;
                new_max = std::max(new_max, -max_lane_offset);   // lane 밖으로 못 가도록 cap
                if (new_max < d_max[k]) d_max[k] = new_max;
            } else {
                double new_min = d_obs_k + lat_eff;
                new_min = std::min(new_min,  max_lane_offset);
                if (new_min > d_min[k]) d_min[k] = new_min;
            }
        }
    }
    last_d_min_ = d_min[0];
    last_d_max_ = d_max[0];
    last_obs_dist_s_ = (active_count > 0) ? min_obs_s : -1.0;

    // 속도 감소: NPC가 path 앞에 있으면 거리 기반 감속 (회피 시간 확보)
    if (last_obs_dist_s_ > 0 && last_obs_dist_s_ < cfg_.obs_s_window) {
        double ratio = last_obs_dist_s_ / cfg_.obs_s_window;
        ratio = std::clamp(ratio, cfg_.obs_v_scale_min, 1.0);
        v_scale = ratio * ratio;
    }
}

void PathFollower::egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
    cur_x_ = msg->position.x;
    cur_y_ = msg->position.y;
    cur_yaw_ = wrapAngle(msg->heading * M_PI / 180.0);
    cur_v_ = std::hypot(msg->velocity.x, msg->velocity.y);
    cur_v_signed_ = msg->velocity.x;   // 후진 시 음수 (body-frame x)
    ego_rcvd_ = true;
}

PathFollower::NearResult PathFollower::findNearest() {
    const int n = static_cast<int>(wp_x_.size());
    // 후진 구간이면 heading 벡터를 180° 반전하여 dot product 계산
    double gear_sign = (cur_gear_ < 0) ? -1.0 : 1.0;
    const double hx = gear_sign * std::cos(cur_yaw_), hy = gear_sign * std::sin(cur_yaw_);

    // ── 기어 세그먼트 범위 결정: 현재 세그먼트만 검색. 전환 중이면 다음 세그먼트도 포함 ──
    int allow_lo = 0, allow_hi = n - 1;
    if (!gear_segments_.empty()) {
        int seg = std::clamp(cur_segment_, 0, (int)gear_segments_.size() - 1);
        allow_lo = gear_segments_[seg].first;
        allow_hi = gear_segments_[seg].second;
        if (gear_switching_ && seg + 1 < (int)gear_segments_.size()) {
            allow_lo = std::min(allow_lo, gear_segments_[seg + 1].first);
            allow_hi = std::max(allow_hi, gear_segments_[seg + 1].second);
        }
    }

    // ═══════════════════════════════════════════════════════════════
    // arc-length s 기반 nearest 검색 (self-overlapping path 강건)
    //   - 차량 vehicle_s_ 추적값 기준 [s-5m, s+30m] 윈도우 안에서만 매칭
    //   - self-overlap 두 번째 lap wp는 s 차이 크므로 자동 제외
    //   - 첫 frame search_init_=false: vehicle_s_=0 → wp[0] 근처
    // ═══════════════════════════════════════════════════════════════
    auto s_to_idx = [&](double s_target, int idx_lo, int idx_hi) -> int {
        // wp_s_가 monotonic 가정 — std::lower_bound로 idx 찾음
        auto it = std::lower_bound(wp_s_.begin() + idx_lo, wp_s_.begin() + idx_hi + 1, s_target);
        int idx = static_cast<int>(it - wp_s_.begin());
        return std::clamp(idx, idx_lo, idx_hi);
    };

    const double S_BACK    = 5.0;    // 차량 뒤로 검색 여유 [m]
    const double S_FORWARD = 30.0;   // 차량 앞으로 검색 거리 [m]

    int idx_lo = s_to_idx(vehicle_s_ - S_BACK,    allow_lo, allow_hi);
    int idx_hi = s_to_idx(vehicle_s_ + S_FORWARD, allow_lo, allow_hi);
    if (idx_hi < idx_lo) idx_hi = idx_lo;

    double min_d = std::numeric_limits<double>::max();
    int closest = std::clamp(nearest_idx_, idx_lo, idx_hi);
    bool found = false;
    for (int i = idx_lo; i <= idx_hi; ++i) {
        double dx = wp_x_[i] - cur_x_, dy = wp_y_[i] - cur_y_;
        double dist = std::hypot(dx, dy);
        // 차량 진행 방향(또는 R: 반대) 우선
        bool fwd = (dx * hx + dy * hy >= kDotThreshold);
        if (!found || (fwd && dist < min_d) || (!fwd && dist < min_d && min_d > kRecovDist)) {
            min_d = dist; closest = i; found = true;
        }
    }

    // window 안에 매칭 못 했거나 cte 매우 큼 → 전체 segment scan (fallback, 1회)
    if (!found || min_d > 10.0) {
        double best = std::numeric_limits<double>::max(); int best_idx = closest; bool fwd_found = false;
        for (int i = allow_lo; i <= allow_hi; ++i) {
            double dx = wp_x_[i] - cur_x_, dy = wp_y_[i] - cur_y_, dist = std::hypot(dx, dy);
            bool fwd = (dx * hx + dy * hy > 0.0);
            if (fwd) {
                if (!fwd_found || dist < best) { best = dist; best_idx = i; fwd_found = true; }
            } else if (!fwd_found && dist < best) {
                best = dist; best_idx = i;
            }
        }
        closest = best_idx; min_d = best;
    }
    search_init_ = true;

    // ── nidx 점프 cap (kMaxIndexStep): self-overlap 점프 추가 방어 ──
    int delta = closest - nearest_idx_;
    if (delta < 0) closest = nearest_idx_;
    else if (delta > kMaxIndexStep) closest = nearest_idx_ + kMaxIndexStep;
    nearest_idx_ = std::min(closest, n - 1);

    int ni = nearest_idx_, ni1 = std::min(ni + 1, n - 1);
    double path_yaw = std::atan2(wp_y_[ni1] - wp_y_[ni], wp_x_[ni1] - wp_x_[ni]);
    double rx = cur_x_ - wp_x_[ni], ry = cur_y_ - wp_y_[ni];
    double signed_cte = -std::sin(path_yaw) * rx + std::cos(path_yaw) * ry;

    // ── vehicle_s_ 갱신: nearest wp의 s + path tangent에 차량 위치 투영 ──
    double proj_along = std::cos(path_yaw) * rx + std::sin(path_yaw) * ry;
    vehicle_s_ = wp_s_[ni] + proj_along;
    // legacy /avoidance_offset 평행이동 — 신규 corridor와 중복되어 비활성화 (2026-05-18)
    // if (avoidance_enabled_) { signed_cte -= avoidance_offset_; }
    // 후진 시: 차량은 경로 진행 방향의 반대를 향하므로 π 보정
    double heading_err = (cur_gear_ < 0)
        ? wrapAngle(cur_yaw_ - path_yaw + M_PI)
        : wrapAngle(cur_yaw_ - path_yaw);
    return { nearest_idx_, min_d, signed_cte, heading_err, path_yaw };
}

double PathFollower::steerRateLimit(double steer_deg, double dt) {
    if (!cmd_init_) { prev_steer_ = steer_deg; cmd_init_ = true; return steer_deg; }
    double max_d = max_steer_rate_ * dt;
    steer_deg = std::clamp(steer_deg, prev_steer_ - max_d, prev_steer_ + max_d);
    return std::clamp(steer_deg, -max_steer_deg_, max_steer_deg_);
}

double PathFollower::velocitySigmoid(double v_tgt, double dt) {
    if (!v_sig_init_) { v_sig_ = v_tgt; v_sig_init_ = true; return v_sig_; }
    double tau = (v_tgt > v_sig_) ? sig_tau_up_ : sig_tau_down_;
    double alpha = 1.0 - std::exp(-dt / std::max(tau, 1e-3));
    v_sig_ += alpha * (v_tgt - v_sig_);
    return std::max(0.0, v_sig_);
}

void PathFollower::controlLoop(const ros::TimerEvent&) {
    if (!ego_rcvd_ || wp_x_.empty()) return;
    ros::Time now = ros::Time::now();
    double dt = (now - prev_cmd_time_).toSec();
    if (!std::isfinite(dt) || dt <= 1e-4) dt = cfg_.Ts;
    prev_cmd_time_ = now;
    auto t_start = ros::WallTime::now();

    // 실시간 진단 — 1초마다 핵심 상태 출력 (사용자가 ssh/터미널에서 직접 확인)
    ROS_INFO_THROTTLE(1.0, "[diag] cur_gear=%s v_signed=%+.2f m/s v_mag=%.2f km/h idx=%d gear_sw=%d ego(%6.1f,%6.1f) yaw=%+.0f deg",
                      (cur_gear_ < 0) ? "R" : "D",
                      cur_v_signed_, cur_v_ * 3.6,
                      nearest_idx_, gear_switching_ ? 1 : 0,
                      cur_x_, cur_y_, cur_yaw_ * 180.0 / M_PI);

    // ── 초기 기어 + 자율주행 모드 동기화 ──────────────────────
    // 첫 틱: 정지 명령 + service call 보내고 dwell 진입.
    // service call은 차량 정지 후 호출해야 MORAI가 수락 (단, 첫 틱은 v=0 시작이라 즉시 OK).
    if (!gear_initialized_ && !gear_switching_ && !wp_gear_.empty()) {
        gear_initialized_ = true;
        gear_switching_ = true;
        // 시작 시점은 무조건 정지 가정 → 즉시 service call (v<0.1 대기 생략)
        gear_switch_sent_ = true;
        gear_switch_target_ = wp_gear_[0];
        cur_gear_ = gear_switch_target_;
        gear_switch_time_ = now;
        morai_msgs::MoraiEventCmdSrv srv;
        srv.request.request.option = 3;
        srv.request.request.ctrl_mode = 3;
        srv.request.request.gear = (cur_gear_ < 0) ? 2 : 4;
        int ok_n = 0;
        for (int retry = 0; retry < 5; ++retry) {
            if (gear_srv_.call(srv)) ok_n++;
        }
        // /InsnControl topic fallback (service 미advertise 환경 대응)
        morai_msgs::EventInfo ev;
        ev.option = 3; ev.ctrl_mode = 3; ev.gear = srv.request.request.gear;
        for (int retry = 0; retry < 10; ++retry) {
            event_pub_.publish(ev);
            ros::Duration(0.05).sleep();
        }
        ROS_INFO("[PathFollower] 시작 기어 강제 설정: %s (gear=%d) [service ok=%d/5 + topic 10회]",
                 (cur_gear_ < 0) ? "R" : "D", srv.request.request.gear, ok_n);
        publishCmd(0.0, 0.0);
        return;
    }

    // 기어 역전 감지 — 명령 D인데 차량이 0.5 m/s 이상 후진 중이면 MORAI gear 잘못
    // (또는 명령 R인데 0.5 이상 전진) → service 재호출
    if (!gear_switching_ && ego_rcvd_) {
        bool inversion = (cur_gear_ > 0 && cur_v_signed_ < -0.5) ||
                         (cur_gear_ < 0 && cur_v_signed_ > +0.5);
        if (inversion) {
            ROS_WARN_THROTTLE(2.0, "[PathFollower] 기어 역전 감지: cmd_gear=%s v_signed=%.2f → service 재호출",
                              (cur_gear_ < 0) ? "R" : "D", cur_v_signed_);
            gear_switching_ = true;
            gear_switch_sent_ = false;
            gear_switch_target_ = cur_gear_;
            publishCmd(0.0, 0.0);
            return;
        }
    }

    // ── 기어 전환 관리 ─────────────────────────────────────────
    // 현재 세그먼트 끝점에 도달했고 다음 세그먼트가 다른 기어이면 전환 시작
    int next_gear = cur_gear_;
    bool at_segment_end = false;
    if (!gear_segments_.empty() && cur_segment_ + 1 < (int)gear_segments_.size()) {
        int seg_hi = gear_segments_[cur_segment_].second;
        // 트리거 조건:
        // (a) nearest_idx >= seg_hi - 1 (끝 1점 이내 도달)
        // (b) nearest_idx >= seg_hi - 5 + 차량이 끝점 좌표를 지났음
        // (c) drift 발생 (cte > 2m) + idx >= seg_hi - 10 (실패 안전망)
        bool a_idx_at_end = (nearest_idx_ >= seg_hi - 1);
        bool b_passed_end = false;
        if (nearest_idx_ >= seg_hi - 5 && seg_hi >= 1) {
            // 끝점 방향 vector. vehicle이 끝점 너머로 갔는지 확인
            double tx = wp_x_[seg_hi] - wp_x_[seg_hi - 1];
            double ty = wp_y_[seg_hi] - wp_y_[seg_hi - 1];
            double vx = cur_x_ - wp_x_[seg_hi];
            double vy = cur_y_ - wp_y_[seg_hi];
            // vehicle이 path 끝점 진행방향 너머에 있으면 dot > 0
            if (tx * vx + ty * vy > 0) b_passed_end = true;
        }
        bool c_drift = false;
        if (nearest_idx_ >= seg_hi - 10) {
            int ni = std::max(0, std::min((int)wp_x_.size()-2, nearest_idx_));
            double th = std::atan2(wp_y_[ni+1] - wp_y_[ni], wp_x_[ni+1] - wp_x_[ni]);
            double cte_chk = -std::sin(th) * (cur_x_ - wp_x_[ni]) +
                              std::cos(th) * (cur_y_ - wp_y_[ni]);
            if (std::abs(cte_chk) > 2.0) c_drift = true;
        }
        if (a_idx_at_end || b_passed_end || c_drift) {
            next_gear = wp_gear_[gear_segments_[cur_segment_ + 1].first];
            at_segment_end = true;
        }
    }
    // 기어 전환 트리거 — at_segment_end (nearest_idx >= seg_hi)이면 무조건 즉시
    // (정밀 정렬은 NMPC_LO 사전감속에서 거리비례 v 감속으로 처리됨)
    if (at_segment_end && next_gear != cur_gear_ && !gear_switching_) {
        int seg_end_idx = gear_segments_[cur_segment_].second;
        double dx_end = cur_x_ - wp_x_[seg_end_idx];
        double dy_end = cur_y_ - wp_y_[seg_end_idx];
        double d_end = std::sqrt(dx_end * dx_end + dy_end * dy_end);
        double hdg_at_end = wrapAngle(cur_yaw_ - wp_h_[seg_end_idx]);

        gear_switching_ = true;
        gear_switch_sent_ = false;
        gear_switch_target_ = next_gear;
        cur_segment_++;
        publishCmd(0.0, 0.0);
        ROS_INFO("[PathFollower] 기어 전환 시작: dist=%.2fm hdg=%.1f° v=%.2f km/h",
                 d_end, hdg_at_end * 180.0 / M_PI, std::abs(cur_v_) * 3.6);
        return;
    }

    // 기어 전환 진행: (1) 차량 정지 대기 → (2) service call → (3) 0.5초 dwell → 재개
    if (gear_switching_) {
        publishCmd(0.0, 0.0);  // 항상 정지 명령 유지
        if (!gear_switch_sent_) {
            // 차량이 거의 정지했을 때(0.1 m/s 미만) service call. 안전 timeout 2초.
            double waited = (now - prev_cmd_time_).toSec();
            (void)waited;
            if (std::abs(cur_v_) < 0.1) {
                gear_switch_sent_ = true;
                cur_gear_ = gear_switch_target_;
                gear_switch_time_ = now;
                morai_msgs::MoraiEventCmdSrv srv;
                srv.request.request.option = 3;
                srv.request.request.ctrl_mode = 3;
                srv.request.request.gear = (cur_gear_ < 0) ? 2 : 4;
                int success_n = 0;
                for (int retry = 0; retry < 3; ++retry) {
                    if (gear_srv_.call(srv)) success_n++;
                }
                // /InsnControl topic fallback — 30회 × 100ms = 3초 강하게 publish
                morai_msgs::EventInfo ev;
                ev.option = 2;  // gear만 (자율주행 모드 유지)
                ev.gear = srv.request.request.gear;
                for (int retry = 0; retry < 30; ++retry) {
                    event_pub_.publish(ev);
                    publishCmd(0.0, 0.0);  // 차량 정지 유지
                    ros::Duration(0.1).sleep();
                }
                ROS_INFO("[PathFollower] 정지 확인 → 기어 전환: %s (gear=%d) seg=%d  [service ok=%d/3 + topic 30회]",
                         (cur_gear_ < 0) ? "R" : "D", srv.request.request.gear,
                         cur_segment_, success_n);
            }
            return;
        }
        // service 호출 후 0.5초 dwell
        if ((now - gear_switch_time_).toSec() < kGearSwitchWait) {
            return;
        }
        gear_switching_ = false;
        in_recov_ = false;          // 새 세그먼트 시작 시 RECOV 잔여 상태 리셋
        prev_was_recov_ = true;     // 첫 NORMAL 틱 kappa 초기화 트리거
        current_kappa_ = 0.0;       // 누적 steer 리셋 (이전 모드의 kappa 누적 제거)
        if (nmpc_) nmpc_->reset();          // legacy NMPC warm-start 폐기
        if (rti_nmpc_) rti_nmpc_->reset();  // RTI-NMPC 상태 + warm-start 폐기
        // 첫 R 진입 시 주차 모드 활성 — 이후 D는 저속 유지
        if (cur_gear_ < 0 && !parking_mode_) {
            parking_mode_ = true;
            ROS_INFO("[PathFollower] 주차 모드 활성 — 이후 모든 D segment %.1f km/h 제한",
                     parking_max_kmh_);
        }
        // 속도/조향 smoother 리셋 — D→R 전환 시 잔여값(13km/h 등)이 R 첫 명령에 누설되는 것 방지
        v_sig_init_ = false;
        v_sig_      = 0.0;
        prev_steer_ = 0.0;
        cmd_init_   = false;
        // R 진입이면 yaw 동기화 phase 활성화
        if (cur_gear_ < 0) {
            r_align_active_ = true;
            r_entry_yaw_   = cur_yaw_;   // 현재 vehicle yaw를 시작 yaw로
            r_align_x0_    = cur_x_;
            r_align_y0_    = cur_y_;
            ROS_INFO("[PathFollower] R 진입 — yaw 동기화 시작: entry_yaw=%.1f°",
                     cur_yaw_ * 180.0 / M_PI);
        } else {
            r_align_active_ = false;
        }
        ROS_INFO("[PathFollower] 기어 전환 완료, 주행 재개");
    }

    NearResult near = findNearest();
    const int n = static_cast<int>(wp_x_.size());

    // R idx stuck 감지 + 강제 catch-up (cte 작을 때만 — 가짜 진행 방지)
    if (cur_gear_ < 0 && !gear_switching_) {
        if (!r_idx_stuck_init_ || nearest_idx_ != r_idx_stuck_prev_) {
            r_idx_stuck_prev_ = nearest_idx_;
            r_idx_stuck_t_ = now;
            r_idx_stuck_init_ = true;
        } else {
            double stuck_dt = (now - r_idx_stuck_t_).toSec();
            // cte 가까울 때만 advance — 멀면 가짜 진행이라 안 함
            double cte_now = std::abs(near.signed_cte);
            if (stuck_dt > kStuckTimeoutSec && cte_now < kStuckMaxCte) {
                int seg_hi_r = nearest_idx_;
                if (!gear_segments_.empty() && cur_segment_ < (int)gear_segments_.size()) {
                    seg_hi_r = gear_segments_[cur_segment_].second;
                }
                int new_idx = std::min(nearest_idx_ + kStuckAdvanceStep, seg_hi_r);
                if (new_idx > nearest_idx_) {
                    ROS_WARN("[PathFollower] R idx %d stuck %.1fs cte=%.2f → +%d (idx %d→%d)",
                             nearest_idx_, stuck_dt, cte_now, kStuckAdvanceStep, nearest_idx_, new_idx);
                    nearest_idx_ = new_idx;
                    r_idx_stuck_prev_ = new_idx;
                    r_idx_stuck_t_ = now;
                }
            }
        }
    } else {
        r_idx_stuck_init_ = false;
    }

    // ── Compute curvature lookahead (현재 세그먼트 내부로 제한) ──
    // 세그먼트 경계 넘어 다음 기어 곡률까지 보면 D 추종 중 R 곡선에 끌려가 감속 폭주
    int seg_hi_for_la = (int)wp_k_.size() - 1;
    if (!gear_segments_.empty() && cur_segment_ < (int)gear_segments_.size()) {
        seg_hi_for_la = gear_segments_[cur_segment_].second;
    }
    double max_kappa_ahead = 0.0;
    {
        double la_m = cfg_.curve_lookahead_m;
        int la_steps = std::max(1, (int)std::round(la_m / wp_spacing_));
        for (int i = 0; i <= la_steps; ++i) {
            int ki = std::min(nearest_idx_ + i, seg_hi_for_la);
            max_kappa_ahead = std::max(max_kappa_ahead, std::abs(wp_k_[ki]));
            if (ki == seg_hi_for_la) break;
        }
    }

    Json::Value rec(Json::objectValue);
    rec["t"] = (now - log_t0_).toSec();
    rec["x"] = cur_x_; rec["y"] = cur_y_;
    rec["v_kmh"] = cur_v_ * 3.6;
    rec["cte"] = near.signed_cte;
    rec["hdg_err_deg"] = near.heading_err * 180.0 / M_PI;
    rec["target_vel"] = cfg_.target_vel * 3.6;
    rec["actual_vel"] = cur_v_ * 3.6;
    rec["vel_error"] = (cfg_.target_vel - cur_v_) * 3.6;
    rec["path_curvature"] = wp_k_[nearest_idx_];
    rec["max_kappa_ahead"] = max_kappa_ahead;
    rec["near_dist"]    = near.dist;
    rec["nearest_idx"]  = near.idx;
    rec["cur_gear"]     = cur_gear_;
    rec["gear"]         = (cur_gear_ < 0) ? "R" : "D";

    if (nearest_idx_ >= n - 2) {
        if (std::hypot(wp_x_.back() - cur_x_, wp_y_.back() - cur_y_) < 2.0) {
            publishCmd(0.0, 0.0); return;
        }
    }

    // 후진 시 hdg 임계값 완화 (60°), 전진은 그대로 (35°)
    double hdg_thresh = (cur_gear_ < 0) ? kRecovHdgThreshR : kRecovHdgThresh;
    // 회피 활성 시 cooldown flag 켜기 → ego가 정확히 정렬될 때까지 RECOV/가속 차단
    bool obs_now_active = avoidance_enabled_ && (last_obs_dist_s_ > 0);
    if (obs_now_active) { obs_block_until_align_ = true; align_stable_count_ = 0; }
    // 정확한 정렬: cte<0.2m + |yaw_err|<0.05rad(약 2.9°)
    bool ego_aligned = (std::abs(near.signed_cte) < 0.2) &&
                       (std::abs(near.heading_err) < 0.05);
    // align stable check: 회피 path가 cte=0 가로지를 때 단일 tick 만족 false-positive 방지
    if (ego_aligned) align_stable_count_++; else align_stable_count_ = 0;
    const int kAlignStableTicks = 20;   // 1.0초 연속 정렬
    if (obs_block_until_align_ && !obs_now_active && align_stable_count_ >= kAlignStableTicks) {
        obs_block_until_align_ = false;
    }
    bool recov_blocked = avoidance_enabled_ && obs_block_until_align_;
    bool enter_recov = !recov_blocked && (
        (near.dist > kRecovDist) || (std::abs(near.heading_err) > hdg_thresh));
    bool exit_recov = (near.dist < kRecovDistExit) && (std::abs(near.heading_err) < kRecovHdgExit);
    if (exit_recov) in_recov_ = false; else if (enter_recov) in_recov_ = true;
    if (recov_blocked && in_recov_) in_recov_ = false;   // 회피 cooldown 중 잔여 RECOV 강제 해제
    rec["obs_block_align"] = obs_block_until_align_ ? 1 : 0;
    rec["ego_aligned"] = ego_aligned ? 1 : 0;

    // ── path 끝 도달 시 정지 (헬리콥터 발산 방지) ────────────────
    // 마지막 세그먼트 + nearest_idx_가 끝 근처 + 끝점에 충분히 가까움
    if (!gear_segments_.empty() && cur_segment_ == (int)gear_segments_.size() - 1) {
        int seg_end = gear_segments_[cur_segment_].second;
        double dx = cur_x_ - wp_x_[seg_end];
        double dy = cur_y_ - wp_y_[seg_end];
        double dist_end = std::sqrt(dx*dx + dy*dy);
        // 종료 조건: idx가 끝(혹은 그 근처) + 끝점에서 1.0m 이내, 또는 idx==end + 0.3초 이상 idx 진행 없음
        if (nearest_idx_ >= seg_end - 1 && dist_end < 1.0) {
            publishCmd(0.0, 0.0);
            rec["mode"] = "FINISHED";
            rec["controller"] = "STOP";
            rec["dist_end"] = dist_end;
            log_recs_.push_back(rec);
            std_msgs::String status; status.data = "FINISHED";
            status_pub_.publish(status);
            // 3초 후 자동 종료 (로그 flush 자동)
            static ros::Time finished_at = ros::Time(0);
            if (finished_at.toSec() == 0.0) {
                finished_at = now;
                ROS_INFO("[PathFollower] path 끝 도달 (idx=%d/%d, dist=%.2fm) — 3초 후 자동 종료",
                         nearest_idx_, seg_end, dist_end);
            } else if ((now - finished_at).toSec() > 3.0) {
                ROS_INFO("[PathFollower] 자동 종료 — 로그 flush");
                ros::shutdown();
            }
            return;
        }
    }

    // R 또는 저속 D는 NMPC 우선 — RECOV 우회 (NMPC가 큰 오차도 처리 가능)
    // 저속 hysteresis: thresh ± hyst/2 (default 4 ± 0.5 → 3.5/4.5 km/h)
    double v_kmh_now = std::abs(cur_v_) * 3.6;
    double thresh_hi = low_speed_thresh_kmh_ + low_speed_hyst_kmh_ * 0.5;
    double thresh_lo = low_speed_thresh_kmh_ - low_speed_hyst_kmh_ * 0.5;
    if (v_kmh_now > thresh_hi)      in_low_speed_ = false;
    else if (v_kmh_now < thresh_lo) in_low_speed_ = true;

    // 기어 전환 사전 감속 (적극): max(pre_gear_change_dist_m_, 0.7 * v_kmh)
    // 60km/h시 42m, 30km/h시 21m, 4km/h시 = pre_gear_change_dist_m_ (default 10m)
    double v_kmh_for_dist = std::abs(cur_v_) * 3.6;
    double effective_pre_dist = std::max(pre_gear_change_dist_m_, 0.7 * v_kmh_for_dist);
    bool approaching_gear_change = false;
    if (!gear_segments_.empty() && cur_segment_ + 1 < (int)gear_segments_.size()) {
        int next_gear = wp_gear_[gear_segments_[cur_segment_ + 1].first];
        if (next_gear != cur_gear_) {
            int seg_end = gear_segments_[cur_segment_].second;
            double dist_remaining = 0.0;
            for (int k = nearest_idx_; k < seg_end; ++k) {
                double dx = wp_x_[k + 1] - wp_x_[k];
                double dy = wp_y_[k + 1] - wp_y_[k];
                dist_remaining += std::sqrt(dx * dx + dy * dy);
                if (dist_remaining > effective_pre_dist) break;
            }
            if (dist_remaining <= effective_pre_dist) {
                approaching_gear_change = true;
            }
        }
    }

    // 컨트롤러 분기:
    //   1) R 모드: 무조건 RTI-NMPC
    //   2) D + parking_mode + sharp curve (max_κ > 0.10): RTI-NMPC
    //      → D2 같은 저속 sharp curve 영역에서 LTV 인커브 컷 회피
    //   3) 그 외 D 모드: LTV
    bool d_parking_sharp = (cur_gear_ > 0) && parking_mode_ && (max_kappa_ahead > 0.10);
    // 회피 종료 cooldown — 큰 cte/yaw 비선형 영역. LTV mismatch → NMPC로 정확한 운동학 적용
    bool obs_corridor_on = avoidance_enabled_ && (cur_gear_ > 0) && (last_obs_dist_s_ > 0);
    bool obs_cooldown = avoidance_enabled_ && obs_block_until_align_ && !obs_corridor_on;
    bool use_nmpc_now = (cur_gear_ < 0) || d_parking_sharp || in_low_speed_ || obs_cooldown || force_nmpc_;

    // LTV→NMPC 전환 감지: 직전 LTV의 steering을 RTI에 인계 (warm-start)
    // 이렇게 안 하면 전환 첫 tick에 RTI kappa=0에서 시작해 갑작스런 명령 점프 발생
    if (use_nmpc_now && !prev_use_nmpc_ && rti_nmpc_) {
        // LTV가 만든 마지막 steer (deg) → kappa 변환
        double last_steer_rad = prev_steer_ * M_PI / 180.0;
        double kappa_init = std::tan(last_steer_rad) / cfg_.L;
        rti_nmpc_->setInitialKappa(kappa_init);
        ROS_INFO_THROTTLE(1.0, "[PathFollower] LTV→NMPC 인계: prev_steer=%.2fdeg → kappa=%.4f",
                          prev_steer_, kappa_init);
    }
    prev_use_nmpc_ = use_nmpc_now;

    if (in_recov_ && !use_nmpc_now) {
        double la_dist = std::max(4.0, 10.0 - near.dist * 2.0);
        // 후진 시 look-ahead를 경로 진행 방향으로 (인덱스 증가 방향)
        int target_idx = std::min(nearest_idx_ + (int)std::round(la_dist / wp_spacing_), n - 1);
        double bearing = std::atan2(wp_y_[target_idx] - cur_y_, wp_x_[target_idx] - cur_x_);
        // 후진 시: 차량 뒤쪽으로 가야 하므로 bearing에서 π 보정
        double alpha = (cur_gear_ < 0)
            ? wrapAngle(bearing - cur_yaw_ + M_PI)
            : wrapAngle(bearing - cur_yaw_);
        
        // RECOV mode output also in RADIANS
        double steer_rad = std::clamp(alpha, -max_steer_deg_ * M_PI / 180.0, max_steer_deg_ * M_PI / 180.0);
        double recov_vel = (cur_gear_ < 0) ? std::min(kRecovMaxVel, reverse_max_vel_kmh_) : kRecovMaxVel;
        double v_cmd = velocitySigmoid(recov_vel, dt);

        // Apply rate limit and unify sign (- for MORAI)
        double steer_deg_limited = steerRateLimit(steer_rad * 180.0 / M_PI, dt);
        double final_steer_rad = steer_deg_limited * M_PI / 180.0;

        prev_steer_ = steer_deg_limited;
        prev_was_recov_ = true;

        rec["mode"] = (cur_gear_ < 0) ? "RECOV_R" : "RECOV";
        rec["steer_cmd"] = final_steer_rad;
        log_recs_.push_back(rec);

        {
            std_msgs::Float32MultiArray perf;
            perf.data.resize(7);
            perf.data[0] = static_cast<float>(near.dist);
            perf.data[1] = 0.0f;
            perf.data[2] = static_cast<float>(near.signed_cte);
            perf.data[3] = static_cast<float>(near.heading_err * 180.0 / M_PI);
            perf.data[4] = static_cast<float>(cur_v_ * 3.6);
            perf.data[5] = 0.0f;
            perf.data[6] = static_cast<float>(recov_vel);
            perf_pub_.publish(perf);

            std_msgs::String status;
            status.data = (cur_gear_ < 0) ? "RECOV_R" : "RECOV";
            status_pub_.publish(status);
        }

        publishCmd(v_cmd, final_steer_rad); return;
    }

    // ═══════════════════════════════════════════════════════════════
    // RTI-NMPC 분기 — 후진(R) 또는 저속(< low_speed_thresh_kmh_) 시 활성
    // (RECOV보다 우선 적용; 5-state 운동학 자전거, 암시적 오일러)
    // ═══════════════════════════════════════════════════════════════
    if (use_nmpc_now) {
        // force_nmpc + 고속/non-low 시 NMPC_HS 표시 (GUI 명확화)
        const char* fwd_mode = (force_nmpc_ && !in_low_speed_) ? "NMPC_HS" : "NMPC_LO";
        rec["mode"]       = (cur_gear_ < 0) ? "NMPC_R" : fwd_mode;
        rec["gear"]       = (cur_gear_ < 0) ? "R"      : "D";
        rec["controller"] = "RTI_NMPC";

        // 곡률 기반 자동 속도 계산 (ω = v·κ ≥ 15°/s 보장)
        // k_eff floor 0.02로 직선부 보호, clamp [floor, cap]
        auto calc_v_from_kappa = [](double max_k, double v_floor, double v_cap) {
            const double omega_tgt = 15.0 * M_PI / 180.0;  // 15°/s = 0.262 rad/s
            double k_eff = std::max(0.02, max_k);
            double v_kmh = (omega_tgt / k_eff) * 3.6;
            return std::clamp(v_kmh, v_floor, v_cap);
        };

        // 목표 속도 결정
        double v_mag_kmh;
        if (cur_gear_ < 0) {
            // R 모드: 곡률 자동 속도 (1.5~reverse_max cap)
            double r_cap = std::max(reverse_max_vel_kmh_, 5.0);
            v_mag_kmh = calc_v_from_kappa(max_kappa_ahead, 1.5, r_cap);
        } else if (approaching_gear_change) {
            // D→R 직전: 거리 비례 정밀 감속
            // dist > 5m: 4 km/h
            // dist 0.5~5m: 4 → 1 km/h 선형
            // dist < 0.5m: 1 km/h crawl (정밀 정렬)
            v_mag_kmh = low_speed_thresh_kmh_;
            if (cur_segment_ + 1 < (int)gear_segments_.size()) {
                int sei = gear_segments_[cur_segment_].second;
                double de = std::hypot(cur_x_ - wp_x_[sei], cur_y_ - wp_y_[sei]);
                if (de < 5.0) {
                    double tgt = 1.0 + (de - 0.5) * (4.0 - 1.0) / (5.0 - 0.5);
                    v_mag_kmh = std::max(1.0, std::min(low_speed_thresh_kmh_, tgt));
                }
                if (de < 0.5) v_mag_kmh = 1.0;
            }
        } else if (d_parking_sharp) {
            // D2 sharp curve: 곡률 자동 속도 (3.5~5.0 km/h, ω≥15°/s 목표)
            v_mag_kmh = calc_v_from_kappa(max_kappa_ahead, 3.5, 5.0);
        } else if (obs_cooldown) {
            // 회피 종료 cooldown: 큰 cte/yaw 회복용 저속 (13 km/h)
            v_mag_kmh = 13.0;
        } else if (in_recov_ && cur_gear_ > 0) {
            // RECOV — 큰 cte/yaw 회복용. 8→25 km/h 부스트 (정지 시 ramp-up 가속)
            v_mag_kmh = 25.0;
        } else {
            // 고속 D 일반주행: 사전감속 3중 제약 (A·B·C 결합)
            //   C) 속도비례 lookahead — 빠를수록 더 멀리 곡률 탐색
            //   B) 횡가속도 제한    — v² · κ ≤ a_lat_max (물리 한계)
            //   1) 곡률 비례 감속   — v = v_max / (1 + α·κ) (부드러운 프로필)
            //   A) jerk rate-limit  — target_vel step 변화 차단 (실제 사전감속 효과)
            // 튜닝 (목표: 평균 속도 30 km/h 달성)
            //   alpha 6   : κ=0.05 → 38.5km/h, κ=0.1 → 31.3km/h
            //   a_lat 6.0: κ=0.05 → 39.5km/h, κ=0.1 → 27.9km/h
            //   v_floor 28: 완만 곡선까지 최소 28km/h (sharp만 자동 감속)
            constexpr double alpha     = 6.0;
            constexpr double a_lat_max = 5.0;  // 6→5: 그립한계 더 엄수
            constexpr double max_dec   = 4.0;
            constexpr double max_acc   = 3.5;
            constexpr double dt_tick   = 0.05;   // 제어 주기 [s] (RTI Ts와 매칭)

            // C) lookahead 동적 (55m best)
            const double v_now  = std::abs(cur_v_);
            double la_dyn = v_now * 3.5 + (v_now * v_now) / (2.0 * 2.0);
            double la_m_d = std::max(30.0, std::min(la_dyn, 55.0));
            int la_steps_d = std::max(1, (int)std::round(la_m_d / wp_spacing_));
            double max_k_d = 0.0;
            for (int i = 0; i <= la_steps_d; ++i) {
                int ki = std::min(nearest_idx_ + i, seg_hi_for_la);
                max_k_d = std::max(max_k_d, std::abs(wp_k_[ki]));
                if (ki == seg_hi_for_la) break;
            }
            rec["la_m_dyn"] = la_m_d;
            rec["max_kappa_dyn"] = max_k_d;

            // 1) 곡률 비례 감속
            double v_kmh = (cfg_.target_vel / (1.0 + alpha * max_k_d)) * 3.6;
            // B) 횡가속도 캡
            if (max_k_d > 1e-3) {
                double v_alat_kmh = std::sqrt(a_lat_max / max_k_d) * 3.6;
                v_kmh = std::min(v_kmh, v_alat_kmh);
            }
            v_kmh = std::max(cfg_.curve_min_vel * 3.6, v_kmh);
            // 평균 속도 floor + a_lat hard cap (그립한계 엄수)
            auto cap_by_alat = [&](double v_floor) {
                if (max_k_d < 1e-3) return v_floor;
                double v_cap_kmh = std::sqrt(a_lat_max / max_k_d) * 3.6;
                return std::min(v_floor, v_cap_kmh);
            };
            // floor를 target_vel과 동기 — hardcoded 38 무시되던 버그 fix
            const double tv_kmh = cfg_.target_vel * 3.6;  // R/D 부호 제거 (abs 사용)
            const double tv_abs = std::abs(tv_kmh);
            if (max_k_d < 0.08)      v_kmh = std::max(cap_by_alat(tv_abs), v_kmh);
            else if (max_k_d < 0.15) v_kmh = std::max(cap_by_alat(tv_abs * 0.7), v_kmh);
            else if (max_k_d < 0.20) v_kmh = std::max(cap_by_alat(tv_abs * 0.5), v_kmh);
            // sharp(κ>0.20) 자동감속

            // A) rate-limit (target_vel step → 부드러운 감속 = 사전감속)
            if (prev_v_target_kmh_ > 0.0) {
                double dv_up   = max_acc * dt_tick * 3.6;
                double dv_down = max_dec * dt_tick * 3.6;
                double dv = v_kmh - prev_v_target_kmh_;
                dv = std::max(-dv_down, std::min(dv_up, dv));
                v_kmh = prev_v_target_kmh_ + dv;
            }
            prev_v_target_kmh_ = v_kmh;
            v_mag_kmh = v_kmh;
        }
        double v_target_mps = (cur_gear_ < 0 ? -1.0 : 1.0) * (v_mag_kmh / 3.6);

        // 매 tick cfg에 부호 반영 (R/D 전환 시 즉시 적용)
        rti_cfg_.target_velocity = v_target_mps;
        // D 끝 사전감속 시: w_psi 강하게 (yaw 정렬 강제 — D 끝점에 정렬 도달)
        RTINMPCConfig eff_cfg = rti_cfg_;
        if (approaching_gear_change && cur_gear_ > 0) {
            eff_cfg.w_psi = 80.0;     // hdg 강제 (D 끝 정렬)
            eff_cfg.w_kappa = 0.5;    // κ 거의 무시
        } else if (d_parking_sharp) {
            // D2 sharp curve: cte 추종 강화 — 헤딩만 잘 따르고 path 이탈 1m+ 방지
            eff_cfg.w_px  = 30.0;     // 10 → 30 (위치 추종 3배)
            eff_cfg.w_py  = 30.0;
            eff_cfg.w_psi =  6.0;     // 8 → 6 (헤딩 가중치 약간 낮춤 — 위치 우선)
        } else if (cur_gear_ < 0) {
            // R 모드: cte 추종 강화 + yaw 강화 (R hdg 36° → 안정화)
            eff_cfg.w_px    = 35.0;
            eff_cfg.w_py    = 35.0;
            eff_cfg.w_psi   = 28.0;   // 12→28 yaw 정렬 강화 (R 후진 추종 정확도)
            eff_cfg.w_kappa =  5.0;   // 3→5 curvature feedforward 강화
            eff_cfg.w_akappa = 40.0;  // steer rate cost (진동 방지)
        } else if (obs_cooldown) {
            // 회피 종료 cooldown: yaw 정렬 우선 + cte 부드럽게 회복 (overshoot 방지)
            eff_cfg.w_px    = 15.0;   // lateral 압박 약화 (큰 yaw 변화 강요 X)
            eff_cfg.w_py    = 15.0;
            eff_cfg.w_psi   = 30.0;   // yaw 정렬 강화 (yaw 작게 유지)
            eff_cfg.w_kappa =  5.0;   // input 부드럽게 (steer rate 변화 ↓)
        } else if (in_recov_ && cur_gear_ > 0) {
            // RECOV (D): cte/yaw 발산 회복 — steer 자체와 rate 둘 다 강 제한
            // 핵심: kappa_max 강 제한 (steer 35°→13°) — 풀스트로크 못 내게
            eff_cfg.w_px      = 10.0;
            eff_cfg.w_py      = 10.0;
            eff_cfg.w_psi     = 25.0;
            eff_cfg.w_kappa   = 10.0;   // 5→10: kappa 자체 페널티
            eff_cfg.w_akappa  = 25.0;
            eff_cfg.akappa_min = -0.10;
            eff_cfg.akappa_max =  0.10;
            eff_cfg.kappa_min  = -0.08; // steer ±13.5° 한계 (이전 ±40° 풀스트로크)
            eff_cfg.kappa_max  =  0.08;
        } else {
            // D 일반주행 적응형 게인 (곡선 정교화 우선)
            //   곡선: w_kappa 강화 (ref_κ 정확 추종) + cte 약화 (yaw 강요 X) + heading 강화
            //   원리: NMPC가 path 곡률을 ref_kappa로 직접 따라가게 → cte 보정용 풀스트로크 방지
            const double v_kmh_now = std::abs(cur_v_) * 3.6;
            const bool   straight  = (max_kappa_ahead < 0.03);  // κ<0.03 ≈ 반경 33m+
            // ── 회피 영역 감지 — active(진입 ~ 통과) vs cooldown(통과 후 30wp)
            bool in_avoid_active = false;     // 회피 영역 안 또는 진입 직전
            bool in_avoid_cooldown = false;   // 회피 통과 직후 (path 복귀 추종)
            if (!wp_avoid_off_.empty()) {
                int la_fwd  = std::min((int)wp_avoid_off_.size() - 1, nearest_idx_ + 30);
                // active: 현재 + 앞 30wp (진입 준비 + 통과 중)
                for (int i = nearest_idx_; i <= la_fwd; ++i) {
                    if (wp_avoid_off_[i] > 0.1) { in_avoid_active = true; break; }
                }
                if (!in_avoid_active) {
                    // cooldown: 뒤 30wp 안 회피 영역 있으면
                    int la_back = std::max(0, nearest_idx_ - 30);
                    for (int i = la_back; i < nearest_idx_; ++i) {
                        if (wp_avoid_off_[i] > 0.1) { in_avoid_cooldown = true; break; }
                    }
                }
            }
            bool in_avoid = in_avoid_active || in_avoid_cooldown;
            if (in_avoid_active) {
                // 회피 active — cte 최우선 (path 정확 lateral 추종)
                eff_cfg.w_px      = 60.0;
                eff_cfg.w_py      = 60.0;
                eff_cfg.w_psi     = 3.0;    // yaw 거의 무시 (over-react 차단)
                eff_cfg.w_kappa   = 80.0;
                eff_cfg.w_akappa  = 70.0;
                eff_cfg.akappa_min = -0.08;
                eff_cfg.akappa_max =  0.08;
                eff_cfg.target_velocity = (cur_gear_ < 0 ? -1.0 : 1.0) * 18.0 / 3.6;
            } else if (in_avoid_cooldown) {
                // 회피 cooldown — yaw 정렬 강화 (path 원본 방향 복귀)
                eff_cfg.w_px      = 30.0;   // cte 추종 적정
                eff_cfg.w_py      = 30.0;
                eff_cfg.w_psi     = 40.0;   // yaw 강화 (path tangent 정렬 → path 복귀)
                eff_cfg.w_kappa   = 20.0;
                eff_cfg.w_akappa  = 100.0;  // 진동 방지
                eff_cfg.akappa_min = -0.05;
                eff_cfg.akappa_max =  0.05;
                eff_cfg.target_velocity = (cur_gear_ < 0 ? -1.0 : 1.0) * 22.0 / 3.6;
            } else if (straight && v_kmh_now > 20.0) {
                // 일반 직선 고속 (안전 default — 추종 안정성 유지)
                eff_cfg.w_px      = 15.0;
                eff_cfg.w_py      = 15.0;
                eff_cfg.w_psi     = 16.0;
                eff_cfg.w_kappa   = 12.0;
                eff_cfg.w_akappa  = 90.0;
                eff_cfg.akappa_min = -0.04;
                eff_cfg.akappa_max =  0.04;
            } else if (!straight) {
                // 곡선 분기 — path 끝부근 R=30~50m 영역 cte tracking 강화
                const bool sharp = (max_kappa_ahead > 0.15);
                const double cte_abs = std::abs(near.signed_cte);
                const double cte_boost = std::clamp(1.0 + 1.2 * std::max(0.0, cte_abs - 0.2), 1.0, 2.4);
                eff_cfg.w_px      = 18.0 * cte_boost;  // 8→18: 곡선 cte 누적 (path 끝 sharp curve) 방지
                eff_cfg.w_py      = 18.0 * cte_boost;
                eff_cfg.w_psi     = sharp ? 28.0 : 22.0;
                eff_cfg.w_kappa   = sharp ? 14.0 : 12.0;
                const double base_akappa = sharp ? 30.0 : 25.0;
                eff_cfg.w_akappa = base_akappa + 1.5 * std::max(0.0, v_kmh_now - 10.0);
                if (sharp) {
                    eff_cfg.akappa_min = -0.10;
                    eff_cfg.akappa_max =  0.10;
                } else {
                    eff_cfg.akappa_min = -0.12;
                    eff_cfg.akappa_max =  0.12;
                }
            }
        }
        rti_nmpc_->setConfig(eff_cfg);

        // ── 참조 경로 시퀀스 (PoseStamped) — 현재 segment 내부로 한정 ──
        int seg_lo = 0, seg_hi = (int)wp_x_.size() - 1;
        if (!gear_segments_.empty() && cur_segment_ < (int)gear_segments_.size()) {
            seg_lo = gear_segments_[cur_segment_].first;
            seg_hi = gear_segments_[cur_segment_].second;
        }
        // 회피 cooldown / RECOV: ref_path를 ego 앞 lookahead 지점부터 시작.
        // nearest_idx 직접 사용 시 NMPC가 ego의 lateral 위치를 강제 fit하려고 큰 yaw 변화
        // → overshoot. lookahead로 ego가 부드럽게 path에 합류하는 trajectory 생성.
        if (obs_cooldown) {
            int lookahead_steps = (int)std::round(3.0 / wp_spacing_);  // 3m 앞
            seg_lo = std::max(seg_lo, nearest_idx_ + lookahead_steps);
            seg_lo = std::min(seg_lo, seg_hi);
        } else if (in_recov_ && cur_gear_ > 0) {
            // RECOV: 짧은 lookahead (이전 15m → 5m). NMPC가 가까운 path 따라가 부드럽게 합류
            double la_m = std::min(5.0, 1.5 + std::abs(near.signed_cte) * 0.4);
            int lookahead_steps = (int)std::round(la_m / wp_spacing_);
            seg_lo = std::max(seg_lo, nearest_idx_ + lookahead_steps);
            seg_lo = std::min(seg_lo, seg_hi);
            rec["recov_la_m"] = la_m;
        }
        // R 시 vehicle yaw는 path_yaw + π여야 정합 → ref yaw에도 +π 적용
        // R 진입 동기화: 처음 2m 이동 동안 ψ_ref를 entry_yaw → wp_h+π로 blend
        double yaw_off = (cur_gear_ < 0) ? M_PI : 0.0;
        double align_blend = 1.0;  // 1.0 = 완전 path 따름, 0 = entry_yaw 유지
        if (r_align_active_) {
            double moved = std::hypot(cur_x_ - r_align_x0_, cur_y_ - r_align_y0_);
            align_blend = std::min(1.0, moved / kAlignDist);
            if (align_blend >= 0.999) {
                r_align_active_ = false;
                ROS_INFO("[PathFollower] R yaw 동기화 완료 (이동거리 %.2fm)", moved);
            }
        }
        auto wrapPi = [](double a){ while (a>M_PI) a-=2*M_PI; while (a<-M_PI) a+=2*M_PI; return a; };
        std::vector<geometry_msgs::PoseStamped> ref_path;
        ref_path.reserve(seg_hi - seg_lo + 1);
        for (int idx = seg_lo; idx <= seg_hi; ++idx) {
            geometry_msgs::PoseStamped ps;
            ps.pose.position.x = wp_x_[idx];
            ps.pose.position.y = wp_y_[idx];
            double yaw_target = wp_h_[idx] + yaw_off;
            // R 동기화 blend: yaw 시작점 = entry_yaw, 종착 = yaw_target
            double yaw;
            if (r_align_active_) {
                double diff = wrapPi(yaw_target - r_entry_yaw_);
                yaw = r_entry_yaw_ + align_blend * diff;
            } else {
                yaw = yaw_target;
            }
            ps.pose.orientation.z = std::sin(yaw * 0.5);
            ps.pose.orientation.w = std::cos(yaw * 0.5);
            ref_path.push_back(ps);
        }

        // ── ego pose 구성 ──
        // ── NMPC actuator delay 보상 (lookahead shift, 논문 기반)
        //    회피 영역 detect (in_avoid scope 다름 — 여기서 재계산)
        bool ego_in_avoid = false;
        if (!wp_avoid_off_.empty()) {
            int la_fwd  = std::min((int)wp_avoid_off_.size() - 1, nearest_idx_ + 30);
            int la_back = std::max(0, nearest_idx_ - 30);
            for (int i = la_back; i <= la_fwd; ++i) {
                if (wp_avoid_off_[i] > 0.1) { ego_in_avoid = true; break; }
            }
        }
        constexpr double kActuatorLag = 0.12;  // [s] sim 차량 steer servo lag
        double lookahead_x = cur_x_;
        double lookahead_y = cur_y_;
        double lookahead_yaw = cur_yaw_;
        if (ego_in_avoid) {
            lookahead_x   = cur_x_ + cur_v_signed_ * std::cos(cur_yaw_) * kActuatorLag;
            lookahead_y   = cur_y_ + cur_v_signed_ * std::sin(cur_yaw_) * kActuatorLag;
            lookahead_yaw = cur_yaw_ + cur_v_signed_ * current_kappa_ * kActuatorLag;
        }
        geometry_msgs::Pose ego_pose;
        ego_pose.position.x = lookahead_x;
        ego_pose.position.y = lookahead_y;
        ego_pose.orientation.z = std::sin(lookahead_yaw * 0.5);
        ego_pose.orientation.w = std::cos(lookahead_yaw * 0.5);

        // 장애물 NMPC stage 제약 — 차량 현재 위치 기준 전방 path 위 NPC만 활성
        // (1) lateral d 필터: |d_path| ≤ 2m + NPC half (다른 차선 무시)
        // (2) longitudinal idx 필터: NPC의 path 매칭점이 차량 nearest_idx ± lookahead 안 (차선 겹침 / 뒤편 NPC 무시)
        if (rti_cfg_.obs_enable && !obstacles_.empty()) {
            const int n_wp = (int)wp_x_.size();
            // 차량 진행 전방 lookahead window [현재, 현재+N], wp_spacing 0.3m 기준 ~30m (NMPC obs_active_dist 8m + 여유)
            const int idx_lookahead = std::max(20, (int)(30.0 / std::max(wp_spacing_, 0.1)));
            const int idx_lo = std::max(0, nearest_idx_ - 5);
            const int idx_hi = std::min(n_wp - 1, nearest_idx_ + idx_lookahead);

            auto projectPath = [&](double ox, double oy) {
                // 차량 전방 window 내에서만 nearest wp 검색 (전체 path 검색 시 차선 겹침 잘못 매칭)
                double best_d2 = 1e18; int best = idx_lo;
                for (int i = idx_lo; i <= idx_hi; ++i) {
                    double dx = wp_x_[i] - ox, dy = wp_y_[i] - oy;
                    double dd = dx*dx + dy*dy;
                    if (dd < best_d2) { best_d2 = dd; best = i; }
                }
                int bi1 = std::min(best + 1, n_wp - 1);
                double th = std::atan2(wp_y_[bi1] - wp_y_[best], wp_x_[bi1] - wp_x_[best]);
                double rx = ox - wp_x_[best], ry = oy - wp_y_[best];
                double d = -std::sin(th) * rx + std::cos(th) * ry;
                return std::make_pair(best, d);
            };

            std::vector<RTINMPCObstacle> nmpc_obs;
            nmpc_obs.reserve(obstacles_.size());
            for (const auto& o : obstacles_) {
                auto [obs_idx, d_proj] = projectPath(o.x, o.y);
                // (1) lateral 필터 — 다른 차선 무시
                double lat_half = std::max(o.sy, 1.0) * 0.5;
                if (std::abs(d_proj) > 2.0 + lat_half) continue;
                // (2) longitudinal 필터 — 매칭점이 lookahead 끝점이면 실제로는 path 밖 NPC (포함 안 함)
                if (obs_idx >= idx_hi - 1) continue;
                // (3) 차량 뒤편 NPC 무시 (현재 idx 이전)
                if (obs_idx < nearest_idx_ - 2) continue;
                RTINMPCObstacle no;
                no.cx = o.x;
                no.cy = o.y;
                const double r_obs = 0.5 * std::hypot(std::max(o.sx, 1.0),
                                                      std::max(o.sy, 1.0));
                no.r_safe = r_obs + rti_cfg_.obs_safe_margin;
                no.vx = o.vx;
                no.vy = o.vy;
                nmpc_obs.push_back(no);
            }
            rti_nmpc_->setObstacles(nmpc_obs);
        } else {
            rti_nmpc_->setObstacles({});
        }

        // RTI에 전달할 v는 signed (R이면 음수)
        double v_signed_in = (cur_gear_ < 0 ? -1.0 : 1.0) * std::abs(cur_v_);
        RTINMPCCommand cmd = rti_nmpc_->computeControl(ego_pose, ref_path, v_signed_in);

        if (cmd.solved) {
            double raw_steer_deg = cmd.steer_deg;
            double steer_deg_lim = steerRateLimit(raw_steer_deg, dt);
            double final_steer_rad = steer_deg_lim * M_PI / 180.0;
            prev_steer_ = steer_deg_lim;

            // 속도는 RTI 적분 v_cmd 무시 — target 직접 사용 (저속 stop 방지)
            double v_pub = velocitySigmoid(v_mag_kmh / 3.6, dt) * 3.6;
            publishCmd(v_pub, final_steer_rad);

            rec["steer_cmd"]      = final_steer_rad;
            rec["target_vel"]     = v_target_mps * 3.6;
            rec["rti_v_cmd"]      = cmd.v_cmd;
            rec["rti_kappa_cmd"]  = cmd.kappa_cmd;
            rec["rti_solver_us"]  = cmd.solver_time_us;
            rec["rti_model_us"]   = cmd.model_time_us;
        } else {
            rec["solve_failed"] = true;
            // solve 실패 시 이전 steer 유지 + 감속 (steer=0 직진 → 곡선 중 시각적 튐 방지)
            double v_pub = velocitySigmoid(v_mag_kmh / 3.6 * 0.5, dt) * 3.6;  // 절반 감속
            double fallback_steer_rad = prev_steer_ * M_PI / 180.0;
            publishCmd(v_pub, fallback_steer_rad);
            rec["steer_cmd"] = fallback_steer_rad;
        }

        // 성능 + status 토픽
        {
            std_msgs::Float32MultiArray perf;
            perf.data.resize(7);
            perf.data[0] = static_cast<float>(near.dist);
            perf.data[1] = static_cast<float>(cmd.solver_time_us / 1000.0);  // ms
            perf.data[2] = static_cast<float>(near.signed_cte);
            perf.data[3] = static_cast<float>(near.heading_err * 180.0 / M_PI);
            perf.data[4] = static_cast<float>(cur_v_ * 3.6);
            perf.data[5] = static_cast<float>(max_kappa_ahead);
            perf.data[6] = static_cast<float>(v_target_mps * 3.6);
            perf_pub_.publish(perf);

            std_msgs::String status;
            const char* fwd_mode_s = (force_nmpc_ && !in_low_speed_) ? "NMPC_HS" : "NMPC_LO";
            status.data = (cur_gear_ < 0) ? "NMPC_R" : fwd_mode_s;
            status_pub_.publish(status);
        }
        log_recs_.push_back(rec);
        return;
    }

    rec["mode"] = "NORMAL";
    rec["gear"] = "D";
    rec["controller"] = "LTV";
    // ── NORMAL MPC (Mobility Structure) ──────────────────────────
    double theta_ref = wp_h_[nearest_idx_], kappa_ref = wp_k_[nearest_idx_];
    double dx = cur_x_ - wp_x_[nearest_idx_], dy = cur_y_ - wp_y_[nearest_idx_];
    double dr = -std::sin(theta_ref) * dx + std::cos(theta_ref) * dy;
    // 후진 시: 차량 heading이 경로 방향과 π 차이 → 보정
    double delta_theta = (cur_gear_ < 0)
        ? wrapAngle(cur_yaw_ - theta_ref + M_PI)
        : wrapAngle(cur_yaw_ - theta_ref);

    Eigen::VectorXd x0(kNx); x0 << dr, theta_ref + delta_theta, current_kappa_, theta_ref, kappa_ref;

    // ── Curvature-based velocity profile ────────────────────────
    double v_ref_calc = std::max(2.0 / 3.6, cur_v_);
    int idx_per_step_v = std::max(1, (int)std::round(v_ref_calc * cfg_.Ts / wp_spacing_));
    std::vector<double> v_profile(cfg_.N);
    for (int i = 0; i < cfg_.N; ++i) {
        // 예측 구간 내 최대 곡률 (lookahead 윈도우, segment 내부로 제한)
        int la_steps = std::max(1, (int)std::round(cfg_.curve_lookahead_m / wp_spacing_));
        int base_idx = nearest_idx_ + (i + 1) * idx_per_step_v;
        double max_k = 0.0;
        for (int j = 0; j <= la_steps; ++j) {
            int ki = std::min(base_idx + j, seg_hi_for_la);
            max_k = std::max(max_k, std::abs(wp_k_[ki]));
            if (ki == seg_hi_for_la) break;
        }
        // 곡률 비례 연속 감속
        double alpha = 20.0;
        double v_max_for_seg = cfg_.target_vel;  // 기본 60 km/h
        if (parking_mode_) {
            // 주차 모드 — 곡률 인지 cap (D2 출렁임 fix)
            //   직선부 (κ<0.05) : 2 km/h (정밀 유지)
            //   곡선부 (κ≥0.10) : 5 km/h (yaw rate 충분히 확보 — ω=v·κ가 너무 작아 cte 누적되는 문제)
            //   ramp 0.05→0.10 선형 보간
            double parking_cap_kmh;
            if (max_k < 0.05) parking_cap_kmh = parking_max_kmh_;
            else if (max_k >= 0.10) parking_cap_kmh = 5.0;
            else {
                double kk = (max_k - 0.05) / (0.10 - 0.05);
                parking_cap_kmh = parking_max_kmh_ + (5.0 - parking_max_kmh_) * kk;
            }
            v_max_for_seg = parking_cap_kmh / 3.6;
        }
        double v_target = v_max_for_seg / (1.0 + alpha * max_k);
        v_target = std::max(cfg_.curve_min_vel, v_target);
        // D 끝 사전감속 (정밀 정렬 — 끝점에서 yaw 안정 도달)
        if (cur_gear_ > 0 && cur_segment_ + 1 < (int)gear_segments_.size()) {
            int next_g = wp_gear_[gear_segments_[cur_segment_ + 1].first];
            if (next_g != cur_gear_) {
                int seg_e = gear_segments_[cur_segment_].second;
                double d_remain = 0.0;
                int idx_pred = std::min(nearest_idx_ + (i + 1) * idx_per_step_v, seg_e);
                for (int kk = idx_pred; kk < seg_e; ++kk) {
                    d_remain += std::sqrt(std::pow(wp_x_[kk + 1] - wp_x_[kk], 2.0)
                                          + std::pow(wp_y_[kk + 1] - wp_y_[kk], 2.0));
                }
                // 정밀 단계별 감속 (parking 모드 v_max_for_seg = 0.83 m/s = 3 km/h 기준):
                //   d > 20m  : v_max_for_seg (3 km/h)
                //   d 5m     : 1.5 km/h (0.42)
                //   d 1m     : 0.5 km/h (0.14)
                //   d 0.3m   : 0.3 km/h (0.083) crawl
                double v_max_appr;
                if (d_remain > 20.0) {
                    v_max_appr = v_max_for_seg;
                } else if (d_remain > 5.0) {
                    v_max_appr = 0.42 + (d_remain - 5.0) * (v_max_for_seg - 0.42) / (20.0 - 5.0);
                } else if (d_remain > 1.0) {
                    v_max_appr = 0.14 + (d_remain - 1.0) * (0.42 - 0.14) / (5.0 - 1.0);
                } else if (d_remain > 0.3) {
                    v_max_appr = 0.083 + (d_remain - 0.3) * (0.14 - 0.083) / (1.0 - 0.3);
                } else {
                    v_max_appr = 0.083;
                }
                v_target = std::min(v_target, v_max_appr);
            }
        }
        // 후진 시 (LTV는 R 모드 안 들어옴, 안전 코드)
        if (cur_gear_ < 0) {
            v_target = std::min(v_target, reverse_max_vel_kmh_ / 3.6);
            v_target = -v_target;
        }
        v_profile[i] = v_target;
    }
    Eigen::MatrixXd A_bar, B_bar, E_bar; model_->buildBatchMatrices(v_profile, A_bar, B_bar, E_bar);

    // ── Build z_bar (Future Curvature Changes) ──────────────────
    double v_ref = std::max(2.0 / 3.6, cur_v_); // Min 2km/h for index calculation
    int idx_per_step = std::max(1, (int)std::round(v_ref * cfg_.Ts / wp_spacing_));
    Eigen::VectorXd z_bar = Eigen::VectorXd::Zero(cfg_.N);
    for (int i = 0; i < cfg_.N; ++i) {
        int ki  = std::min(nearest_idx_ + (i + 1) * idx_per_step, (int)wp_k_.size() - 1);
        int ki0 = std::min(nearest_idx_ +  i      * idx_per_step, (int)wp_k_.size() - 1);
        z_bar(i) = (wp_k_[ki] - wp_k_[ki0]) / cfg_.Ts;
    }

    Eigen::VectorXd x_free = A_bar * x0 + E_bar * z_bar;

    Eigen::SparseMatrix<double> P; Eigen::VectorXd q_vec;
    cost_->buildQPObjective(x0, z_bar, A_bar, B_bar, E_bar, v_profile, max_kappa_ahead, P, q_vec);

    // [Structure] Physical Constraint Synchronization (Paper & Mobility Style)
    // 1. Dynamic Input Limit based on max_steer_rate
    const double cur_steer_rad = std::atan(current_kappa_ * cfg_.L);
    const double cos_s = std::cos(cur_steer_rad);
    // kappa_dot = (sec^2(delta) * delta_dot) / L
    double k_dot_limit = (max_steer_rate_ * M_PI / 180.0) / (cfg_.L * cos_s * cos_s + 1e-6);
    double u_lim = k_dot_limit; // Now u is pure kappa_dot

    // ── Frenet d corridor 계산 (Phase 1+2) ────────────────────────
    // 회피 모드 ON + 전진 시에만 활성화 (후진/주차에서는 disable)
    std::vector<double> d_min_k, d_max_k;
    double v_scale_obs = 1.0;
    const bool obs_active = avoidance_enabled_ && (cur_gear_ > 0);
    if (obs_active) {
        buildObstacleCorridor(v_profile, d_min_k, d_max_k, v_scale_obs);
    }

    // 2. Build Constraint Matrix
    // Decision = [u (N); s_slack (N)]   — 회피 비활성 시에는 slack 항 0 weight로 무시
    const int N = cfg_.N;
    const int n_dec = 2 * N;
    const int n_row = 5 * N;     // u, κ, dr_max+slack, dr_min+slack, slack≥0
    const double INF = 1.0e10;
    Eigen::MatrixXd A_cons_dense = Eigen::MatrixXd::Zero(n_row, n_dec);
    // row 0..N-1: u bound  →  [I | 0]
    A_cons_dense.block(0,     0, N, N).setIdentity();
    // row N..2N-1: κ bound →  [B_κ | 0]
    for (int k = 0; k < N; ++k) {
        A_cons_dense.block(N + k, 0, 1, N) = B_bar.block(k * kNx + 2, 0, 1, N);
    }
    // row 2N..3N-1: dr_max + slack →  [B_dr | -I]  (Ax ≤ dr_max - dr_free)
    // row 3N..4N-1: dr_min + slack →  [B_dr | +I]  (Ax ≥ dr_min - dr_free)
    for (int k = 0; k < N; ++k) {
        A_cons_dense.block(2*N + k, 0,  1, N) = B_bar.block(k * kNx + 0, 0, 1, N);
        A_cons_dense.block(2*N + k, N + k, 1, 1) << -1.0;
        A_cons_dense.block(3*N + k, 0,  1, N) = B_bar.block(k * kNx + 0, 0, 1, N);
        A_cons_dense.block(3*N + k, N + k, 1, 1) << +1.0;
    }
    // row 4N..5N-1: slack ≥ 0  → [0 | I]
    A_cons_dense.block(4*N, N, N, N).setIdentity();
    Eigen::SparseMatrix<double> A_cons = A_cons_dense.sparseView();

    Eigen::VectorXd l_cons(n_row), u_cons(n_row);
    // u bound
    l_cons.segment(0, N) = Eigen::VectorXd::Constant(N, -u_lim);
    u_cons.segment(0, N) = Eigen::VectorXd::Constant(N,  u_lim);
    // κ bound
    for (int k = 0; k < N; ++k) {
        double k_free = x_free(k * kNx + 2);
        l_cons(N + k) = cfg_.kappa_min - k_free;
        u_cons(N + k) = cfg_.kappa_max - k_free;
    }
    // dr bound + slack
    for (int k = 0; k < N; ++k) {
        double dr_free = x_free(k * kNx + 0);
        double d_max_v = obs_active ? d_max_k[k] :  INF;
        double d_min_v = obs_active ? d_min_k[k] : -INF;
        // dr_max + slack: -∞ ≤ B_dr*u - s ≤ d_max - dr_free
        l_cons(2*N + k) = -INF;
        u_cons(2*N + k) = d_max_v - dr_free;
        // dr_min + slack: d_min - dr_free ≤ B_dr*u + s ≤ +∞
        l_cons(3*N + k) = d_min_v - dr_free;
        u_cons(3*N + k) = INF;
    }
    // slack ≥ 0
    l_cons.segment(4*N, N) = Eigen::VectorXd::Zero(N);
    u_cons.segment(4*N, N) = Eigen::VectorXd::Constant(N, INF);

    // ── P, q 확장 (slack quadratic + linear penalty) ──────────────
    // 기존 P (N×N), q (N) → 2N×2N, 2N로 확장. slack block: diag(w_slack_quad)
    Eigen::SparseMatrix<double> P_ext(n_dec, n_dec);
    P_ext.reserve(P.nonZeros() + N);
    std::vector<Eigen::Triplet<double>> trips;
    trips.reserve(P.nonZeros() + N);
    for (int kcol = 0; kcol < P.outerSize(); ++kcol) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(P, kcol); it; ++it) {
            trips.emplace_back(it.row(), it.col(), it.value());
        }
    }
    // slack quadratic — 회피 활성 시에만 실제 weight (비활성 시 0 → infeasible 무관)
    double w_sq = obs_active ? cfg_.w_slack_quad : 0.0;
    double w_sl = obs_active ? cfg_.w_slack_lin  : 0.0;
    for (int k = 0; k < N; ++k) {
        trips.emplace_back(N + k, N + k, 2.0 * w_sq);   // 0.5 * 2w * s^2 = w*s^2
    }
    P_ext.setFromTriplets(trips.begin(), trips.end());
    P_ext.makeCompressed();

    Eigen::VectorXd q_ext(n_dec);
    q_ext.head(N) = q_vec;
    q_ext.tail(N) = Eigen::VectorXd::Constant(N, w_sl);

    Eigen::VectorXd sol;
    if (solver_->solve(P_ext, q_ext, A_cons, l_cons, u_cons, sol) && sol.size() >= N) {
        current_kappa_ += sol[0] * cfg_.Ts;
        current_kappa_ = std::clamp(current_kappa_, cfg_.kappa_min, cfg_.kappa_max);

        double steer_rad = std::atan(current_kappa_ * cfg_.L);
        // Apply kappa_gain for MORAI responsiveness
        double raw_steer = steer_rad * cfg_.kappa_gain;

        // Apply rate limit on degrees
        double steer_deg_limited = steerRateLimit(raw_steer * 180.0 / M_PI, dt);
        double final_steer_rad = steer_deg_limited * M_PI / 180.0;

        prev_steer_ = steer_deg_limited;
        // 회피 활성 + NPC 가까우면 추가 감속
        // 회피 활성 중: 거리 기반 강한 감속. 정렬 cooldown 중: yaw 복귀 가능 속도 유지.
        double v_factor = 1.0;
        if (obs_active) {
            v_factor = v_scale_obs;            // 회피 중 거리 기반 감속
        } else if (avoidance_enabled_ && obs_block_until_align_) {
            v_factor = cfg_.obs_cooldown_v_scale;   // 정렬까지 0.30 유지 (yaw 복귀용)
        }
        double v_cmd_mps = std::abs(v_profile[0]) * v_factor;
        double cmd_vel = velocitySigmoid(v_cmd_mps, dt);
        publishCmd(cmd_vel * 3.6, final_steer_rad);
        rec["steer_cmd"] = final_steer_rad; rec["current_kappa"] = current_kappa_;
        rec["target_vel"] = v_profile[0] * 3.6 * v_factor;
        rec["predicted_cte"] = dr + sol[0] * cfg_.Ts;
        if (obs_active) {
            rec["obs_d_min"]   = last_d_min_;
            rec["obs_d_max"]   = last_d_max_;
            rec["obs_dist_s"]  = last_obs_dist_s_;
            rec["obs_v_scale"] = v_scale_obs;
            // slack 합 (회피 가용성 지표; 클수록 corridor 위반)
            double slack_sum = 0.0;
            for (int k = 0; k < N; ++k) slack_sum += std::abs(sol[N + k]);
            rec["obs_slack_sum"] = slack_sum;
        }
    } else {
        rec["solve_failed"] = true;
    }
    
    auto t_end = ros::WallTime::now();
    rec["solve_ms"] = (t_end - t_start).toSec() * 1000.0;

    // Additional diagnostics
    rec["lateral_accel"] = cur_v_ * cur_v_ * std::abs(wp_k_[nearest_idx_]);
    if (cmd_init_ && dt > 1e-4) {
        double steer_now = rec.isMember("steer_cmd") ? rec["steer_cmd"].asDouble() * 180.0 / M_PI : prev_steer_;
        rec["steering_rate"] = (steer_now - prev_steer_) / dt;
    }
    rec["yaw_rate"] = cur_v_ * current_kappa_;

    // ── Publish real-time performance topic ─────────────────────
    {
        std_msgs::Float32MultiArray perf;
        perf.data.resize(7);
        perf.data[0] = static_cast<float>(near.dist);
        perf.data[1] = static_cast<float>(rec["solve_ms"].asDouble());
        perf.data[2] = static_cast<float>(near.signed_cte);
        perf.data[3] = static_cast<float>(near.heading_err * 180.0 / M_PI);
        perf.data[4] = static_cast<float>(cur_v_ * 3.6);
        perf.data[5] = static_cast<float>(max_kappa_ahead);
        perf.data[6] = static_cast<float>(v_profile[0] * 3.6);
        perf_pub_.publish(perf);

        std_msgs::String status;
        status.data = (cur_gear_ < 0) ? "NORMAL_R" : "NORMAL";
        status_pub_.publish(status);
    }

    // ── Log every tick (no filter) ───────────────────────────────
    log_recs_.push_back(rec);

    if (++log_tick_ % 200 == 0) flushLog();
}

void PathFollower::publishCmd(double vel_kmh, double steer_deg) {
    morai_msgs::CtrlCmd cmd; cmd.longlCmdType = 2;
    // 후진 시에도 MORAI에는 양수 속도 전달 (기어가 R이면 자동 후진)
    cmd.velocity = std::abs(vel_kmh); cmd.steering = steer_deg;
    ctrl_pub_.publish(cmd);
}

} // namespace moraimpc

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower_node");
    ros::NodeHandle nh("~");
    moraimpc::PathFollower node(nh);
    ros::spin();
    return 0;
}
