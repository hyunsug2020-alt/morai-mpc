#include "moraimpc/controller_consistency.hpp"
#include "moraimpc/junction_curve.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <Eigen/Sparse>
#include <unsupported/Eigen/MatrixFunctions>

namespace moraimpc {

// ═══════════════════════════════════════════════════════════════════════
// C1 measurement
// ═══════════════════════════════════════════════════════════════════════
WheelUnit parseWheelUnit(const std::string& text) {
    if (text == "deg" || text == "degree" || text == "degrees") return WheelUnit::DEG;
    if (text == "rad" || text == "radian" || text == "radians") return WheelUnit::RAD;
    return WheelUnit::UNSET;
}

const char* wheelUnitName(WheelUnit unit) {
    switch (unit) {
        case WheelUnit::DEG: return "deg";
        case WheelUnit::RAD: return "rad";
        default: return "unset";
    }
}

WheelMeasurement evaluateWheel(bool received, double raw, double age_s,
                               const WheelParams& params,
                               bool have_emitted, double emitted_prev_rad,
                               double measured_speed_mps, double dt,
                               WheelTracker& tracker) {
    WheelMeasurement m;
    m.raw = raw;
    m.age_s = age_s;
    if (params.unit == WheelUnit::UNSET) { m.reason = "unit-unset"; return m; }
    if (!received) { m.reason = "missing"; return m; }
    if (!std::isfinite(raw)) { m.reason = "nonfinite"; return m; }
    if (!std::isfinite(age_s) || age_s < 0.0 || age_s > params.timeout_s) {
        m.reason = "stale";
        return m;
    }
    const double delta = (params.unit == WheelUnit::DEG) ? raw * M_PI / 180.0 : raw;
    const double limit = (params.physical_max_deg + params.tolerance_deg) * M_PI / 180.0;
    if (std::abs(delta) > limit) { m.reason = "out-of-range"; return m; }
    m.delta_rad = delta;
    // Residual policy: a live feed must follow the emitted road-wheel command
    // while moving. A constant/frozen value (e.g. a bridge that never fills
    // wheel_angle) is detected by a sustained residual, not by its value.
    const double step = (std::isfinite(dt) && dt > 0.0) ? dt : 0.0;
    if (have_emitted && std::isfinite(emitted_prev_rad) &&
        measured_speed_mps > params.moving_mps &&
        std::abs(delta - emitted_prev_rad) > params.residual_rad) {
        tracker.residual_time_s += step;
    } else {
        tracker.residual_time_s = 0.0;   // clears as soon as the residual is back in bound
    }
    // Frozen policy: a moving vehicle whose emitted steering command changes must see the
    // measured value change; an exactly repeated value (e.g. a bridge that never fills the
    // field and sends 0) is flagged even when the commands are small. A constant straight
    // command with a constant 0 measurement is not flagged. Clears on the first change.
    const bool moving = measured_speed_mps > params.moving_mps;
    if (tracker.have_last && raw == tracker.last_raw && moving && have_emitted &&
        std::isfinite(emitted_prev_rad)) {
        if (tracker.unchanged_time_s == 0.0) {
            tracker.emitted_min = tracker.emitted_max = emitted_prev_rad;
        }
        tracker.unchanged_time_s += step;
        tracker.emitted_min = std::min(tracker.emitted_min, emitted_prev_rad);
        tracker.emitted_max = std::max(tracker.emitted_max, emitted_prev_rad);
    } else if (!tracker.have_last || raw != tracker.last_raw) {
        tracker.unchanged_time_s = 0.0;
    }
    tracker.have_last = true;
    tracker.last_raw = raw;
    if (tracker.residual_time_s > params.residual_time_s) {
        m.reason = "residual";
        return m;
    }
    if (tracker.unchanged_time_s > params.frozen_time_s &&
        tracker.emitted_max - tracker.emitted_min > params.frozen_cmd_span_rad) {
        m.reason = "frozen";
        return m;
    }
    m.valid = true;
    m.reason = "ok";
    return m;
}

// ═══════════════════════════════════════════════════════════════════════
// C3 arc-length sampling
// ═══════════════════════════════════════════════════════════════════════
double wrapPi(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

namespace {
// Same normalisation as the former path_follower.cpp wrapAngle (bitwise legacy).
double loopWrap(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
}  // namespace

RefSample samplePath(const PathArrays& path, double station) {
    RefSample r;
    const auto& s = *path.s;
    const int lo = std::max(0, path.lo);
    const int hi = std::min(static_cast<int>(s.size()) - 1, path.hi);
    if (hi <= lo) {
        const int i = std::clamp(lo, 0, static_cast<int>(s.size()) - 1);
        r.s = s[i]; r.x = (*path.x)[i]; r.y = (*path.y)[i];
        r.heading = (*path.h)[i]; r.kappa = (*path.k)[i]; r.index = i;
        r.clamped = true;
        return r;
    }
    double q = station;
    if (!std::isfinite(q)) q = s[lo];
    if (q <= s[lo]) { q = s[lo]; r.clamped = station < s[lo]; }
    if (q >= s[hi]) { q = s[hi]; r.clamped = station > s[hi]; }
    // first index j in [lo, hi] with s[j] >= q
    auto first = s.begin() + lo;
    auto last = s.begin() + hi + 1;
    int j = static_cast<int>(std::lower_bound(first, last, q) - s.begin());
    j = std::clamp(j, lo + 1, hi);
    int i = j - 1;
    // skip zero-length (duplicate) intervals backwards
    while (i > lo && s[j] - s[i] < 1e-9) --i;
    const double span = s[j] - s[i];
    const double f = (span < 1e-9) ? 0.0 : std::clamp((q - s[i]) / span, 0.0, 1.0);
    r.s = q;
    r.index = i;
    r.x = (*path.x)[i] + f * ((*path.x)[j] - (*path.x)[i]);
    r.y = (*path.y)[i] + f * ((*path.y)[j] - (*path.y)[i]);
    r.heading = wrapPi((*path.h)[i] + f * wrapPi((*path.h)[j] - (*path.h)[i]));
    r.kappa = (*path.k)[i] + f * ((*path.k)[j] - (*path.k)[i]);
    return r;
}

// ═══════════════════════════════════════════════════════════════════════
// C2 reachable speeds
// ═══════════════════════════════════════════════════════════════════════
std::vector<double> reachableStageSpeeds(double v0, const std::vector<double>& targets,
                                         double accel, double decel, double Ts,
                                         std::vector<double>* nodes) {
    std::vector<double> stage(targets.size());
    double v = std::max(0.0, std::isfinite(v0) ? v0 : 0.0);
    if (nodes) { nodes->assign(1, v); }
    for (size_t i = 0; i < targets.size(); ++i) {
        const double target = std::max(0.0, targets[i]);
        const double dv = std::clamp(target - v, -std::abs(decel) * Ts, std::abs(accel) * Ts);
        const double next = std::max(0.0, v + dv);
        stage[i] = 0.5 * (v + next);
        v = next;
        if (nodes) nodes->push_back(v);
    }
    return stage;
}

// ═══════════════════════════════════════════════════════════════════════
// Legacy helpers (verbatim semantics of the former controlLoop lambdas)
// ═══════════════════════════════════════════════════════════════════════
namespace {

struct CurveHelpers {
    const LtvParams& p;
    const LtvInput& in;
    int seg_hi_for_la;

    double lookahead(double speed_mps) const {
        double base = std::max(p.cfg.curve_lookahead_m, p.curve_lookahead_min_m);
        if (!p.curve_decel_enable) return base;
        double v = std::max(0.0, speed_mps);
        double brake = std::max(0.5, p.curve_brake_decel_mps2);
        double dyn = v * p.curve_lookahead_time_s + (v * v) / (2.0 * brake);
        return std::clamp(std::max(base, dyn), p.curve_lookahead_min_m, p.curve_lookahead_max_m);
    }
    double maxKappaAhead(int base_idx, double la_m) const {
        const auto& k = *in.path.k;
        double max_k = 0.0;
        const double spacing = std::max(0.1, in.wp_spacing);
        int la_steps = std::max(1, (int)std::round(la_m / spacing));
        for (int i = 0; i <= la_steps; ++i) {
            int ki = std::min(base_idx + i, seg_hi_for_la);
            max_k = std::max(max_k, std::abs(k[ki]));
            if (ki == seg_hi_for_la) break;
        }
        return max_k;
    }
    double curveLimited(double v_max_mps, double max_k) const {
        if (!p.curve_decel_enable) return v_max_mps;
        if (max_k <= p.cfg.curve_kappa_thresh) return v_max_mps;
        double v_target = v_max_mps / (1.0 + std::max(0.0, p.curve_speed_alpha) * max_k);
        if (max_k > 1e-3) {
            double v_alat = std::sqrt(std::max(0.5, p.curve_alat_max) / max_k);
            v_target = std::min(v_target, v_alat);
        }
        return std::max(p.cfg.curve_min_vel, v_target);
    }
    double parkingCap(double max_k) const {
        double parking_cap_kmh;
        if (max_k < 0.05) parking_cap_kmh = in.parking_max_kmh;
        else if (max_k >= 0.10) parking_cap_kmh = 5.0;
        else {
            double kk = (max_k - 0.05) / (0.10 - 0.05);
            parking_cap_kmh = in.parking_max_kmh + (5.0 - in.parking_max_kmh) * kk;
        }
        return parking_cap_kmh / 3.6;
    }
    static double approach(double d_remain, double v_max_for_seg) {
        if (d_remain > 20.0) return v_max_for_seg;
        if (d_remain > 5.0) return 0.42 + (d_remain - 5.0) * (v_max_for_seg - 0.42) / (20.0 - 5.0);
        if (d_remain > 1.0) return 0.14 + (d_remain - 1.0) * (0.42 - 0.14) / (5.0 - 1.0);
        if (d_remain > 0.3) return 0.083 + (d_remain - 0.3) * (0.14 - 0.083) / (1.0 - 0.3);
        return 0.083;
    }
    bool gearChangeAhead(int* seg_e) const {
        if (in.cur_gear > 0 && in.cur_segment + 1 < (int)in.segments.size()) {
            int next_g = (*in.gear)[in.segments[in.cur_segment + 1].first];
            if (next_g != in.cur_gear) {
                *seg_e = in.segments[in.cur_segment].second;
                return true;
            }
        }
        return false;
    }
};

}  // namespace

// ═══════════════════════════════════════════════════════════════════════
// Lag-aware 6-state model: x = [dr, theta, kappa_p, theta_r, kappa_r, kappa_c],
// u = d(kappa_c)/dt, z = d(kappa_r)/dt, kappa_p' = (kappa_c - kappa_p)/tau.
// Cost identical to LTVCost on (dr, theta-theta_r, kappa_p-kappa_r) + R on u.
// ═══════════════════════════════════════════════════════════════════════
namespace {
constexpr int kNxLag = 6;

void lagBatch(const LTVMPCConfig& cfg, double tau, const std::vector<double>& v_profile,
              Eigen::MatrixXd& A_bar, Eigen::MatrixXd& B_bar, Eigen::MatrixXd& E_bar) {
    const int N = cfg.N, nx = kNxLag;
    std::vector<Eigen::MatrixXd> A(N), B(N), E(N);
    for (int i = 0; i < N; ++i) {
        const double v = v_profile[i];
        Eigen::MatrixXd M = Eigen::MatrixXd::Zero(nx + 2, nx + 2);
        M(0, 1) = v; M(0, 3) = -v; M(1, 2) = v; M(3, 4) = v;
        M(2, 2) = -1.0 / tau; M(2, 5) = 1.0 / tau;
        M(5, nx) = 1.0;          // u -> kappa_c
        M(4, nx + 1) = 1.0;      // z -> kappa_r
        Eigen::MatrixXd ex = (M * cfg.Ts).exp();
        A[i] = ex.block(0, 0, nx, nx); B[i] = ex.block(0, nx, nx, 1); E[i] = ex.block(0, nx + 1, nx, 1);
    }
    A_bar = Eigen::MatrixXd::Zero(nx * N, nx);
    B_bar = Eigen::MatrixXd::Zero(nx * N, N);
    E_bar = Eigen::MatrixXd::Zero(nx * N, N);
    for (int i = 0; i < N; ++i) {
        Eigen::MatrixXd prod = Eigen::MatrixXd::Identity(nx, nx);
        for (int j = 0; j <= i; ++j) prod = A[j] * prod;
        A_bar.block(i * nx, 0, nx, nx) = prod;
        for (int j = 0; j <= i; ++j) {
            Eigen::MatrixXd b = Eigen::MatrixXd::Identity(nx, nx);
            for (int k = j + 1; k <= i; ++k) b = A[k] * b;
            B_bar.block(i * nx, j, nx, 1) = b * B[j];
            E_bar.block(i * nx, j, nx, 1) = b * E[j];
        }
    }
}

void lagCost(const LTVMPCConfig& cfg, const Eigen::VectorXd& x_free, const Eigen::MatrixXd& B_bar,
             const std::vector<double>& v_profile, double max_kappa_ahead,
             Eigen::SparseMatrix<double>& P, Eigen::VectorXd& q) {
    const int N = cfg.N, nx = kNxLag;
    double v_avg = 0.0;
    for (int i = 0; i < N; ++i) v_avg += v_profile[i];
    v_avg /= N;
    double t_frac = std::clamp((v_avg - cfg.w_theta_v_low) / (cfg.w_theta_v_high - cfg.w_theta_v_low + 1e-6), 0.0, 1.0);
    double w_theta = cfg.w_theta_low_speed + t_frac * (cfg.w_theta_high_speed - cfg.w_theta_low_speed);
    double w_dr = cfg.w_dr * ((max_kappa_ahead > cfg.curve_kappa_thresh) ? cfg.w_dr_curve_boost : 1.0);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nx * N, nx * N);
    for (int i = 0; i < N; ++i) {
        Eigen::MatrixXd Qi = Eigen::MatrixXd::Zero(nx, nx);
        Qi(0, 0) = w_dr;
        Qi(1, 1) = w_theta; Qi(1, 3) = -w_theta; Qi(3, 1) = -w_theta; Qi(3, 3) = w_theta;
        Qi(2, 2) = cfg.w_kappa; Qi(2, 4) = -cfg.w_kappa; Qi(4, 2) = -cfg.w_kappa; Qi(4, 4) = cfg.w_kappa;
        Q.block(i * nx, i * nx, nx, nx) = Qi;
    }
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(N, N);
    for (int i = 0; i < N; ++i) R(i, i) = cfg.w_u + cfg.w_u_v_gain * v_profile[i];
    Eigen::MatrixXd H = 2.0 * (B_bar.transpose() * Q * B_bar + R);
    q = 2.0 * B_bar.transpose() * Q * x_free;
    Eigen::MatrixXd Pd = H.triangularView<Eigen::Upper>();
    P = Pd.sparseView();
}
}  // namespace

Eigen::MatrixXd tailWeight(double v, double Ts, bool lag, double tau, double w_dr, double w_theta,
                           double w_kappa, double r, int M) {
    const int ne = lag ? 4 : 3;
    const int nx = lag ? kNxLag : kNx;
    // error model e = [dr, theta-theta_r, kappa_p-kappa_r (, kappa_c-kappa_r)], z = 0 after N
    Eigen::MatrixXd Mc = Eigen::MatrixXd::Zero(ne + 1, ne + 1);
    Mc(0, 1) = v; Mc(1, 2) = v;
    if (lag) { Mc(2, 2) = -1.0 / tau; Mc(2, 3) = 1.0 / tau; Mc(3, ne) = 1.0; }
    else     { Mc(2, ne) = 1.0; }
    const Eigen::MatrixXd ex = (Mc * Ts).exp();
    const Eigen::MatrixXd A = ex.block(0, 0, ne, ne);
    const Eigen::VectorXd B = ex.block(0, ne, ne, 1);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(ne, ne);
    Q(0, 0) = w_dr; Q(1, 1) = w_theta; Q(2, 2) = w_kappa;
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(ne, ne);
    for (int j = 0; j < M; ++j) {
        const Eigen::MatrixXd S = Q + P;
        const Eigen::RowVectorXd SB_t = (S * B).transpose();
        const double den = r + B.dot(S * B);
        P = A.transpose() * S * A - (A.transpose() * S * B) * (SB_t * A) / den;
        P = 0.5 * (P + P.transpose());
    }
    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(ne, nx);
    T(0, 0) = 1.0;
    T(1, 1) = 1.0; T(1, 3) = -1.0;
    T(2, 2) = 1.0; T(2, 4) = -1.0;
    if (lag) { T(3, 5) = 1.0; T(3, 4) = -1.0; }
    return T.transpose() * P * T;
}

std::string validateLtvConfig(const LtvParams& p, WheelUnit unit) {
    const LtvFlags& f = p.flags;
    if (f.measured_state && f.emitted_command_state)
        return "steering state source must be exactly one of measured/emitted/command-integral";
    if (f.measured_state && unit == WheelUnit::UNSET)
        return "steering_state_source=measured requires wheel_angle_unit deg|rad";
    if (!std::isfinite(p.actuator_tau_model) || p.actuator_tau_model < 0.0 || p.actuator_tau_model > 1.0)
        return "actuator_tau_model must be within [0, 1] s";
    if (p.actuator_tau_model > 1e-3 && !f.physical_output)
        return "actuator lag model requires steer_output_mode=physical (legacy output gain changes the modelled curvature)";
    if (!std::isfinite(p.tail_horizon_s) || p.tail_horizon_s < 0.0 || p.tail_horizon_s > 20.0)
        return "tail_horizon_s must be within [0, 20] s";
    if (!std::isfinite(p.steering_output_gain) || p.steering_output_gain <= 0.0)
        return "steering_output_gain must be finite and > 0";
    if (!(p.max_steer_deg > 0.0) || !(p.max_steer_rate_deg_s > 0.0))
        return "max_steer_deg and max_steer_rate must be > 0";
    return "";
}

// ═══════════════════════════════════════════════════════════════════════
// Forward-LTV step
// ═══════════════════════════════════════════════════════════════════════
LtvOutput ltvStep(const LtvParams& p, const LtvInput& in, LtvPersistent& st,
                  LTVModel& model, LTVCost& cost, LTVSolver& solver) {
    LtvOutput out;
    const auto& cfg = p.cfg;
    const auto& wx = *in.path.x;
    const auto& wy = *in.path.y;
    const auto& wh = *in.path.h;
    const auto& wk = *in.path.k;
    const auto& ws = *in.path.s;
    const int N = cfg.N;
    const int nearest = in.nearest_idx;
    int seg_hi_for_la = (int)wk.size() - 1;
    if (!in.segments.empty() && in.cur_segment < (int)in.segments.size()) {
        seg_hi_for_la = in.segments[in.cur_segment].second;
    }
    CurveHelpers curve{p, in, seg_hi_for_la};
    const LtvFlags& f = p.flags;
    std::string fallback;

    out.str["preview_mode"] = f.station_preview ? "station" : "index";
    out.str["velocity_contract"] = f.reachable_velocity ? "reachable" : "legacy";
    out.str["output_mode"] = f.physical_output ? "physical" : "legacy_gain";

    // ── C1: steering state ──────────────────────────────────────────────
    double kappa_state = st.current_kappa;
    std::string state_source = "command-integral";
    bool wheel_fallback_cap = false;
    bool no_state_stop = false;
    const bool lag_model = (f.measured_state || f.emitted_command_state) && p.actuator_tau_model > 1e-3;
    const bool have_emit = in.have_emitted && std::isfinite(in.emitted_steer_rad);
    // Actuator observer (revision2): physical road-wheel curvature estimated from the command
    // actually emitted during the last interval. A MODEL estimate (tau_model is an assumption).
    {
        const double k_emit_phys = have_emit ? std::tan(in.emitted_steer_rad) / cfg.L
                                             : std::numeric_limits<double>::quiet_NaN();
        if (!st.obs_init || !std::isfinite(st.kappa_obs)) {
            st.kappa_obs = in.wheel.valid ? std::tan(in.wheel.delta_rad) / cfg.L
                         : (std::isfinite(k_emit_phys) ? k_emit_phys : 0.0);
            st.obs_init = true;
        } else if (std::isfinite(k_emit_phys)) {
            const double dt_obs = std::isfinite(in.dt) ? std::clamp(in.dt, 0.0, 0.5) : cfg.Ts;
            const double a = (p.actuator_tau_model > 1e-3) ? std::exp(-dt_obs / p.actuator_tau_model) : 0.0;
            st.kappa_obs = k_emit_phys + (st.kappa_obs - k_emit_phys) * a;
        }
        st.kappa_obs = std::clamp(st.kappa_obs, cfg.kappa_min, cfg.kappa_max);
        out.num["observer_kappa"] = st.kappa_obs;
        if (in.wheel.valid) out.num["observer_residual_kappa"] = std::tan(in.wheel.delta_rad) / cfg.L - st.kappa_obs;
    }
    if (f.measured_state && lag_model) {
        if (in.wheel.valid) {
            kappa_state = std::tan(in.wheel.delta_rad) / cfg.L;
            state_source = "measured";
        } else if (have_emit) {
            kappa_state = st.kappa_obs;
            state_source = "fallback-observer";
            wheel_fallback_cap = true;
            fallback = "wheel-" + in.wheel.reason;
        } else {
            kappa_state = 0.0;
            state_source = "unknown-stop";
            no_state_stop = true;
            fallback = "wheel-" + in.wheel.reason + "-no-emitted-command";
        }
        kappa_state = std::clamp(kappa_state, cfg.kappa_min, cfg.kappa_max);
    } else if (f.measured_state) {
        if (in.wheel.valid) {
            kappa_state = std::tan(in.wheel.delta_rad) / cfg.L;
            state_source = "measured";
        } else if (in.have_emitted && std::isfinite(in.emitted_steer_rad)) {
            kappa_state = std::tan(in.emitted_steer_rad) / cfg.L;
            state_source = "fallback-command-model";
            wheel_fallback_cap = true;
            fallback = "wheel-" + in.wheel.reason;
        } else {
            kappa_state = 0.0;
            state_source = "unknown-stop";
            no_state_stop = true;
            fallback = "wheel-" + in.wheel.reason + "-no-emitted-command";
        }
        kappa_state = std::clamp(kappa_state, cfg.kappa_min, cfg.kappa_max);
    }
    const double gain_for_state = f.physical_output ? p.steering_output_gain : cfg.kappa_gain;
    if (!f.measured_state && f.emitted_command_state && lag_model) {
        kappa_state = st.kappa_obs;                  // physical curvature = observer estimate
        state_source = "observer-model";
    } else if (!f.measured_state && f.emitted_command_state) {
        kappa_state = (in.have_emitted && std::isfinite(in.emitted_steer_rad))
            ? std::tan(in.emitted_steer_rad / std::max(1e-6, gain_for_state)) / cfg.L : 0.0;
        kappa_state = std::clamp(kappa_state, cfg.kappa_min, cfg.kappa_max);
        state_source = "emitted-command";
    }
    out.str["state_source"] = state_source;
    out.num["state_kappa"] = kappa_state;

    // ── Reference origin ────────────────────────────────────────────────
    double theta_ref, kappa_ref, dr;
    double s0 = in.vehicle_s;
    RefSample ref0;
    if (f.station_preview) {
        ref0 = samplePath(in.path, s0);
        s0 = ref0.s;
        theta_ref = ref0.heading;
        kappa_ref = ref0.kappa;
        const double dx = in.ego_x - ref0.x, dy = in.ego_y - ref0.y;
        dr = -std::sin(theta_ref) * dx + std::cos(theta_ref) * dy;
    } else {
        theta_ref = wh[nearest];
        kappa_ref = wk[nearest];
        const double dx = in.ego_x - wx[nearest], dy = in.ego_y - wy[nearest];
        dr = -std::sin(theta_ref) * dx + std::cos(theta_ref) * dy;
    }
    const double delta_theta = (in.cur_gear < 0)
        ? loopWrap(in.ego_yaw - theta_ref + M_PI)
        : loopWrap(in.ego_yaw - theta_ref);
    Eigen::VectorXd x0(kNx);
    x0 << dr, theta_ref + delta_theta, kappa_state, theta_ref, kappa_ref;

    // ── Speed targets / profile (C2) and stations (C3) ──────────────────
    std::vector<double> targets(N), v_profile(N), nodes;
    std::vector<double> stations;
    const double v_ref_calc = std::max(2.0 / 3.6, in.ego_v);
    const int idx_per_step_v = std::max(1, (int)std::round(v_ref_calc * cfg.Ts / in.wp_spacing));
    double stage0_max_k = 0.0, stage0_la = 0.0;
    // revision3: a fresh planner cap that is not a finite value >= 0 is malformed -> stop (NaN compares false)
    const bool cap_malformed = in.planner_cap_fresh &&
        !(std::isfinite(in.planner_cap_mps) && in.planner_cap_mps >= 0.0);
    const bool cap_stop = in.planner_cap_fresh && (cap_malformed || in.planner_cap_mps <= 0.05);
    // Safety/validity caps apply to the prediction in EVERY flag combination (revision2);
    // the legacy contract keeps the target sign (reverse) and applies v_factor at its output.
    auto applyCaps = [&](double v) {
        const double sign = (!f.reachable_velocity && v < 0.0) ? -1.0 : 1.0;
        double a = f.reachable_velocity ? v * in.v_factor : std::abs(v);
        if (in.planner_cap_fresh) a = cap_malformed ? 0.0 : std::min(a, std::max(0.0, in.planner_cap_mps));
        if (cap_stop) a = 0.0;
        if (wheel_fallback_cap) a = std::min(a, p.wheel_fallback_max_mps);
        if (no_state_stop) a = 0.0;
        if (f.reachable_velocity) return std::max(0.0, a);
        return sign * std::max(0.0, a);
    };
    auto stageTarget = [&](int i, int base_idx, double station, double d_remain, bool have_remain) {
        double la_m = curve.lookahead(std::max(v_ref_calc, cfg.target_vel));
        double max_k = curve.maxKappaAhead(base_idx, la_m);
        if (i == 0) { stage0_max_k = max_k; stage0_la = la_m; }
        double v_max_for_seg = cfg.target_vel;
        if (in.parking_mode) v_max_for_seg = curve.parkingCap(max_k);
        double v_target = curve.curveLimited(v_max_for_seg, max_k);
        if (p.curve_decel_enable && in.cur_gear > 0
                && !in.parking_mode && f.station_preview) {
            v_target = v_max_for_seg;
            for (int k = std::max(in.path.lo, base_idx); k <= seg_hi_for_la; ++k) {
                const double distance = std::max(0., ws[k]-station);
                const double curve_speed = curve.curveLimited(v_max_for_seg, std::abs(wk[k]));
                v_target = std::min(v_target, junctionCurveApproachSpeed(curve_speed,
                    distance, v_max_for_seg, std::max(.5, p.curve_brake_decel_mps2)));
                if (distance >= la_m) break;
            }
        }
        if (have_remain) v_target = std::min(v_target, CurveHelpers::approach(d_remain, v_max_for_seg));
        if (in.cur_gear < 0) {
            v_target = std::min(v_target, in.reverse_max_vel_kmh / 3.6);
            v_target = -v_target;
        }
        return v_target;
    };
    int seg_e = 0;
    const bool gear_change = curve.gearChangeAhead(&seg_e);
    if (!f.station_preview) {
        for (int i = 0; i < N; ++i) {
            int base_idx = nearest + (i + 1) * idx_per_step_v;
            double d_remain = 0.0;
            if (gear_change) {
                int idx_pred = std::min(nearest + (i + 1) * idx_per_step_v, seg_e);
                for (int kk = idx_pred; kk < seg_e; ++kk) {
                    d_remain += std::sqrt(std::pow(wx[kk + 1] - wx[kk], 2.0) + std::pow(wy[kk + 1] - wy[kk], 2.0));
                }
            }
            targets[i] = applyCaps(stageTarget(i, base_idx, s0, d_remain, gear_change));
        }
        if (f.reachable_velocity) {
            v_profile = reachableStageSpeeds(in.ego_v, targets, p.pred_accel_mps2, p.pred_decel_mps2, cfg.Ts, &nodes);
        } else {
            v_profile = targets;
        }
    } else {
        // stations integrate the same stage speeds that build the matrices
        double v = std::max(0.0, in.ego_v);
        nodes.assign(1, v);
        stations.assign(1, s0);
        const double s_hi = ws[std::min((int)ws.size() - 1, std::max(in.path.lo, in.path.hi))];
        for (int i = 0; i < N; ++i) {
            const RefSample here = samplePath(in.path, stations[i]);
            const int base_idx = std::max(here.index, nearest);
            double d_remain = 0.0;
            if (gear_change) d_remain = std::max(0.0, ws[seg_e] - stations[i]);
            targets[i] = applyCaps(stageTarget(i, base_idx, stations[i], d_remain, gear_change));
            double stage_speed;
            if (f.reachable_velocity) {
                const double target = std::max(0.0, targets[i]);
                const double dv = std::clamp(target - v, -p.pred_decel_mps2 * cfg.Ts, p.pred_accel_mps2 * cfg.Ts);
                const double next = std::max(0.0, v + dv);
                stage_speed = 0.5 * (v + next);
                v = next;
                nodes.push_back(v);
            } else {
                stage_speed = targets[i];
            }
            v_profile[i] = stage_speed;
            stations.push_back(std::min(s_hi, stations[i] + std::abs(stage_speed) * cfg.Ts));
        }
    }
    out.v_profile = v_profile;
    out.stations = stations;
    out.num["ltv_curve_lookahead_m"] = stage0_la;
    out.num["ltv_max_kappa_cmd"] = stage0_max_k;
    out.num["ltv_curve_cap_kmh"] = v_profile.empty() ? 0.0 : v_profile[0] * 3.6;
    out.num["v_target0_mps"] = targets.empty() ? 0.0 : targets[0];
    out.num["v_pred0_mps"] = nodes.empty() ? v_profile[0] : nodes.front();
    out.num["v_predN_mps"] = nodes.empty() ? v_profile.back() : nodes.back();
    if (!stations.empty()) {
        out.num["station0"] = stations.front();
        out.num["stationN"] = stations.back();
    }

    // safety outputs (flag independent)
    if (no_state_stop || cap_stop) out.safety_cap_mps = 0.0;
    else if (wheel_fallback_cap) out.safety_cap_mps = p.wheel_fallback_max_mps;
    if (no_state_stop) { out.safe_stop = true; out.hold_steer = true; out.stop_reason = "no-steering-state"; }
    if (cap_malformed) { out.safe_stop = true; out.stop_reason = "planner-cap-malformed"; }
    out.num["safety_cap_mps"] = std::isfinite(out.safety_cap_mps) ? out.safety_cap_mps : -1.0;

    const int nx = lag_model ? kNxLag : kNx;
    const int kappa_row = lag_model ? 5 : 2;      // bounded (commanded) curvature row
    const double out_gain = f.physical_output ? p.steering_output_gain : cfg.kappa_gain;
    double kappa_cmd0 = kappa_state;
    if (lag_model) {
        kappa_cmd0 = (in.have_emitted && std::isfinite(in.emitted_steer_rad))
            ? std::tan(in.emitted_steer_rad / std::max(1e-6, out_gain)) / cfg.L : kappa_state;
        kappa_cmd0 = std::clamp(kappa_cmd0, cfg.kappa_min, cfg.kappa_max);
        Eigen::VectorXd x6(kNxLag);
        x6 << x0(0), x0(1), x0(2), x0(3), x0(4), kappa_cmd0;
        x0 = x6;
    }
    out.str["state_model"] = lag_model ? "lag6" : "ltv5";
    out.num["actuator_tau_model"] = lag_model ? p.actuator_tau_model : 0.0;
    out.num["cmd_kappa_state"] = kappa_cmd0;
    Eigen::MatrixXd A_bar, B_bar, E_bar;
    if (lag_model) lagBatch(cfg, p.actuator_tau_model, v_profile, A_bar, B_bar, E_bar);
    else model.buildBatchMatrices(v_profile, A_bar, B_bar, E_bar);

    // ── z_bar ──────────────────────────────────────────────────────────
    Eigen::VectorXd z_bar = Eigen::VectorXd::Zero(N);
    if (f.station_preview) {
        double k_prev = kappa_ref;
        for (int i = 0; i < N; ++i) {
            const double k_next = samplePath(in.path, stations[i + 1]).kappa;
            z_bar(i) = (k_next - k_prev) / cfg.Ts;
            k_prev = k_next;
        }
    } else {
        double v_ref = std::max(2.0 / 3.6, in.ego_v);
        int idx_per_step = std::max(1, (int)std::round(v_ref * cfg.Ts / in.wp_spacing));
        for (int i = 0; i < N; ++i) {
            int ki  = std::min(nearest + (i + 1) * idx_per_step, (int)wk.size() - 1);
            int ki0 = std::min(nearest +  i      * idx_per_step, (int)wk.size() - 1);
            z_bar(i) = (wk[ki] - wk[ki0]) / cfg.Ts;
        }
    }

    Eigen::VectorXd x_free = A_bar * x0 + E_bar * z_bar;
    Eigen::SparseMatrix<double> P; Eigen::VectorXd q_vec;
    if (lag_model) lagCost(cfg, x_free, B_bar, v_profile, in.max_kappa_ahead, P, q_vec);
    else cost.buildQPObjective(x0, z_bar, A_bar, B_bar, E_bar, v_profile, in.max_kappa_ahead, P, q_vec);
    const int tail_steps = (p.tail_horizon_s > 0.0) ? static_cast<int>(std::lround(p.tail_horizon_s / cfg.Ts)) : 0;
    out.num["tail_steps"] = tail_steps;
    if (tail_steps > 0) {
        // same adaptive stage weights as the horizon cost (LTVCost / lagCost)
        double v_avg = 0.0;
        for (int i = 0; i < N; ++i) v_avg += v_profile[i];
        v_avg /= N;
        const double t_frac = std::clamp((v_avg - cfg.w_theta_v_low) / (cfg.w_theta_v_high - cfg.w_theta_v_low + 1e-6), 0.0, 1.0);
        const double w_theta = cfg.w_theta_low_speed + t_frac * (cfg.w_theta_high_speed - cfg.w_theta_low_speed);
        const double w_dr = cfg.w_dr * ((in.max_kappa_ahead > cfg.curve_kappa_thresh) ? cfg.w_dr_curve_boost : 1.0);
        const double vN = v_profile[N - 1];
        const double rN = cfg.w_u + cfg.w_u_v_gain * vN;
        const Eigen::MatrixXd W = tailWeight(vN, cfg.Ts, lag_model, p.actuator_tau_model, w_dr, w_theta,
                                             cfg.w_kappa, rN, tail_steps);
        const Eigen::MatrixXd BN = B_bar.block((N - 1) * nx, 0, nx, N);
        const Eigen::VectorXd xN = x_free.segment((N - 1) * nx, nx);
        Eigen::MatrixXd Pd = Eigen::MatrixXd(P);
        Pd += (2.0 * BN.transpose() * W * BN).triangularView<Eigen::Upper>().toDenseMatrix();
        P = Pd.sparseView();
        q_vec += 2.0 * BN.transpose() * W * xN;
        out.num["tail_vN_mps"] = vN;
    }

    const double cur_steer_rad = std::atan((lag_model ? kappa_cmd0 : kappa_state) * cfg.L);
    const double cos_s = std::cos(cur_steer_rad);
    double k_dot_limit = (p.max_steer_rate_deg_s * M_PI / 180.0) / (cfg.L * cos_s * cos_s + 1e-6);
    double u_lim = k_dot_limit;

    std::vector<double> d_min_k, d_max_k;
    double v_scale_obs = 1.0;
    const bool obs_active = static_cast<bool>(in.corridor);
    if (obs_active) in.corridor(v_profile, d_min_k, d_max_k, v_scale_obs);
    out.obs_active = obs_active;
    out.v_scale_obs = v_scale_obs;

    const int n_dec = 2 * N;
    const int n_row = 5 * N;
    const double INF = 1.0e10;
    Eigen::MatrixXd A_cons_dense = Eigen::MatrixXd::Zero(n_row, n_dec);
    A_cons_dense.block(0, 0, N, N).setIdentity();
    for (int k = 0; k < N; ++k) {
        A_cons_dense.block(N + k, 0, 1, N) = B_bar.block(k * nx + kappa_row, 0, 1, N);
    }
    for (int k = 0; k < N; ++k) {
        A_cons_dense.block(2*N + k, 0,  1, N) = B_bar.block(k * nx + 0, 0, 1, N);
        A_cons_dense.block(2*N + k, N + k, 1, 1) << -1.0;
        A_cons_dense.block(3*N + k, 0,  1, N) = B_bar.block(k * nx + 0, 0, 1, N);
        A_cons_dense.block(3*N + k, N + k, 1, 1) << +1.0;
    }
    A_cons_dense.block(4*N, N, N, N).setIdentity();
    Eigen::SparseMatrix<double> A_cons = A_cons_dense.sparseView();

    Eigen::VectorXd l_cons(n_row), u_cons(n_row);
    l_cons.segment(0, N) = Eigen::VectorXd::Constant(N, -u_lim);
    u_cons.segment(0, N) = Eigen::VectorXd::Constant(N,  u_lim);
    for (int k = 0; k < N; ++k) {
        double k_free = x_free(k * nx + kappa_row);
        l_cons(N + k) = cfg.kappa_min - k_free;
        u_cons(N + k) = cfg.kappa_max - k_free;
    }
    for (int k = 0; k < N; ++k) {
        double dr_free = x_free(k * nx + 0);
        double d_max_v = obs_active ? d_max_k[k] :  INF;
        double d_min_v = obs_active ? d_min_k[k] : -INF;
        l_cons(2*N + k) = -INF;
        u_cons(2*N + k) = d_max_v - dr_free;
        l_cons(3*N + k) = d_min_v - dr_free;
        u_cons(3*N + k) = INF;
    }
    l_cons.segment(4*N, N) = Eigen::VectorXd::Zero(N);
    u_cons.segment(4*N, N) = Eigen::VectorXd::Constant(N, INF);

    Eigen::SparseMatrix<double> P_ext(n_dec, n_dec);
    std::vector<Eigen::Triplet<double>> trips;
    trips.reserve(P.nonZeros() + N);
    for (int kcol = 0; kcol < P.outerSize(); ++kcol) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(P, kcol); it; ++it) {
            trips.emplace_back(it.row(), it.col(), it.value());
        }
    }
    double w_sq = obs_active ? cfg.w_slack_quad : 0.0;
    double w_sl = obs_active ? cfg.w_slack_lin  : 0.0;
    for (int k = 0; k < N; ++k) trips.emplace_back(N + k, N + k, 2.0 * w_sq);
    P_ext.setFromTriplets(trips.begin(), trips.end());
    P_ext.makeCompressed();
    Eigen::VectorXd q_ext(n_dec);
    q_ext.head(N) = q_vec;
    q_ext.tail(N) = Eigen::VectorXd::Constant(N, w_sl);

    Eigen::VectorXd sol;
    const bool solved = solver.solve(P_ext, q_ext, A_cons, l_cons, u_cons, sol) && sol.size() >= N;
    out.solved = solved;
    // legacy: obstacle v_scale (known only after the corridor) replaces the cooldown factor
    const double v_factor_cmd = obs_active ? v_scale_obs : in.v_factor;

    if (!solved) {
        // revision2: every mode publishes an explicit safe stop (zero speed target at the
        // command boundary, steering held at the last emitted value). Persistent controller
        // state (command integral / observer) is left untouched; the solver has invalidated
        // its workspace and re-runs a fresh setup on the next valid problem.
        fallback = "solver-failed";
        out.publish = true;
        out.safe_stop = true;
        out.hold_steer = true;
        out.stop_reason = "solver-failed:" + solver.lastError();
        out.steer_cmd_rad = have_emit ? in.emitted_steer_rad : 0.0;
        out.vel_cmd_kmh = 0.0;
        out.publish_filter_bypass = true;
        out.str["fallback_reason"] = fallback;
        out.str["solver_error"] = solver.lastError();
        return out;
    }

    // ── predicted CTE from the batch model with the optimised U ────────
    const Eigen::VectorXd U = sol.head(N);
    const Eigen::VectorXd x_pred = x_free + B_bar * U;
    out.num["predicted_cte"] = x_pred(0);
    out.num["predicted_cte_legacy_formula"] = dr + sol[0] * cfg.Ts;

    // ── steering output (C1) ───────────────────────────────────────────
    double final_steer_rad;
    bool rate_limited = false, angle_limited = false;
    if (!f.measured_state && !f.emitted_command_state) {
        // legacy command integral
        st.current_kappa += sol[0] * cfg.Ts;
        st.current_kappa = std::clamp(st.current_kappa, cfg.kappa_min, cfg.kappa_max);
        double steer_rad = std::atan(st.current_kappa * cfg.L);
        double gain = f.physical_output ? p.steering_output_gain : cfg.kappa_gain;
        double raw_steer = steer_rad * gain;
        double raw_deg = raw_steer * 180.0 / M_PI;
        double lim;
        if (!st.cmd_init) {
            // revision2: the post-gain angle clamp also applies to the first emission
            lim = std::clamp(raw_deg, -p.max_steer_deg, p.max_steer_deg);
            angle_limited = lim != raw_deg;
            st.prev_steer_deg = lim; st.cmd_init = true;
        } else {
            double max_d = p.max_steer_rate_deg_s * in.dt;
            lim = std::clamp(raw_deg, st.prev_steer_deg - max_d, st.prev_steer_deg + max_d);
            rate_limited = lim != raw_deg;
            double c = std::clamp(lim, -p.max_steer_deg, p.max_steer_deg);
            angle_limited = c != lim;
            lim = c;
        }
        final_steer_rad = lim * M_PI / 180.0;
        st.prev_steer_deg = lim;
        out.num["cmd_kappa_desired"] = st.current_kappa;
        out.num["steer_raw_rad"] = raw_steer;
    } else {
        // desired next commanded curvature: from the physical state (5-state) or from the
        // commanded-curvature state (lag model); predicted physical response is x_pred(2)
        const double kappa_base = lag_model ? kappa_cmd0 : kappa_state;
        const double kappa_des = std::clamp(kappa_base + sol[0] * cfg.Ts, cfg.kappa_min, cfg.kappa_max);
        out.num["pred_phys_kappa1"] = x_pred(2);
        const double delta_des = std::atan(kappa_des * cfg.L);
        const double raw_steer = delta_des * out_gain;
        const double raw_deg = raw_steer * 180.0 / M_PI;
        double lim = raw_deg;
        if (in.have_emitted && std::isfinite(in.emitted_steer_rad)) {
            const double prev_deg = in.emitted_steer_rad * 180.0 / M_PI;
            const double max_d = p.max_steer_rate_deg_s * in.dt;
            lim = std::clamp(raw_deg, prev_deg - max_d, prev_deg + max_d);
            rate_limited = lim != raw_deg;
        }
        const double c = std::clamp(lim, -p.max_steer_deg, p.max_steer_deg);
        angle_limited = c != lim;
        lim = c;
        final_steer_rad = lim * M_PI / 180.0;
        // command state is informational only; never fed back as physical state
        st.current_kappa = kappa_des;
        st.prev_steer_deg = lim;
        st.cmd_init = true;
        out.num["cmd_kappa_desired"] = kappa_des;
        out.num["steer_raw_rad"] = raw_steer;
    }
    out.num["steer_limited_rad"] = final_steer_rad;
    out.num["rate_limited"] = rate_limited ? 1.0 : 0.0;
    out.num["angle_limited"] = angle_limited ? 1.0 : 0.0;

    // ── speed output (C2) ─────────────────────────────────────────────
    double vel_cmd_mps;
    if (!f.reachable_velocity) {
        double v_cmd_mps = std::abs(v_profile[0]) * v_factor_cmd;
        if (!st.v_sig_init) { st.v_sig = v_cmd_mps; st.v_sig_init = true; vel_cmd_mps = st.v_sig; }
        else {
            // Resume from the measured speed after a stop pulse (see command_boundary).
            if (v_cmd_mps > st.v_sig && std::abs(in.ego_v) > st.v_sig) st.v_sig = std::min(v_cmd_mps, std::abs(in.ego_v));
            double tau = (v_cmd_mps > st.v_sig) ? p.sig_tau_up : p.sig_tau_down;
            double alpha = 1.0 - std::exp(-in.dt / std::max(tau, 1e-3));
            st.v_sig += alpha * (v_cmd_mps - st.v_sig);
            vel_cmd_mps = std::max(0.0, st.v_sig);
        }
        out.num["target_vel_kmh"] = v_profile[0] * 3.6 * v_factor_cmd;
    } else {
        const double target0 = std::max(0.0, targets[0]) * (obs_active ? v_scale_obs : 1.0);
        // After a stop pulse the last emitted speed is 0 while the car still
        // moves: ramping up from 0 commanded 3-16 km/h at 60 km/h and braked
        // the car to 5 km/h (highway, 2026-10-04). Resume from measured speed.
        double prev = in.have_emitted ? in.emitted_speed_mps : in.ego_v;
        if (target0 > prev && std::abs(in.ego_v) > prev) prev = std::min(target0, std::abs(in.ego_v));
        const double dv = std::clamp(target0 - prev, -p.pred_decel_mps2 * in.dt, p.pred_accel_mps2 * in.dt);
        vel_cmd_mps = std::max(0.0, prev + dv);
        out.publish_filter_bypass = true;
        out.num["target_vel_kmh"] = target0 * 3.6;
    }
    out.publish = true;
    out.steer_cmd_rad = final_steer_rad;
    out.vel_cmd_kmh = vel_cmd_mps * 3.6;

    if (obs_active) {
        out.d_min0 = d_min_k.empty() ? -1e6 : d_min_k[0];
        out.d_max0 = d_max_k.empty() ? 1e6 : d_max_k[0];
        double slack_sum = 0.0;
        for (int k = 0; k < N; ++k) slack_sum += std::abs(sol[N + k]);
        out.slack_sum = slack_sum;
    }
    if (!std::isfinite(out.steer_cmd_rad) || !std::isfinite(out.vel_cmd_kmh)) {
        fallback = "nonfinite-command";
        out.steer_cmd_rad = have_emit ? in.emitted_steer_rad : 0.0;
        out.vel_cmd_kmh = 0.0;
        out.safe_stop = true;
        out.hold_steer = true;
        out.stop_reason = "nonfinite-command";
    }
    if (out.safe_stop) out.vel_cmd_kmh = 0.0;
    out.str["fallback_reason"] = fallback;
    return out;
}

}  // namespace moraimpc
