#pragma once
// Forward-D LTV controller consistency core (no ROS dependency).
//
// C1 physical steering state (measured wheel angle, explicit units)
// C2 one effective/reachable velocity contract
// C3 time-distance consistent curvature preview (arc-length stations)
// C4 diagnostics computed from the actually emitted command
//
// Every correction has an ablation flag; with all flags at their legacy value
// ltvStep() reproduces the previous path_follower.cpp LTV block exactly.
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "moraimpc/ltv_types.hpp"
#include "moraimpc/ltv_model.hpp"
#include "moraimpc/ltv_cost.hpp"
#include "moraimpc/ltv_solver.hpp"

namespace moraimpc {

// ── C1 measurement contract ────────────────────────────────────────────────
enum class WheelUnit { UNSET, DEG, RAD };
WheelUnit parseWheelUnit(const std::string& text);
const char* wheelUnitName(WheelUnit unit);

struct WheelParams {
    WheelUnit unit = WheelUnit::UNSET;   // must be configured explicitly
    double timeout_s = 0.5;              // receipt age limit
    double physical_max_deg = 40.0;      // vehicle physical road-wheel limit
    double tolerance_deg = 0.5;          // numeric slack on the physical limit
    double residual_rad = 0.35;          // |measured - emitted| bound while moving
    double residual_time_s = 1.0;        // continuous time over the bound -> 'residual'
    double moving_mps = 1.0;             // residual/frozen policy active above this speed
    double frozen_time_s = 1.0;          // bit-identical value for longer than this ...
    double frozen_cmd_span_rad = 0.02;   // ... while the emitted command spanned more than this
};

struct WheelTracker {                    // persistent residual/frozen state (no eternal latch)
    double residual_time_s = 0.0;
    bool have_last = false;
    double last_raw = 0.0;
    double unchanged_time_s = 0.0;
    double emitted_min = 0.0, emitted_max = 0.0;
};

struct WheelMeasurement {
    bool valid = false;
    double raw = 0.0;
    double delta_rad = 0.0;
    double age_s = 0.0;
    std::string reason = "missing";
};

// Validate one wheel sample. 'emitted_prev_rad' is the previously emitted
// CtrlCmd road-wheel command (MORAI semantics: road-wheel radians).
WheelMeasurement evaluateWheel(bool received, double raw, double age_s,
                               const WheelParams& params,
                               bool have_emitted, double emitted_prev_rad,
                               double measured_speed_mps, double dt,
                               WheelTracker& tracker);

// ── C3 arc-length preview ──────────────────────────────────────────────────
struct PathArrays {
    const std::vector<double>* x = nullptr;
    const std::vector<double>* y = nullptr;
    const std::vector<double>* h = nullptr;
    const std::vector<double>* k = nullptr;
    const std::vector<double>* s = nullptr;   // cumulative arc length
    int lo = 0;                                // current gear segment [lo, hi]
    int hi = -1;
};

struct RefSample {
    double s = 0.0, x = 0.0, y = 0.0, heading = 0.0, kappa = 0.0;
    int index = 0;            // segment start index of the interpolation interval
    bool clamped = false;     // station outside [s_lo, s_hi]
};

double wrapPi(double a);
RefSample samplePath(const PathArrays& path, double station);

// ── C2 velocity ───────────────────────────────────────────────────────────
// v_{i+1} = v_i + clamp(target_i - v_i, -decel*Ts, +accel*Ts); returns stage
// speeds 0.5*(v_i + v_{i+1}); fills node speeds (size N+1) if requested.
std::vector<double> reachableStageSpeeds(double v0, const std::vector<double>& targets,
                                         double accel, double decel, double Ts,
                                         std::vector<double>* nodes = nullptr);

// ── Step configuration / state ───────────────────────────────────────────
struct LtvFlags {
    bool measured_state = true;      // C1: false -> legacy command-integral kappa
    bool physical_output = true;     // C1 mapping: false -> legacy atan(kappa L)*kappa_gain
    bool station_preview = true;     // C3: false -> legacy index preview
    bool reachable_velocity = true;  // C2: false -> legacy v_profile + sigmoid + publish limiter
    // C1 command-state variant (used when measured_state is false): true -> the model
    // state is the curvature of the command actually EMITTED (after rate/angle clamps,
    // output gain removed); false -> legacy unreconciled command integral.
    bool emitted_command_state = false;
};

struct LtvParams {
    LTVMPCConfig cfg;
    LtvFlags flags;
    double steering_output_gain = 1.0;        // physical mode: CtrlCmd.steering = delta * gain
    double max_steer_rate_deg_s = 50.0;
    double max_steer_deg = 35.0;
    double pred_accel_mps2 = 1.0;
    double pred_decel_mps2 = 5.0;
    double wheel_fallback_max_mps = 10.0 / 3.6;
    // Explicit actuator model: physical curvature follows the commanded curvature with a
    // first-order time constant tau (s). Active (6-state model) when tau > 0 and the state
    // source is 'measured' or 'emitted' (revision2: with 'emitted' the physical curvature is
    // the output of an actuator observer driven by the emitted commands). 0 = 5-state model,
    // physical curvature assumed to follow the command instantly.
    // A robustness/test assumption, NOT a measured or calibrated MORAI lag.
    double actuator_tau_model = 0.0;
    // Revision2 tail cost-to-go: exact cost of tail_horizon_s/Ts additional UNCONSTRAINED
    // steps of the error model at the terminal stage speed (reference curvature held after
    // the horizon), added as a terminal weight on x_N. 0 = legacy (no terminal cost).
    double tail_horizon_s = 0.0;
    // curve speed profile (unchanged semantics)
    bool curve_decel_enable = true;
    bool junction_curve_profile = false;  // fresh committed ring only
    double curve_speed_alpha = 12.0;
    double curve_alat_max = 2.26;
    double curve_lookahead_min_m = 25.0;
    double curve_lookahead_max_m = 70.0;
    double curve_lookahead_time_s = 2.5;
    double curve_brake_decel_mps2 = 2.0;
    // legacy velocity filter
    double sig_tau_up = 0.30;
    double sig_tau_down = 0.15;
};

struct LtvPersistent {                 // controller state carried between ticks
    double current_kappa = 0.0;        // legacy command integral (also kappa_des info)
    double prev_steer_deg = 0.0;       // legacy rate-limiter state
    bool cmd_init = false;
    bool v_sig_init = false;
    double v_sig = 0.0;
    // revision2 actuator observer: physical curvature estimate propagated with the emitted
    // command and actuator_tau_model (model estimate, not a measurement)
    bool obs_init = false;
    double kappa_obs = 0.0;
};

struct LtvInput {
    PathArrays path;
    const std::vector<int>* gear = nullptr;
    std::vector<std::pair<int, int>> segments;
    int cur_segment = 0;
    double wp_spacing = 0.5;
    int nearest_idx = 0;
    double vehicle_s = 0.0;           // continuous projection station
    int cur_gear = 1;
    bool parking_mode = false;
    double parking_max_kmh = 2.0;
    double reverse_max_vel_kmh = 5.0;
    double ego_x = 0.0, ego_y = 0.0, ego_yaw = 0.0, ego_v = 0.0;
    double dt = 0.05;
    double max_kappa_ahead = 0.0;     // cost weight selector (computed by caller as before)
    WheelMeasurement wheel;
    bool have_emitted = false;
    double emitted_steer_rad = 0.0;   // previously emitted command
    double emitted_speed_mps = 0.0;
    bool planner_cap_fresh = false;
    double planner_cap_mps = 0.0;
    double v_factor = 1.0;            // obstacle/cooldown factor (avoidance_enabled only)
    std::function<void(const std::vector<double>&, std::vector<double>&,
                       std::vector<double>&, double&)> corridor;   // null = inactive
};

struct LtvOutput {
    bool solved = false;
    bool publish = false;             // revision2: always true once a step ran (safe stop on failure)
    // revision2 safety contract (independent of ablation flags): the command boundary must
    // emit a zero speed target when safe_stop is set and never exceed safety_cap_mps.
    bool safe_stop = false;
    bool hold_steer = false;          // stop without a valid new steering solution: hold last emitted
    std::string stop_reason;
    double safety_cap_mps = std::numeric_limits<double>::infinity();
    double steer_cmd_rad = 0.0;       // to CtrlCmd.steering (before publish-side veto)
    double vel_cmd_kmh = 0.0;
    bool publish_filter_bypass = false;  // reachable mode: publish must not re-filter speed
    std::vector<double> v_profile;    // stage speeds used by matrices/stations
    std::vector<double> stations;     // N+1 stations (station mode) else empty
    std::map<std::string, double> num;
    std::map<std::string, std::string> str;
    double d_min0 = -1e6, d_max0 = 1e6, v_scale_obs = 1.0, slack_sum = 0.0;
    bool obs_active = false;
};

LtvOutput ltvStep(const LtvParams& params, const LtvInput& in, LtvPersistent& st,
                  LTVModel& model, LTVCost& cost, LTVSolver& solver);

// Tail cost-to-go weight in FULL model coordinates (nx x nx; nx = 5, or 6 with lag):
// P_M of M backward Riccati steps of the error model e = T x at speed v with stage weights
// diag(w_dr, w_theta, w_kappa[, 0]) and input weight r; returns T' P_M T.
Eigen::MatrixXd tailWeight(double v, double Ts, bool lag, double tau, double w_dr, double w_theta,
                           double w_kappa, double r, int M);

// Validate a flag/parameter combination. Returns "" when allowed, else a diagnosis.
std::string validateLtvConfig(const LtvParams& params, WheelUnit unit);

}  // namespace moraimpc
