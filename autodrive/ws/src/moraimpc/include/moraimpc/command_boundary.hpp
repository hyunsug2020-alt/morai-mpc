#pragma once
// Revision2 command boundary (no ROS dependency).
//
// Every CtrlCmd the node emits passes through CommandBoundary::emit(). It owns the
// publish-side speed filter state and the last emitted command, and enforces, independently
// of controller mode, ablation flags and filter_bypass:
//   * stop dominance: a fresh planner stop (cap <= stop_cap_mps), a safety stop request, a
//     non-finite request, a stale/unknown ego or a zero safety cap emit a ZERO speed target in
//     the same call (the vehicle decelerates physically; no smoothing toward zero);
//   * steering during a stop: the requested (tracking) steering or, for hold requests, the
//     last emitted steering while moving; centered only at rest with a fresh ego state;
//   * post-gain angle clamp +-max_steer_deg and rate clamp max_rate*dt on EVERY emission,
//     including the first one (reference: previous emission, else valid measured wheel, else 0);
//   * anti-windup: the publish limiter state never exceeds what was actually emitted, so a
//     restart after a stop ramps from the emitted zero (no stale-integrator pulse).
#include <cmath>
#include <limits>
#include <string>

namespace moraimpc {

struct BoundaryParams {
    double max_steer_deg = 35.0;
    double max_steer_rate_deg_s = 50.0;
    double center_speed_mps = 0.5;      // centering allowed only at or below this measured speed
    double rise_mps2 = 1.0;             // legacy publish limiter (planner-path mode)
    double fall_mps2 = 5.0;
    double Ts = 0.05;                   // legacy limiter step (the node used cfg_.Ts)
    double stop_cap_mps = 0.05;         // fresh planner cap at or below this = stop request
    bool cap_as_ceiling = true;         // avoid_vel_as_cap
    double max_rate_dt_s = 0.1;         // rate-clamp interval ceiling (stalled loop)
};

struct BoundaryState {
    bool have_emitted = false;
    double steer_rad = 0.0;             // last emitted steering (road-wheel rad, before sign)
    double speed_kmh = 0.0;             // last emitted speed target
    double prev_avoid_cmd_kmh = 0.0;    // legacy publish limiter state
};

struct BoundaryRequest {
    double vel_kmh = 0.0;
    double steer_rad = 0.0;
    bool filter_bypass = false;         // speed already filtered once (reachable contract)
    bool stop = false;                  // explicit stop (safety / gear switch / path end)
    bool hold_steer = false;            // no valid new steering: keep the last emitted value
    std::string reason;
};

struct BoundaryContext {
    bool use_planner_path = false;      // node use_avoid_path_
    bool planner_cap_fresh = false;     // planner speed received and fresh
    double planner_cap_mps = 0.0;
    bool ego_fresh = true;
    double ego_speed_mps = 0.0;         // measured speed magnitude
    bool wheel_valid = false;           // validated measured road-wheel angle (first-emission reference)
    double wheel_rad = 0.0;
    double dt_s = 0.05;                 // time since the previous emission
    double safety_cap_mps = std::numeric_limits<double>::infinity();
};

struct BoundaryResult {
    double vel_kmh = 0.0;
    double steer_rad = 0.0;
    bool stopped = false;               // zero speed target due to a stop condition
    bool centered = false;
    bool rate_limited = false;
    bool angle_limited = false;
    bool speed_modified = false;
    bool capped = false;                // planner-cap ceiling or safety cap lowered the target
    bool steer_modified = false;        // emitted steering differs from the request
    std::string veto;                   // '|' separated reasons
};

class CommandBoundary {
public:
    explicit CommandBoundary(const BoundaryParams& p = BoundaryParams()) : p_(p) {}
    void setParams(const BoundaryParams& p) { p_ = p; }
    const BoundaryParams& params() const { return p_; }
    BoundaryResult emit(const BoundaryRequest& req, const BoundaryContext& ctx);
    const BoundaryState& state() const { return s_; }
    BoundaryState& mutableState() { return s_; }
    // true when a fresh planner cap requests a stop
    bool plannerStop(const BoundaryContext& ctx) const {
        // NaN compares false: handled as malformed in emit()
        return ctx.use_planner_path && ctx.planner_cap_fresh && ctx.planner_cap_mps <= p_.stop_cap_mps;
    }

private:
    BoundaryParams p_;
    BoundaryState s_;
};

// ── Revision3 planner feed validity contract ───────────────────────────────────
// One validity contract from message receipt to emission. Every /avoid_target_vel message is classified at
// receipt (valid: finite, 0 <= v <= cap_max_mps; otherwise malformed). snapshot(now) is taken ONCE per control
// tick and used for the watchdog, the controller prediction and every emission of that tick.
struct PlannerFeedParams {
    double timeout_s = 0.7;             // SAME limit as the feed watchdog (avoid_path_timeout_s)
    double stop_cap_mps = 0.05;
    double cap_max_mps = 60.0;          // plausibility bound for a finite cap
    double future_tolerance_s = 1.0e-3; // receipt stamps later than now by more than this are invalid
};

struct PlannerFeedSnapshot {
    bool stale = true;                  // missing / expired / invalid stamp -> watchdog safety stop
    bool stop = true;                   // any stop constraint (stale, malformed, planner stop, retained stop)
    bool cap_usable = false;            // cap_mps is a valid constraint for this tick (a valid 0 counts)
    double cap_mps = 0.0;               // valid cap, or 0 whenever stop
    double cap_age_s = std::numeric_limits<double>::infinity();
    double path_age_s = std::numeric_limits<double>::infinity();
    bool malformed = false;
    bool retained_stop = false;
    std::string reason = "no-feed";
};

class PlannerFeed {
public:
    explicit PlannerFeed(const PlannerFeedParams& p = PlannerFeedParams()) : p_(p) {}
    void setParams(const PlannerFeedParams& p) { p_ = p; }
    const PlannerFeedParams& params() const { return p_; }
    // revision4: forget every receipt (clock epoch change); snapshot() is 'no-path-yet' until new messages
    void reset() { *this = PlannerFeed(p_); }
    void onPath(double receipt_s);
    // Embedded path/cap pairs supersede the unversioned legacy cap topic.
    bool onPlan(double receipt_s, double value_mps);
    bool usesBoundPlan() const { return bound_plan_; }
    // Standalone caps are ignored while a bound plan owns the feed; otherwise
    // returns true when valid (false: malformed, recorded as a stop).
    bool onCap(double receipt_s, double value_mps);
    PlannerFeedSnapshot snapshot(double now_s) const;
    bool haveCap() const { return have_cap_; }
    double lastRaw() const { return cap_raw_; }

private:
    PlannerFeedParams p_;
    bool have_path_ = false, have_cap_ = false;
    bool bound_plan_ = false;
    double path_t_ = 0.0, cap_t_ = 0.0, cap_raw_ = 0.0, cap_valid_value_ = 0.0;
    bool cap_malformed_ = false;
    bool stop_latched_ = false;
};

// ── Revision4 receipt-age contract and clock epochs ────────────────────────────────
// Every receipt age used for a positive decision (ego, wheel, planner feed) is valid only if the clock reading is
// a started clock (now > 0: a zero ROS time means an unstarted simulated clock), the age is finite and lies in
// [-tolerance, timeout]. A receipt later than now beyond the tolerance (backward clock step) is invalid, never fresh.
struct AgeCheck {
    bool fresh = false;
    double age_s = std::numeric_limits<double>::infinity();
    const char* reason = "missing";
};

inline AgeCheck checkAge(bool received, double now_s, double receipt_s, double timeout_s, double tolerance_s) {
    AgeCheck c;
    if (!received) return c;
    if (!(now_s > 0.0) || !std::isfinite(now_s)) { c.reason = "clock-not-started"; return c; }
    c.age_s = now_s - receipt_s;
    if (!std::isfinite(c.age_s)) { c.reason = "nonfinite-age"; return c; }
    if (c.age_s < -std::abs(tolerance_s)) { c.reason = "future-receipt"; return c; }
    if (!(c.age_s <= timeout_s)) { c.reason = "stale"; return c; }
    c.fresh = true; c.reason = "fresh";
    return c;
}

// Detects backward steps of the ROS clock (sim-time reset, wall-clock step). observe() returns true exactly when
// `now` is earlier than the latest observed reading by more than the tolerance: the caller starts a new input epoch.
class ClockEpoch {
public:
    explicit ClockEpoch(double tolerance_s = 1.0e-3) : tol_(std::abs(tolerance_s)) {}
    void setTolerance(double t) { tol_ = std::abs(t); }
    bool observe(double now_s) {
        if (!std::isfinite(now_s)) return false;
        if (have_ && now_s < last_ - tol_) { last_ = now_s; ++epoch_; return true; }
        if (!have_ || now_s > last_) last_ = now_s;
        have_ = true;
        return false;
    }
    long long epoch() const { return epoch_; }
    double last() const { return last_; }

private:
    double tol_;
    bool have_ = false;
    double last_ = 0.0;
    long long epoch_ = 0;
};

}  // namespace moraimpc
