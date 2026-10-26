#include "moraimpc/command_boundary.hpp"

#include <algorithm>
#include <cmath>

namespace moraimpc {

namespace {
void addVeto(std::string& v, const char* r) {
    if (!v.empty()) v += "|";
    v += r;
}
}  // namespace

BoundaryResult CommandBoundary::emit(const BoundaryRequest& req, const BoundaryContext& ctx) {
    BoundaryResult r;
    const bool finite_req = std::isfinite(req.vel_kmh) && std::isfinite(req.steer_rad);
    const bool planner_active = ctx.use_planner_path && ctx.planner_cap_fresh;
    // An externally required planner cap must be finite and >= 0; NaN/+-Inf/negative is malformed and is a
    // stop (never 'no constraint'). The internal safety_cap sentinel (+inf = none) is separate.
    const bool planner_malformed = planner_active &&
        !(std::isfinite(ctx.planner_cap_mps) && ctx.planner_cap_mps >= 0.0);
    const bool planner_stop = plannerStop(ctx);
    // a REQUIRED planner cap (planner path configured) that is missing/expired is a stop, never 'no cap'
    const bool planner_missing = ctx.use_planner_path && !ctx.planner_cap_fresh;
    const bool cap_zero = !(ctx.safety_cap_mps > p_.stop_cap_mps);   // NaN -> stop as well
    const bool stop = req.stop || planner_stop || planner_malformed || planner_missing || cap_zero ||
                      !finite_req || !ctx.ego_fresh;
    if (req.stop) addVeto(r.veto, req.reason.empty() ? "stop-request" : req.reason.c_str());
    if (planner_malformed) addVeto(r.veto, "planner-cap-malformed");
    else if (planner_stop) addVeto(r.veto, "planner-stop");
    if (planner_missing) addVeto(r.veto, "planner-cap-missing");
    if (cap_zero) addVeto(r.veto, "safety-cap-zero");
    if (!finite_req) addVeto(r.veto, "nonfinite-request");
    if (!ctx.ego_fresh) addVeto(r.veto, "ego-stale");

    // ── speed ─────────────────────────────────────────────────────────────
    double vel = std::isfinite(req.vel_kmh) ? std::abs(req.vel_kmh) : 0.0;
    if (stop) {
        vel = 0.0;
        s_.prev_avoid_cmd_kmh = 0.0;
        r.stopped = true;
    } else {
        const bool cap_valid = planner_active;              // malformed already handled as stop
        const double av = cap_valid ? ctx.planner_cap_mps * 3.6 : 0.0;
        if (req.filter_bypass) {
            // reachable contract: filtered once in the step
        } else if (cap_valid) {
            // legacy publish limiter; cap_as_ceiling=false uses the planner speed as the limiter TARGET
            const double tgt = p_.cap_as_ceiling ? std::min(vel, av) : av;
            // A one-frame stop zeroed the limiter; rising again from 0 at the
            // rise rate commanded 1-14 km/h to a car at 82 km/h and braked it
            // to ~40 (highway, 2026-10-03). Resume from the measured speed.
            const double measured_kmh = std::isfinite(ctx.ego_speed_mps) ? std::abs(ctx.ego_speed_mps) * 3.6 : 0.0;
            if (ctx.ego_fresh && s_.prev_avoid_cmd_kmh < measured_kmh)
                s_.prev_avoid_cmd_kmh = std::min(measured_kmh, std::max(tgt, 0.0));
            double dv = tgt - s_.prev_avoid_cmd_kmh;
            const double up = p_.rise_mps2 * 3.6 * p_.Ts;
            const double dn = p_.fall_mps2 * 3.6 * p_.Ts;
            dv = std::max(-dn, std::min(up, dv));
            vel = s_.prev_avoid_cmd_kmh + dv;
        } else if (!ctx.use_planner_path) {
            // no planner path configured: legacy re-entry sync to the measured speed
            s_.prev_avoid_cmd_kmh = std::abs(ctx.ego_speed_mps) * 3.6;
        }
        // revision3: the valid fresh planner cap is a hard CEILING of the emitted target in every mode and for
        // either cap_as_ceiling value (commanded-target reduction is instantaneous; physical braking is not)
        if (cap_valid && vel > av) { vel = av; r.capped = true; addVeto(r.veto, "planner-cap-ceiling"); }
        if (std::isfinite(ctx.safety_cap_mps) && vel > ctx.safety_cap_mps * 3.6) {
            vel = ctx.safety_cap_mps * 3.6;
            r.capped = true;
            addVeto(r.veto, "safety-cap");
        }
        if (!std::isfinite(vel) || vel < 0.0) vel = 0.0;
        // anti-windup: the limiter continues from the command actually emitted
        if (ctx.use_planner_path) s_.prev_avoid_cmd_kmh = vel;
    }
    r.speed_modified = vel != (std::isfinite(req.vel_kmh) ? std::abs(req.vel_kmh) : 0.0);

    // ── steering ──────────────────────────────────────────────────────────
    const double max_rad = p_.max_steer_deg * M_PI / 180.0;
    double ref;
    if (s_.have_emitted && std::isfinite(s_.steer_rad)) ref = s_.steer_rad;
    else if (ctx.wheel_valid && std::isfinite(ctx.wheel_rad)) ref = ctx.wheel_rad;
    else ref = 0.0;
    double goal;
    const bool at_rest = ctx.ego_fresh && std::isfinite(ctx.ego_speed_mps) &&
                         std::abs(ctx.ego_speed_mps) <= p_.center_speed_mps;
    if (stop && at_rest) {
        goal = 0.0;                                   // center only at rest
        r.centered = true;
        addVeto(r.veto, "stop-centering");
    } else if (req.hold_steer || !std::isfinite(req.steer_rad) || (stop && !ctx.ego_fresh)) {
        goal = ref;                                   // keep geometry (unknown / no solution)
    } else {
        goal = req.steer_rad;                         // tracking steering, also while braking
    }
    double dt = std::isfinite(ctx.dt_s) ? std::clamp(ctx.dt_s, 0.0, p_.max_rate_dt_s) : 0.0;
    const double max_d = p_.max_steer_rate_deg_s * M_PI / 180.0 * dt;
    double steer = std::clamp(goal, ref - max_d, ref + max_d);
    r.rate_limited = steer != goal;
    const double c = std::clamp(steer, -max_rad, max_rad);
    r.angle_limited = c != steer;
    steer = c;
    if (!std::isfinite(steer)) steer = 0.0;           // unreachable guard
    r.steer_modified = !(std::isfinite(req.steer_rad) && steer == req.steer_rad);

    r.vel_kmh = vel;
    r.steer_rad = steer;
    s_.have_emitted = true;
    s_.steer_rad = steer;
    s_.speed_kmh = vel;
    return r;
}

void PlannerFeed::onPath(double receipt_s) {
    have_path_ = true;
    path_t_ = receipt_s;
    bound_plan_ = false;
}

bool PlannerFeed::onPlan(double receipt_s, double value_mps) {
    onPath(receipt_s);
    const bool valid = onCap(receipt_s, value_mps);
    bound_plan_ = true;
    return valid;
}

bool PlannerFeed::onCap(double receipt_s, double value_mps) {
    if (bound_plan_) return true;  // cannot identify which path a legacy Float32 belongs to
    have_cap_ = true;
    cap_t_ = receipt_s;
    cap_raw_ = value_mps;
    const bool valid = std::isfinite(value_mps) && value_mps >= 0.0 && value_mps <= p_.cap_max_mps;
    if (!valid) {
        cap_malformed_ = true;                 // stop until a newer valid message
        return false;
    }
    cap_malformed_ = false;
    cap_valid_value_ = value_mps;
    // a valid stop latches; only a newer valid positive cap releases it
    stop_latched_ = value_mps <= p_.stop_cap_mps;
    return true;
}

PlannerFeedSnapshot PlannerFeed::snapshot(double now_s) const {
    PlannerFeedSnapshot s;
    if (!std::isfinite(now_s)) { s.reason = "invalid-clock"; return s; }
    if (!have_path_ || !have_cap_) { s.reason = !have_path_ ? "no-path-yet" : "no-cap-yet"; return s; }
    s.path_age_s = now_s - path_t_;
    s.cap_age_s = now_s - cap_t_;
    const double timeout = p_.timeout_s;
    if (!(timeout > 0.0) || !std::isfinite(timeout)) { s.reason = "invalid-timeout"; return s; }
    if (s.path_age_s < -p_.future_tolerance_s || s.cap_age_s < -p_.future_tolerance_s) {
        s.reason = "future-stamp"; return s;
    }
    if (s.path_age_s > timeout) { s.reason = "path-stale"; return s; }
    if (s.cap_age_s > timeout) { s.reason = "cap-stale"; return s; }
    s.stale = false;
    if (cap_malformed_) {
        s.malformed = true; s.cap_usable = true; s.cap_mps = 0.0; s.reason = "planner-cap-malformed";
        return s;
    }
    s.cap_usable = true;
    if (stop_latched_) {
        s.retained_stop = true; s.cap_mps = 0.0; s.reason = "planner-stop";
        return s;
    }
    s.stop = false;
    s.cap_mps = cap_valid_value_;
    s.reason = "valid";
    return s;
}

}  // namespace moraimpc
