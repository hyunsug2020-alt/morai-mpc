#pragma once

#include <cmath>
#include <string>

namespace moraimpc {

inline bool highwayPlanOwnsSpeed(bool use_avoid_path,
                                 const std::string& planner_mode,
                                 double planner_mode_age_s,
                                 bool feed_usable,
                                 double planner_cap_mps) {
    // Only a fresh, usable highway plan bypasses the normal-road speed
    // heuristic. Its planner already checks curvature, bodies and braking.
    return use_avoid_path && planner_mode == "HIGHWAY" && feed_usable &&
           std::isfinite(planner_mode_age_s) && planner_mode_age_s >= 0.0 &&
           planner_mode_age_s < 0.5 && std::isfinite(planner_cap_mps) &&
           planner_cap_mps > 0.05;
}

inline bool plannerSafeStopIsFresh(bool use_avoid_path,
                                   const std::string& planner_mode,
                                   double planner_mode_age_s,
                                   double timeout_s = 0.5) {
    return use_avoid_path && planner_mode == "SAFE_STOP" &&
           std::isfinite(planner_mode_age_s) && planner_mode_age_s >= 0.0 &&
           planner_mode_age_s < timeout_s;
}

inline double applyPlannerSafeStop(double requested_kmh,
                                   bool use_avoid_path,
                                   const std::string& planner_mode,
                                   double planner_mode_age_s,
                                   double timeout_s = 0.5) {
    return plannerSafeStopIsFresh(
               use_avoid_path, planner_mode, planner_mode_age_s, timeout_s)
        ? 0.0
        : requested_kmh;
}

}  // namespace moraimpc
