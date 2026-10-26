#pragma once

#include <algorithm>
#include <cmath>
#include <jsoncpp/json/json.h>

namespace moraimpc {

inline double junctionCurveDeadline(const Json::Value& root, double now) {
    const auto& decision = root["decision"];
    const auto& validation = decision["validation"];
    const auto& profile = validation["junction_curve_profile"];
    const auto& digest = decision["path_digest"];
    if (!std::isfinite(now) || !profile.isObject()
            || profile["kind"].asString() != "committed_ring"
            || !profile["expires_at"].isNumeric()
            || !profile["target_mps"].isNumeric()
            || !decision["emergency"].isBool() || decision["emergency"].asBool()
            || !validation["valid"].asBool() || !validation["pose_valid"].asBool()
            || !validation["steering_valid"].asBool()
            || !root["plan_id"].isIntegral() || root["plan_id"] != decision["plan_id"]
            || !root["source_stamp"].isNumeric()
            || root["source_stamp"] != decision["source_stamp"]
            || !digest.isString() || digest.asString().empty()
            || digest != validation["geometry_path_digest"]
            || digest != validation["traffic_path_digest"]) return 0.;
    const double end = profile["expires_at"].asDouble();
    const double target = profile["target_mps"].asDouble();
    if (!std::isfinite(end) || end <= now || end-now > .35
            || !std::isfinite(target) || target <= 0. || target > 25./3.6) return 0.;
    return end;
}

// The distant bend owns the speed at its station. Retain a response reserve
// before braking towards it, without imposing that bend's speed everywhere.
inline double junctionCurveApproachSpeed(double curve_mps, double distance_m,
                                         double ceiling_mps, double brake_mps2) {
    if (!std::isfinite(curve_mps) || !std::isfinite(distance_m)
            || !std::isfinite(ceiling_mps) || !std::isfinite(brake_mps2)
            || curve_mps < 0. || ceiling_mps < 0. || brake_mps2 <= 0.) return 0.;
    const double room = std::max(0., distance_m - .5*ceiling_mps);
    return std::min(ceiling_mps,
        std::sqrt(curve_mps*curve_mps + 2.*brake_mps2*room));
}

}  // namespace moraimpc
