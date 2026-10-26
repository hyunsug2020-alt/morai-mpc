#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <jsoncpp/json/json.h>

namespace moraimpc {

// Python time.monotonic() and C++ steady_clock both use CLOCK_MONOTONIC
// on the Linux host/container. Keep the original proof deadline; receipt
// must never extend a permission or carry it to the next bound path.
inline double junctionMonotonicNow() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline double junctionLaunchDeadline(const Json::Value& root, double now) {
    const auto& decision = root["decision"];
    const auto& validation = decision["validation"];
    const auto& motion = validation["junction_motion"];
    const auto& live = validation["junction_live"];
    const auto& source = validation["junction_source_launch"];
    const auto& traffic = validation["live_safety"]["junction_traffic"];
    const bool map_entry = traffic["state"].asString() == "MAP_TIMED_ENTRY"
        && motion["cap_mps"].isNumeric() && motion["expires_at"].isNumeric();
    const bool source_entry = traffic["state"].asString() == "ENTER"
        && traffic["gap_checked"].isBool() && traffic["gap_checked"].asBool()
        && source["consumed"].isBool() && source["consumed"].asBool()
        && (source["kind"].asString() == "launch" || source["kind"].asString() == "provisional")
        && source["expires_at"].isNumeric() && decision["allowed_mps"].isNumeric()
        && live["source_frame"] == source["frame"] && source["frame"].isObject();
    if (!std::isfinite(now) || !root["target_speed_mps"].isNumeric()
            || !decision["emergency"].isBool() || decision["emergency"].asBool()
            || !validation["valid"].asBool() || !validation["pose_valid"].asBool()
            || !validation["steering_valid"].asBool()
            || (!map_entry && !source_entry)
            || !root["plan_id"].isIntegral() || root["plan_id"] != decision["plan_id"]
            || !root["source_stamp"].isNumeric()
            || root["source_stamp"] != decision["source_stamp"]
            || !live["expires_at"].isNumeric()) return 0.;
    const double target = root["target_speed_mps"].asDouble();
    const double cap = map_entry ? motion["cap_mps"].asDouble() : decision["allowed_mps"].asDouble();
    const double end = std::min(map_entry ? motion["expires_at"].asDouble()
        : source["expires_at"].asDouble(), live["expires_at"].asDouble());
    const auto& digest = decision["path_digest"];
    if (!digest.isString() || digest.asString().empty()
            || digest != validation["geometry_path_digest"]
            || digest != validation["traffic_path_digest"]
            || !std::isfinite(target) || !std::isfinite(cap) || !std::isfinite(end)
            || target <= .2 || target > 30./3.6 || target > cap + 1e-6
            || end <= now || end-now > .35) return 0.;
    return end;
}

inline double junctionLaunchTarget(double ramped_kmh, double checked_target_mps,
                                  double deadline, double now, bool eligible) {
    if (!eligible || !std::isfinite(now) || !std::isfinite(deadline) || now >= deadline
            || deadline-now > .35 || !std::isfinite(checked_target_mps)
            || checked_target_mps <= .2 || checked_target_mps > 30./3.6) return ramped_kmh;
    // This is a speed target for throttle feedback, not an instantaneous
    // physical velocity. The MPC prediction still starts from measured speed
    // and retains its physical acceleration model and steering limits.
    return std::max(ramped_kmh, checked_target_mps * 3.6);
}

}  // namespace moraimpc
