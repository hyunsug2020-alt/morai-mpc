#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "moraimpc/ltv_types.hpp"

namespace moraimpc {

struct LTVSpeedRow {
    double speed_kmh = 0.0;
    double w_dr = 220.0;
    double w_theta_low_speed = 250.0;
    double w_theta_high_speed = 220.0;
    double w_kappa = 120.0;
    double w_u = 5200.0;
    double w_u_v_gain = 350.0;
    double w_dr_curve_boost = 2.8;
};

inline std::vector<LTVSpeedRow> loadLTVSpeedProfile(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open LTV speed CSV: " + path);
    const std::string expected = "speed_kmh,w_dr,w_theta_low_speed,w_theta_high_speed,w_kappa,w_u,w_u_v_gain,w_dr_curve_boost";
    std::string line;
    if (!std::getline(input, line) || line != expected)
        throw std::runtime_error("invalid LTV speed CSV header");
    std::vector<LTVSpeedRow> rows;
    while (std::getline(input, line)) {
        if (line.empty()) continue;
        std::stringstream stream(line);
        std::string cell;
        std::array<double, 8> values{};
        for (double& value : values) {
            if (!std::getline(stream, cell, ',')) throw std::runtime_error("short LTV speed CSV row");
            size_t used = 0;
            value = std::stod(cell, &used);
            if (used != cell.size() || !std::isfinite(value)) throw std::runtime_error("invalid LTV speed CSV value");
        }
        if (std::getline(stream, cell, ',')) throw std::runtime_error("extra LTV speed CSV column");
        LTVSpeedRow row{values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7]};
        if ((!rows.empty() && row.speed_kmh <= rows.back().speed_kmh) || row.speed_kmh < 0 || row.speed_kmh > 80 ||
            row.w_dr < 20 || row.w_dr > 1000 || row.w_theta_low_speed < 20 || row.w_theta_low_speed > 1000 ||
            row.w_theta_high_speed < 20 || row.w_theta_high_speed > 1000 || row.w_kappa < 0 || row.w_kappa > 1000 ||
            row.w_u < 100 || row.w_u > 30000 || row.w_u_v_gain < 0 || row.w_u_v_gain > 3000 ||
            row.w_dr_curve_boost < 1 || row.w_dr_curve_boost > 5)
            throw std::runtime_error("out-of-range LTV speed CSV row");
        rows.push_back(row);
    }
    if (rows.size() < 2 || rows.size() > 10 || rows.front().speed_kmh != 0.0 || rows.back().speed_kmh != 80.0)
        throw std::runtime_error("LTV speed CSV needs 2-10 rows from 0 to 80 km/h");
    return rows;
}

inline LTVMPCConfig ltvConfigAtSpeed(const LTVMPCConfig& base, const std::vector<LTVSpeedRow>& rows,
                                     double speed_kmh) {
    if (rows.empty()) return base;
    const double speed = std::clamp(speed_kmh, rows.front().speed_kmh, rows.back().speed_kmh);
    auto upper = std::lower_bound(rows.begin(), rows.end(), speed,
                                  [](const LTVSpeedRow& row, double v) { return row.speed_kmh < v; });
    const LTVSpeedRow& hi = upper == rows.end() ? rows.back() : *upper;
    const LTVSpeedRow& lo = upper == rows.begin() ? hi : *(upper - 1);
    const double t = hi.speed_kmh == lo.speed_kmh ? 0.0 : (speed - lo.speed_kmh) / (hi.speed_kmh - lo.speed_kmh);
    const auto blend = [t](double a, double b) { return a + (b - a) * t; };
    LTVMPCConfig cfg = base;
    cfg.w_dr = blend(lo.w_dr, hi.w_dr);
    cfg.w_theta_low_speed = blend(lo.w_theta_low_speed, hi.w_theta_low_speed);
    cfg.w_theta_high_speed = blend(lo.w_theta_high_speed, hi.w_theta_high_speed);
    cfg.w_kappa = blend(lo.w_kappa, hi.w_kappa);
    cfg.w_u = blend(lo.w_u, hi.w_u);
    cfg.w_u_v_gain = blend(lo.w_u_v_gain, hi.w_u_v_gain);
    cfg.w_dr_curve_boost = blend(lo.w_dr_curve_boost, hi.w_dr_curve_boost);
    return cfg;
}

}  // namespace moraimpc
