#pragma once

#include <array>
#include <vector>

namespace bisa {

struct LTVMPCConfig {
  int N = 25;
  double Ts = 0.05;

  double wheelbase = 0.30;
  double max_velocity = 3.0;
  double min_velocity = 0.3;

  double w_d = 20.0;
  double w_theta = 8.0;
  double w_kappa = 1.0;
  double w_u = 1.5;
  double w_terminal_scale = 1.0;

  double u_min = -2.0;
  double u_max = 2.0;
  double kappa_min = -2.0;
  double kappa_max = 2.0;
  int ref_preview_steps = 2;
  double curvature_speed_gain = 2.5;
  double kappa_blend_alpha = 0.15;

  // Lateral deviation bound [m] from Gutjahr 2017 eq.(8).
  // d_i,min <= d_i(k) <= d_i,max  (i=1,2,3: rear/mid/front axle)
  // Set > 0 to enable. Negative = disabled (cost-only mode).
  double lateral_bound = -1.0;
  // Slack variable penalty weights (Gutjahr 2017 eq.21, k1/K2)
  // Large values = strongly penalize path deviation, but never cause infeasibility.
  double w_lateral_slack_lin  = 500.0;
  double w_lateral_slack_quad = 5000.0;

  int osqp_max_iter = 4000;
  double osqp_eps_abs = 1e-6;
  double osqp_eps_rel = 1e-6;
  bool osqp_polish = true;
  bool osqp_warm_start = true;
};

struct MPCCommand {
  double v_cmd = 0.0;
  double omega_cmd = 0.0;
  double kappa_cmd = 0.0;
  double model_time_us = 0.0;
  double solver_time_us = 0.0;
  bool solved = false;
  std::vector<double> u_seq;
  std::vector<std::array<double, 3>> predicted_xy;
};

static constexpr int kLTVNx = 5;
static constexpr int kLTVNu = 1;
static constexpr int kLTVNy = 4;

}  // namespace bisa
