#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <osqp.h>

namespace moraimpc {

// 운동학 자전거 NMPC (저속·후진 전용)
//   상태  x = [px, py, yaw]      (3)
//   입력  u = [delta]              (1)
//   속도  v_k는 외부 reference (signed; 후진=음수)
struct NMPCConfig {
    int    N            = 15;     // horizon (15 * 0.05 = 0.75s)
    double Ts           = 0.05;
    double L            = 2.7;

    // 비용 가중치
    double w_lat        = 350.0;  // 측방오차 (저속 정확도 우선)
    double w_yaw        = 150.0;  // 헤딩오차
    double w_delta      = 5.0;    // 조향 절대크기
    double w_ddelta     = 120.0;  // 조향 변화율 (저속 안정)

    // 제약
    double max_steer_deg = 35.0;

    // SQP 반복 (운동학 모델은 1~2회면 수렴)
    int    sqp_iters    = 2;
};

struct NMPCRef {
    std::vector<double> px;   // 길이 N, k=1..N
    std::vector<double> py;
    std::vector<double> yaw;
    std::vector<double> v;    // signed (R: 음수)
};

struct NMPCResult {
    bool   success     = false;
    double steer_rad   = 0.0;
    double solve_ms    = 0.0;
    double e_lat_pred  = 0.0;
    double e_yaw_pred  = 0.0;
    std::vector<double> traj_x;
    std::vector<double> traj_y;
};

class NMPCController {
public:
    explicit NMPCController(const NMPCConfig& cfg);
    ~NMPCController();

    NMPCResult solve(const NMPCRef& ref,
                     double cur_x, double cur_y, double cur_yaw,
                     double prev_steer_rad);

    void reset();   // warm-start 폐기 (기어 전환 후 호출)

private:
    NMPCConfig      cfg_;
    OSQPWorkspace*  work_     = nullptr;
    OSQPSettings*   settings_ = nullptr;
    int             prev_n_   = -1;

    Eigen::VectorXd u_warm_;   // 직전 해 (size N)
    bool            has_warm_ = false;

    csc* eigenToCsc(const Eigen::SparseMatrix<double>& mat);
};

}  // namespace moraimpc
