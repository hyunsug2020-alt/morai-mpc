#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <osqp.h>
#include <string>
#include <vector>
#include "moraimpc/ltv_types.hpp"

namespace moraimpc {

class LTVSolver {
public:
    LTVSolver(const LTVMPCConfig& cfg);
    ~LTVSolver();
    // owns raw OSQP workspace/settings pointers: copying would double free
    LTVSolver(const LTVSolver&) = delete;
    LTVSolver& operator=(const LTVSolver&) = delete;
    bool solve(const Eigen::SparseMatrix<double>& P,
               const Eigen::VectorXd& q,
               const Eigen::SparseMatrix<double>& A,
               const Eigen::VectorXd& l,
               const Eigen::VectorXd& u,
               Eigen::VectorXd& solution);

    // Diagnostics of the last solve() call (revision2 QP error contract).
    // last_error: "" on success, else "invalid-input:<what>", "setup:<code>", "update_P:<code>",
    // "update_lin_cost:<code>", "update_A:<code>", "update_bounds:<code>", "solve:<code>",
    // "status:<status_val>", "nonfinite-solution". Any error invalidates the workspace, so the
    // next valid call performs a fresh setup; an old solution is never returned as a new solve.
    const std::string& lastError() const { return last_error_; }
    int lastStatus() const { return last_status_; }
    bool hasWorkspace() const { return work_ != nullptr; }
    int setupCount() const { return setup_count_; }
    // TEST HOOK ONLY: disable input validation so that the OSQP return-code checks themselves
    // can be exercised by fault injection. Production code never calls this (default: on).
    void setInputValidationForTest(bool on) { validate_inputs_ = on; }

private:
    void invalidate();
    std::string last_error_;
    int last_status_ = 0;
    int setup_count_ = 0;
    bool validate_inputs_ = true;
    LTVMPCConfig cfg_;
    OSQPWorkspace* work_ = nullptr;
    OSQPSettings* settings_ = nullptr;
    int prev_A_rows_ = -1;   // A 크기 변경 감지용
    // Sparsity patterns used at setup. OSQP value updates assume an identical
    // pattern; a changed pattern (e.g. v-dependent entries becoming exactly 0
    // at zero predicted speed) requires a new setup.
    std::vector<int> P_outer_, P_inner_, A_outer_, A_inner_;

    csc* eigenToCsc(const Eigen::SparseMatrix<double>& mat);
};

} // namespace moraimpc
