#pragma once

#include "moraimpc/command_boundary.hpp"

#include <cmath>
#include <string>

namespace moraimpc {

// Optional body-frame wheel/CAN speed for the rollback guard only. The ESKF
// remains the source for tracking, speed error and planned-stop holding.
class SignedRollbackSpeed {
public:
    void receive(double speed_mps, double source_s, double receipt_s,
                 const std::string& frame_id, double timeout_s, double tolerance_s) {
        if (discard_next_) { discard_next_ = false; return; }
        valid_ = false;  // an invalid new packet must not keep an older override alive
        if (frame_id != "base_link" || !std::isfinite(speed_mps) ||
            !(std::isfinite(source_s) && source_s > 0.0) ||
            !checkAge(true, receipt_s, source_s, timeout_s, tolerance_s).fresh) return;
        speed_mps_ = speed_mps;
        source_s_ = source_s;
        receipt_s_ = receipt_s;
        valid_ = true;
    }

    double valueOr(double fallback_mps, double now_s,
                   double timeout_s, double tolerance_s) const {
        return valid_ &&
               checkAge(true, now_s, source_s_, timeout_s, tolerance_s).fresh &&
               checkAge(true, now_s, receipt_s_, timeout_s, tolerance_s).fresh
            ? speed_mps_ : fallback_mps;
    }

    void resetEpoch() {
        valid_ = false;
        // Queue size is one; the first processed packet may predate the reset.
        discard_next_ = true;
    }

private:
    bool valid_ = false;
    bool discard_next_ = false;
    double speed_mps_ = 0.0;
    double source_s_ = 0.0;
    double receipt_s_ = 0.0;
};

}  // namespace moraimpc
