"""Exercise the follower's real C++ feed against reordered path/speed messages."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


class TunnelFollowerContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('g++')
        if compiler is None:
            raise RuntimeError('g++ is required to verify bound planner commands')
        cls.work = tempfile.TemporaryDirectory(prefix='morai-tunnel-feed-')
        cls.addClassCleanup(cls.work.cleanup)
        source = Path(cls.work.name)/'contract.cpp'
        cls.binary = Path(cls.work.name)/'contract'
        source.write_text(r'''#include <cmath>
#include <cstdlib>
#include <limits>
#include "moraimpc/command_boundary.hpp"
int main(int argc, char** argv) {
    if (argc != 2) return 2;
    moraimpc::PlannerFeed feed;
    const int scenario = std::atoi(argv[1]);
    if (scenario == 1) {
        feed.onPath(99.9); feed.onCap(99.9, 0.0);
        feed.onPlan(100.0, 8.0); feed.onCap(100.05, 0.0);
        const auto s = feed.snapshot(100.1);
        return s.stop || std::abs(s.cap_mps-8.0) > 1e-9 ||
               std::abs(s.path_age_s-s.cap_age_s) > 1e-9;
    }
    if (scenario == 2) {
        feed.onPlan(100.0, 0.0); feed.onCap(100.05, 12.0);
        if (!feed.snapshot(100.1).stop) return 1;
        feed.onPlan(100.2, 4.0);
        return feed.snapshot(100.3).stop || feed.snapshot(100.3).cap_mps != 4.0;
    }
    if (scenario == 3) {
        feed.onPlan(100.0, std::numeric_limits<double>::quiet_NaN());
        feed.onCap(100.05, 8.0);
        if (!feed.snapshot(100.1).malformed || !feed.snapshot(100.1).stop) return 1;
        feed.onPlan(100.2, 4.0);
        return feed.snapshot(100.3).stop;
    }
    if (scenario == 4) {
        feed.onPlan(100.0, 8.0); feed.onCap(100.6, 20.0);
        if (!feed.snapshot(100.8).stale) return 1;
        feed.reset();
        if (feed.usesBoundPlan()) return 1;
        feed.onPath(101.0); feed.onCap(101.0, 3.0);
        return feed.snapshot(101.1).stop || feed.snapshot(101.1).cap_mps != 3.0;
    }
    return 2;
}
''')
        subprocess.run([compiler, '-std=c++17', '-O0',
            '-I'+str(ROOT/'autodrive/ws/src/moraimpc/include'), str(source),
            str(ROOT/'autodrive/ws/src/moraimpc/src/command_boundary.cpp'),
            '-o', str(cls.binary)], check=True, capture_output=True, text=True, timeout=30)

    def check_scenario(self, number):
        subprocess.run([str(self.binary), str(number)], check=True, capture_output=True, text=True, timeout=5)

    def test_old_stop_cannot_override_new_path_speed(self):
        self.check_scenario(1)

    def test_r_stop_plan_cannot_be_released_by_old_positive_cap(self):
        self.check_scenario(2)

    def test_malformed_bound_cap_waits_for_a_new_valid_plan(self):
        self.check_scenario(3)

    def test_stale_pair_stops_and_epoch_reset_restores_legacy_input(self):
        self.check_scenario(4)


if __name__ == '__main__':
    unittest.main()
