"""Validate the competition CSV with the controller's actual C++ loader."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'autodrive/ros/ltv_speed_profile_competition.csv'


class LTVCompetitionProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('g++')
        if compiler is None:
            raise RuntimeError('g++ is required to validate the actual LTV CSV loader')
        cls.work = tempfile.TemporaryDirectory(prefix='morai-ltv-profile-')
        cls.addClassCleanup(cls.work.cleanup)
        source = Path(cls.work.name) / 'profile.cpp'
        cls.binary = Path(cls.work.name) / 'profile'
        source.write_text(r'''#include <iomanip>
#include <iostream>
#include "moraimpc/ltv_speed_profile.hpp"
int main(int argc, char** argv) {
    try {
        if (argc != 2) return 1;
        const auto rows = moraimpc::loadLTVSpeedProfile(argv[1]);
        const moraimpc::LTVMPCConfig base;
        std::cout << rows.size() << " " << rows.front().speed_kmh
                  << " " << rows.back().speed_kmh << "\n";
        std::cout << std::setprecision(12);
        for (const double speed : {0., 20., 40., 60., 80., 90.}) {
            const auto cfg = moraimpc::ltvConfigAtSpeed(base, rows, speed);
            std::cout << speed << " " << cfg.w_u << " " << cfg.w_dr
                      << " " << cfg.w_theta_low_speed << " " << cfg.w_theta_high_speed
                      << " " << cfg.w_kappa << " " << cfg.w_u_v_gain
                      << " " << cfg.w_dr_curve_boost << " " << cfg.target_vel << "\n";
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        return 2;
    }
}
''')
        subprocess.run([
            compiler, '-std=c++17', '-O0',
            '-I' + str(ROOT / 'autodrive/ws/src/moraimpc/include'),
            '-I/usr/include/eigen3', str(source), '-o', str(cls.binary),
        ], check=True, capture_output=True, text=True, timeout=30)

    def run_profile(self, path=PROFILE):
        return subprocess.run([str(self.binary), str(path)],
                              capture_output=True, text=True, timeout=5)

    def test_production_profile_loads_and_applies_8000_at_every_speed(self):
        result = self.run_profile()
        self.assertEqual(result.returncode, 0, result.stderr)
        lines = result.stdout.splitlines()
        self.assertEqual(list(map(float, lines[0].split())), [5., 0., 80.])
        for line in lines[1:]:
            speed, w_u, *_ = map(float, line.split())
            # Offline-tuned 2026-10-06: 4000 up to 40 km/h, 6000 at 60, 10000 at 80+.
            expected = 4000. if speed <= 40. else 6000. if speed <= 60. else 10000.
            self.assertEqual(w_u, expected, f'{speed} km/h silently uses different steering weight')

    def test_90_kmh_clamps_weights_without_changing_target_velocity(self):
        result = self.run_profile()
        self.assertEqual(result.returncode, 0, result.stderr)
        configs = [list(map(float, line.split())) for line in result.stdout.splitlines()[1:]]
        at80, at90 = configs[-2:]
        self.assertEqual(at90[0], 90.)
        self.assertEqual(at80[1:], at90[1:])
        # Weight interpolation's 80 km/h endpoint is independent of speed commands.
        self.assertEqual(at90[-1], 16.67)

    def test_duplicate_90_row_reproduces_recorded_loader_rejection(self):
        bad = Path(self.work.name) / 'bad_90.csv'
        bad.write_text(PROFILE.read_text() + '90.0,220.0,250.0,220.0,120.0,8000.0,350.0,2.8\n')
        result = self.run_profile(bad)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(result.stderr.strip(), 'out-of-range LTV speed CSV row')


if __name__ == '__main__':
    unittest.main()
