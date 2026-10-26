"""Exercise the C++ mode contract used by the live highway follower."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


class HighwayFollowerContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('g++')
        if compiler is None:
            raise RuntimeError('g++ is required to verify the live follower mode contract')
        cls.work = tempfile.TemporaryDirectory(prefix='morai-highway-contract-')
        cls.addClassCleanup(cls.work.cleanup)
        source = Path(cls.work.name) / 'contract.cpp'
        cls.binary = Path(cls.work.name) / 'contract'
        source.write_text(r'''#include <cstdlib>
#include <iostream>
#include "moraimpc/path_follower_safety.hpp"
int main(int argc, char** argv) {
    if (argc != 6) return 2;
    std::cout << moraimpc::highwayPlanOwnsSpeed(
        std::atoi(argv[1]) != 0, argv[2], std::atof(argv[3]),
        std::atoi(argv[4]) != 0, std::atof(argv[5]));
}
''')
        subprocess.run([
            compiler, '-std=c++17', '-O0',
            '-I' + str(ROOT / 'autodrive/ws/src/moraimpc/include'),
            str(source), '-o', str(cls.binary),
        ], check=True, capture_output=True, text=True, timeout=30)

    def owns_speed(self, mode='HIGHWAY', age=.1, usable=True, cap=25., enabled=True):
        result = subprocess.run([
            str(self.binary), str(int(enabled)), mode, str(age),
            str(int(usable)), str(cap),
        ], check=True, capture_output=True, text=True, timeout=5)
        return result.stdout == '1'

    def test_fresh_highway_plan_owns_speed(self):
        self.assertTrue(self.owns_speed())

    def test_normal_intersection_tunnel_and_return_modes_keep_general_policy(self):
        for mode in ('CRUISE', 'AVOID', 'RETURN', 'SAFE_STOP', '', 'highway'):
            with self.subTest(mode=mode):
                self.assertFalse(self.owns_speed(mode=mode))

    def test_stale_missing_or_future_mode_does_not_bypass_general_policy(self):
        for age in (.5, 1., -1., float('nan'), float('inf')):
            with self.subTest(age=age):
                self.assertFalse(self.owns_speed(age=age))

    def test_stopped_stale_or_invalid_plan_cannot_bypass_general_policy(self):
        self.assertFalse(self.owns_speed(usable=False))
        self.assertFalse(self.owns_speed(enabled=False))
        for cap in (0., .05, -1., float('nan'), float('inf')):
            with self.subTest(cap=cap):
                self.assertFalse(self.owns_speed(cap=cap))


if __name__ == '__main__':
    unittest.main()
