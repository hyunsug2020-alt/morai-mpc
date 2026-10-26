"""Validate the competition rolling-stop steering policy with the actual C++ boundary."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]


class RollingStopSteeringTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.work = tempfile.TemporaryDirectory(prefix='morai-stop-steering-')
        cls.addClassCleanup(cls.work.cleanup)
        source = Path(cls.work.name)/'boundary.cpp'
        cls.binary = Path(cls.work.name)/'boundary'
        source.write_text(r'''#include <cassert>
#include <cmath>
#include <iostream>
#include "moraimpc/command_boundary.hpp"
int main(int argc, char** argv) {
    moraimpc::BoundaryParams p;
    p.center_speed_mps = std::stod(argv[1]);
    p.max_steer_rate_deg_s = 30.;
    moraimpc::CommandBoundary boundary(p);
    auto& state = boundary.mutableState();
    state.have_emitted = true;
    state.steer_rad = .2;
    moraimpc::BoundaryRequest req;
    req.steer_rad = .2;
    req.filter_bypass = true;
    moraimpc::BoundaryContext ctx;
    ctx.use_planner_path = true;
    ctx.planner_cap_fresh = true;
    ctx.dt_s = .05;
    ctx.ego_speed_mps = .4;
    ctx.planner_cap_mps = 0.;
    auto rolling = boundary.emit(req, ctx);
    std::cout << rolling.vel_kmh << ' ' << rolling.steer_rad << ' ' << rolling.centered << '\n';
    double change = 0.;
    for (int i=0; i<20; ++i) {
        const double previous = boundary.state().steer_rad;
        ctx.planner_cap_mps = i%2 ? .5 : 0.;
        req.vel_kmh = i%2 ? 1.8 : 0.;
        auto result = boundary.emit(req, ctx);
        change += std::abs(result.steer_rad-previous);
        assert(std::abs(result.steer_rad-previous) <= 30.*M_PI/180.*.05+1e-12);
        if (!ctx.planner_cap_mps) assert(result.vel_kmh == 0.);
    }
    std::cout << change << '\n';
    ctx.planner_cap_mps=0.; ctx.ego_speed_mps=.15;
    auto rest = boundary.emit(req, ctx);
    assert(rest.vel_kmh == 0.);
    std::cout << rest.centered << '\n';
    const double previous = boundary.state().steer_rad;
    ctx.ego_fresh=false;
    auto stale = boundary.emit(req, ctx);
    assert(stale.vel_kmh == 0.);
    assert(stale.steer_rad == previous);
    assert(!stale.centered);
}
''')
        package = ROOT/'autodrive/ws/src/moraimpc'
        subprocess.run([shutil.which('g++') or 'g++', '-std=c++17', '-O2',
                        '-I'+str(package/'include'), str(source),
                        str(package/'src/command_boundary.cpp'), '-o', str(cls.binary)],
                       check=True, capture_output=True, text=True, timeout=30)

    def threshold(self):
        launch = ET.parse(ROOT/'autodrive/ros/follower_student_competition.launch')
        arg = launch.find("./include/arg[@name='center_steer_stop_speed_mps']")
        return float(arg.attrib['value'])

    def run_boundary(self, threshold):
        result = subprocess.run([str(self.binary), str(threshold)], check=True,
                                capture_output=True, text=True, timeout=5)
        return [list(map(float, line.split())) for line in result.stdout.splitlines()]

    def test_rolling_stop_keeps_tracking_steering_and_zero_speed(self):
        self.assertEqual(self.threshold(), .15)
        rolling, _, rest = self.run_boundary(self.threshold())
        self.assertEqual(rolling, [0., .2, 0.])
        self.assertEqual(rest, [1.])

    def test_repeated_creep_stop_does_not_reverse_steering_towards_zero(self):
        old, new = self.run_boundary(.5), self.run_boundary(self.threshold())
        self.assertEqual(old[0][2], 1.)
        self.assertGreater(old[1][0], .4)
        self.assertEqual(new[1][0], 0.)

    def test_include_passes_the_competition_threshold_to_the_actual_node(self):
        launch = ET.parse(ROOT/'autodrive/ws/src/moraimpc/launch/hdmap_path_follower.launch')
        self.assertIsNotNone(launch.find("./arg[@name='center_steer_stop_speed_mps']"))
        param = launch.find(".//param[@name='center_steer_stop_speed_mps']")
        self.assertEqual(param.attrib['value'], '$(arg center_steer_stop_speed_mps)')


if __name__ == '__main__':
    unittest.main()
