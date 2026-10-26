"""Exercise fresh bound entry permissions through the C++ command boundary."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class FirmJunctionAccelerationTests(unittest.TestCase):
    def test_actual_cpp_permission_and_stop_boundary(self):
        root = Path(__file__).resolve().parents[2]
        core = root / 'autodrive/ws/src/moraimpc'
        with tempfile.TemporaryDirectory(prefix='morai-firm-entry-') as work:
            source = Path(work) / 'test.cpp'
            binary = Path(work) / 'test'
            source.write_text(r'''
#include <cassert>
#include <limits>
#include "moraimpc/junction_launch.hpp"
#include "moraimpc/command_boundary.hpp"
using namespace moraimpc;
int main() {
    Json::Value p;
    p["plan_id"] = 715; p["source_stamp"] = 1792579713.57;
    p["target_speed_mps"] = 3.937409625621612;
    auto& d = p["decision"];
    d["plan_id"] = 715; d["source_stamp"] = p["source_stamp"];
    d["emergency"] = false; d["path_digest"] = "same-published-world-path";
    auto& v = d["validation"];
    v["valid"] = v["pose_valid"] = v["steering_valid"] = true;
    v["geometry_path_digest"] = v["traffic_path_digest"] = d["path_digest"];
    v["junction_motion"]["cap_mps"] = 3.937409625621612;
    v["junction_motion"]["expires_at"] = 100.1;
    v["junction_live"]["expires_at"] = 100.2;
    v["live_safety"]["junction_traffic"]["state"] = "MAP_TIMED_ENTRY";
    const auto deadline = junctionLaunchDeadline(p, 100.);
    assert(deadline == 100.1);
    const auto target = junctionLaunchTarget(.54, 3.937409625621612, deadline, 100.01, true);
    assert(std::abs(target - 14.174674652237803) < 1e-8);
    // The previously emitted .54 km/h target held brake .35. The new target
    // immediately produces full throttle in the deployed .35 gain controller.
    assert(std::clamp(.35 * target / 3.6, 0., 1.) == 1.);
    CommandBoundary boundary;
    BoundaryRequest req;
    req.vel_kmh = target; req.filter_bypass = true;
    BoundaryContext ctx;
    ctx.use_planner_path = ctx.planner_cap_fresh = ctx.ego_fresh = true;
    ctx.planner_cap_mps = 3.937409625621612;
    ctx.safety_cap_mps = 4.; ctx.dt_s = .05;
    assert(std::abs(boundary.emit(req, ctx).vel_kmh - target) < 1e-8);
    ctx.safety_cap_mps = 1.;
    assert(boundary.emit(req, ctx).vel_kmh == 3.6);
    ctx.planner_cap_mps = 0.;
    assert(boundary.emit(req, ctx).vel_kmh == 0.);
    ctx.planner_cap_mps = 4.; ctx.planner_cap_fresh = false;
    assert(boundary.emit(req, ctx).vel_kmh == 0.);
    ctx.planner_cap_fresh = true; ctx.ego_fresh = false;
    assert(boundary.emit(req, ctx).vel_kmh == 0.);
    ctx.ego_fresh = true; req.stop = true;
    assert(boundary.emit(req, ctx).vel_kmh == 0.);
    assert(junctionLaunchTarget(.54, 4., deadline, 100.11, true) == .54);
    assert(junctionLaunchTarget(.54, 4., deadline, 100.01, false) == .54);
    assert(junctionLaunchTarget(.54, 4., deadline, std::numeric_limits<double>::quiet_NaN(), true) == .54);
    assert(junctionLaunchDeadline(p, 100.11) == 0.);
    assert(junctionLaunchDeadline(p, 99.) == 0.);
    auto bad = p; bad["decision"]["emergency"] = true;
    assert(junctionLaunchDeadline(bad, 100.) == 0.);
    bad = p; bad["plan_id"] = 716;
    assert(junctionLaunchDeadline(bad, 100.) == 0.);
    bad = p; bad["decision"]["validation"]["traffic_path_digest"] = "older-path";
    assert(junctionLaunchDeadline(bad, 100.) == 0.);
    bad = p; bad["decision"]["validation"]["live_safety"]["junction_traffic"]["state"] = "CROSSING_CAUTION";
    assert(junctionLaunchDeadline(bad, 100.) == 0.);
    bad = p; bad["decision"]["mode"] = "HIGHWAY"; bad["decision"]["validation"].removeMember("junction_motion");
    assert(junctionLaunchDeadline(bad, 100.) == 0.);
    auto entry = p;
    auto& ev = entry["decision"]["validation"];
    ev.removeMember("junction_motion");
    entry["decision"]["allowed_mps"] = 4.;
    ev["live_safety"]["junction_traffic"]["state"] = "ENTER";
    ev["live_safety"]["junction_traffic"]["gap_checked"] = true;
    ev["junction_source_launch"]["kind"] = "launch";
    ev["junction_source_launch"]["consumed"] = true;
    ev["junction_source_launch"]["expires_at"] = 100.1;
    ev["junction_source_launch"]["frame"]["stamp"] = 90.;
    ev["junction_live"]["source_frame"] = ev["junction_source_launch"]["frame"];
    assert(junctionLaunchDeadline(entry, 100.) == 100.1);
    ev["junction_source_launch"]["consumed"] = false;
    assert(junctionLaunchDeadline(entry, 100.) == 0.);
}
''')
            subprocess.run(['g++', '-std=c++17', '-O0', '-I' + str(core / 'include'),
                            str(source), str(core / 'src/command_boundary.cpp'),
                            '-ljsoncpp', '-o', str(binary)], check=True,
                           capture_output=True, text=True, timeout=30)
            subprocess.run([str(binary)], check=True, capture_output=True,
                           text=True, timeout=5)


if __name__ == '__main__':
    unittest.main()
