"""Compile the production rollback helper and longitudinal command branch without ROS."""
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parent
MPC = ROOT / "ws/src/moraimpc"


class SignedRollbackSpeedTests(unittest.TestCase):
    def test_recorded_restart_and_guard_failure_paths(self):
        source = (MPC / "src/path_follower.cpp").read_text()
        # Exercise the real command mapping, including planned-stop priority and
        # explicit safety braking, rather than duplicating its logic in a test.
        start = source.index("    if (longitudinal_cmd_type_ == 1) {")
        end = source.index("    // C4: remember what was actually emitted", start)
        mapping = source[start:end]
        harness = r'''
#include "moraimpc/signed_rollback_speed.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

using moraimpc::SignedRollbackSpeed;
constexpr double timeout = 0.25, tolerance = 0.001;
struct Time { double value; double toSec() const { return value; } };
struct Cmd { double accel = 0, brake = 0, velocity = 0; };

Cmd command(SignedRollbackSpeed signed_rollback_speed_, double now, double target_kmh,
            double cur_v_signed_, double cur_v_, int cur_gear_ = 1,
            bool stop = false, bool full_brake = false, bool planner_stop = false,
            int longitudinal_cmd_type_ = 1) {
    const double speed_control_deadband_mps_ = 0.15;
    const double speed_accel_kp_ = 0.35, speed_brake_kp_ = 0.45, stop_hold_brake_ = 0.35;
    const double signed_speed_timeout_s_ = timeout, clock_tolerance_s_ = tolerance;
    Time now_b{now};
    moraimpc::CommandBoundary boundary;
    moraimpc::BoundaryRequest req;
    req.vel_kmh = target_kmh;
    req.filter_bypass = true;
    req.stop = stop;
    moraimpc::BoundaryContext ctx;
    ctx.ego_fresh = true;
    ctx.ego_speed_mps = cur_v_;
    ctx.use_planner_path = true;
    ctx.planner_cap_fresh = true;
    ctx.planner_cap_mps = planner_stop ? 0.0 : 20.0;
    const auto br = boundary.emit(req, ctx);
    double vel_kmh = br.vel_kmh;
    Cmd cmd;
''' + mapping + r'''
    return cmd;
}

void receive(SignedRollbackSpeed& speed, double vx = 0.0, double stamp = 100.0,
             double receipt = 100.01, const char* frame = "base_link") {
    speed.receive(vx, stamp, receipt, frame, timeout, tolerance);
}

int main() {
    SignedRollbackSpeed speed;
    assert(speed.valueOr(-0.22, 100.02, timeout, tolerance) == -0.22);
    assert(command(speed, 100.02, 11.0, -0.22, 0.22).brake == 1.0);

    // Native reception fixture: follow-20261017T113528Z-2010283,
    // snapshot wall_ns=1792237047836244131; emission seq=1526.
    // Snapshots establish contemporaneous values, not causal message ordering.
    constexpr double stamp = 1792237047.825785160;
    constexpr double receipt = 1792237047.826170664;
    constexpr double publication = 1792237047.8327818;
    constexpr double eskf = -0.21861335110248956;
    constexpr double can = 0.00019553786842152476;
    SignedRollbackSpeed recorded;
    recorded.receive(can, stamp, receipt, "base_link", timeout, tolerance);
    const double measured = std::hypot(eskf, -0.004421743768415705);
    const Cmd restart = command(recorded, publication, 11.021169665372623, eskf, measured);
    assert(restart.accel > 0.0 && restart.brake == 0.0);
    assert(command(SignedRollbackSpeed{}, publication, 11.021169665372623,
                   eskf, measured).brake == 1.0);

    // Only rollback direction changes source. The servo still uses ESKF speed.
    receive(speed, 4.0);
    const Cmd tracking = command(speed, 100.02, 7.2, 1.0, 1.0);
    assert(std::abs(tracking.accel - 0.35) < 1e-12 && tracking.brake == 0.0);
    receive(speed, -0.4);
    assert(command(speed, 100.02, 11.0, 0.0, 0.0).brake == 1.0);  // D rollback
    assert(command(speed, 100.02, 11.0, -0.2, 0.2, -1).accel > 0.0);  // R travel
    receive(speed, 0.4);
    assert(command(speed, 100.02, 11.0, 0.0, 0.0, -1).brake == 1.0);  // R rollback

    // A fresh CAN sample cannot release any planned or explicit stop.
    receive(speed, 0.0);
    assert(command(speed, 100.02, 0.0, 0.0, 0.0).brake == 0.35);
    const Cmd hold = command(speed, 100.02, 0.0, 2.0, 2.0);
    assert(hold.accel == 0.0 && hold.brake == 0.9);
    const Cmd people_stop = command(speed, 100.02, 11.0, 0.0, 0.0, 1, false, false, true);
    assert(people_stop.accel == 0.0 && people_stop.brake == 0.35);
    const Cmd safety = command(speed, 100.02, 11.0, 0.0, 0.0, 1, true, true);
    assert(safety.accel == 0.0 && safety.brake == 1.0);

    // Freshness uses both source and receipt time and the existing clock tolerance.
    assert(speed.valueOr(-0.22, 100.25, timeout, tolerance) == 0.0);
    assert(speed.valueOr(-0.22, 100.251, timeout, tolerance) == -0.22);
    assert(speed.valueOr(-0.22, 99.99, timeout, tolerance) == -0.22);
    assert(speed.valueOr(-0.22, 0.0, timeout, tolerance) == -0.22);
    receive(speed, 0.0, 100.0, 100.24);  // repeat receipt cannot refresh old source data
    assert(speed.valueOr(-0.22, 100.26, timeout, tolerance) == -0.22);
    assert(command(speed, 100.26, 11.0, -0.22, 0.22).brake == 1.0);

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (double vx : {nan, inf, -inf}) {
        receive(speed);
        receive(speed, vx);
        assert(speed.valueOr(-0.22, 100.02, timeout, tolerance) == -0.22);
    }
    for (double stamp_bad : {0.0, -1.0, nan, inf, 100.1, 99.7}) {
        receive(speed);
        receive(speed, 0.0, stamp_bad);
        assert(speed.valueOr(-0.22, 100.02, timeout, tolerance) == -0.22);
        assert(speed.valueOr(-0.22, 100.1, timeout, tolerance) == -0.22);
    }
    for (const char* frame : {"", "map", "ego_vehicle"}) {
        receive(speed);
        receive(speed, 0.0, 100.0, 100.01, frame);
        assert(speed.valueOr(-0.22, 100.02, timeout, tolerance) == -0.22);
    }
    receive(speed, 0.0, 100.0105, 100.01);  // sub-tolerance source/receipt skew
    assert(speed.valueOr(-0.22, 100.011, timeout, tolerance) == 0.0);
    receive(speed, 0.0, 100.0, nan);
    assert(speed.valueOr(-0.22, 100.02, timeout, tolerance) == -0.22);

    receive(speed);
    speed.resetEpoch();
    assert(speed.valueOr(-0.22, 50.02, timeout, tolerance) == -0.22);
    receive(speed, 0.0, 50.0, 50.01);  // first queued packet after epoch reset discarded
    assert(speed.valueOr(-0.22, 50.02, timeout, tolerance) == -0.22);
    receive(speed, 0.0, 50.01, 50.02);
    assert(speed.valueOr(-0.22, 50.03, timeout, tolerance) == 0.0);

    // Velocity-command mode keeps its existing behavior.
    const Cmd velocity = command(speed, 50.03, 11.0, -0.22, 0.22, 1, false, false, false, 2);
    assert(velocity.velocity == 11.0 && velocity.accel == 0.0 && velocity.brake == 0.0);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / "test.cpp"
            executable = Path(directory) / "test"
            cpp.write_text(harness)
            subprocess.run([
                "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-O2",
                "-I", str(MPC / "include"), str(cpp),
                str(MPC / "src/command_boundary.cpp"), "-o", str(executable),
            ], check=True, capture_output=True, text=True)
            subprocess.run([str(executable)], check=True, capture_output=True, text=True)

    def test_generic_launch_default_and_competition_opt_in(self):
        launch = ET.parse(MPC / "launch/hdmap_path_follower.launch").getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in launch.findall("arg")}
        self.assertEqual(args["signed_speed_topic"], "")
        self.assertEqual(args["signed_speed_timeout_s"], "0.25")
        params = {param.attrib["name"]: param.attrib.get("value")
                  for param in launch.findall("node/param")}
        self.assertEqual(params["signed_speed_topic"], "$(arg signed_speed_topic)")
        competition = ET.parse(ROOT / "ros/follower_student_competition.launch").getroot()
        included = {arg.attrib["name"]: arg.attrib.get("value")
                    for arg in competition.findall("include/arg")}
        self.assertEqual(included["signed_speed_topic"], "/Competition_topic")


if __name__ == "__main__":
    unittest.main()
