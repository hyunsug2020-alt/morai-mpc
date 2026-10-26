"""Exercise the deployed C++ ring profile's binding and braking-distance limit."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class JunctionCurveProfileTests(unittest.TestCase):
    def test_cpp_fresh_bound_profile_and_braking_envelope(self):
        core = Path(__file__).resolve().parents[2] / 'autodrive/ws/src/moraimpc'
        with tempfile.TemporaryDirectory(prefix='morai-ring-curve-') as work:
            source, binary = Path(work)/'test.cpp', Path(work)/'test'
            source.write_text(r'''
#include <cassert>
#include <limits>
#include "moraimpc/junction_curve.hpp"
using namespace moraimpc;
int main() {
    Json::Value p;
    p["plan_id"] = 1; p["source_stamp"] = 99.9;
    auto& d = p["decision"];
    d["plan_id"] = 1; d["source_stamp"] = 99.9; d["emergency"] = false;
    d["path_digest"] = "published-path";
    auto& v = d["validation"];
    v["valid"] = v["pose_valid"] = v["steering_valid"] = true;
    v["geometry_path_digest"] = v["traffic_path_digest"] = "published-path";
    v["junction_curve_profile"]["kind"] = "committed_ring";
    v["junction_curve_profile"]["expires_at"] = 100.2;
    v["junction_curve_profile"]["target_mps"] = 25./3.6;
    assert(junctionCurveDeadline(p, 100.) == 100.2);
    assert(junctionCurveDeadline(p, 100.2) == 0.);
    assert(junctionCurveDeadline(p, 99.) == 0.);
    auto bad=p; bad["decision"]["emergency"]=true;
    assert(junctionCurveDeadline(bad,100.)==0.);
    bad=p; bad["plan_id"]=2;
    assert(junctionCurveDeadline(bad,100.)==0.);
    bad=p; bad["source_stamp"]=99.8;
    assert(junctionCurveDeadline(bad,100.)==0.);
    bad=p; bad["decision"]["validation"]["traffic_path_digest"]="old-path";
    assert(junctionCurveDeadline(bad,100.)==0.);
    bad=p; bad["decision"]["validation"].removeMember("junction_curve_profile");
    assert(junctionCurveDeadline(bad,100.)==0.);
    const double top=25./3.6, brake=2., k=.1057, alat=4.2;
    const double bend=std::sqrt(alat/k);
    assert(std::abs(junctionCurveApproachSpeed(bend,30.,top,brake)-top)<1e-12);
    assert(std::abs(junctionCurveApproachSpeed(bend,0.,top,brake)-bend)<1e-12);
    double previous=bend;
    for(double distance=0.;distance<=30.;distance+=.01) {
        const double cap=junctionCurveApproachSpeed(bend,distance,top,brake);
        assert(cap>=previous-1e-12 && cap<=top);
        const double room=std::max(0.,distance-.5*top);
        assert((cap*cap-bend*bend)/(2.*brake)<=room+1e-9);
        previous=cap;
    }
    assert(junctionCurveApproachSpeed(bend,10.,top,0.)==0.);
    assert(junctionCurveApproachSpeed(bend,std::numeric_limits<double>::quiet_NaN(),top,brake)==0.);
}
''')
            subprocess.run(['g++', '-std=c++17', '-O0', '-I'+str(core/'include'),
                            str(source), '-ljsoncpp', '-o', str(binary)],
                           check=True, capture_output=True, text=True, timeout=30)
            subprocess.run([str(binary)], check=True, capture_output=True, text=True, timeout=5)
