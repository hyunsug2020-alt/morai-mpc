"""The native controller brakes for each bend at its actual remaining distance."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class GeneralCurveProfileTests(unittest.TestCase):
    def test_cpp_distant_and_current_bends_keep_independent_braking_caps(self):
        root = Path(__file__).resolve().parent
        core = root.parents[1] / 'autodrive/ws/src/moraimpc'
        with tempfile.TemporaryDirectory(prefix='morai-road-curve-') as work:
            binary = Path(work) / 'test'
            sources = [core/'src'/name for name in ('controller_consistency.cpp',
                'ltv_model.cpp', 'ltv_cost.cpp', 'ltv_solver.cpp')]
            subprocess.run(['g++', '-std=c++17', '-O0', '-I/usr/include/eigen3',
                '-I/usr/local/include/osqp', '-I'+str(core/'include'),
                str(root/'test_data/general_curve_profile.cpp'), *map(str, sources),
                '-L/usr/local/lib', '-losqp', '-ljsoncpp', '-o', str(binary)],
                check=True, capture_output=True, text=True, timeout=120)
            subprocess.run([str(binary)], check=True, capture_output=True,
                           text=True, timeout=10)


if __name__ == '__main__':
    unittest.main()
