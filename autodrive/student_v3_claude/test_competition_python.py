"""Interpreter routing must not rewrite Python code or unrelated scripts."""
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import competition_python as wrapper


class CompetitionPythonTests(unittest.TestCase):
    def test_exact_original_script_is_the_only_redirected_operand(self):
        original = str(wrapper.ORIGINAL_FUSION)
        args = [original, '--sensor', 'preset.json', '--route', original]
        result = wrapper.redirected_arguments(args)
        self.assertEqual(result, [str(wrapper.COPY_FUSION), *args[1:]])
        self.assertEqual(args[0], original)

    def test_relative_original_script_and_double_dash_are_supported(self):
        root = wrapper.COPY_ROOT.parent.parent
        self.assertEqual(wrapper.redirected_arguments(
            ['--', 'autodrive/semantic_fusion.py', '--hz', '15'], cwd=root),
            ['--', str(wrapper.COPY_FUSION), '--hz', '15'])

    def test_vehicle_detector_uses_the_copy_with_original_arguments(self):
        original = str(wrapper.ORIGINAL_VEHICLES)
        args = ['-u', original, '--route', 'global_path.json', '--model-dir', 'models']
        self.assertEqual(wrapper.redirected_arguments(args),
                         ['-u', str(wrapper.COPY_VEHICLES), *args[2:]])
        root = wrapper.COPY_ROOT.parent.parent
        self.assertEqual(wrapper.redirected_arguments(
            ['autodrive/lidar_vehicle_detection.py'], cwd=root), [str(wrapper.COPY_VEHICLES)])
        for other in (['/tmp/lidar_vehicle_detection.py'], ['-c', original],
                      ['/tmp/other.py', original], [original+'.bak']):
            self.assertEqual(wrapper.redirected_arguments(other), other)

    def test_modules_code_stdin_and_attached_operands_are_unchanged(self):
        original = str(wrapper.ORIGINAL_FUSION)
        for args in (['-c', original], ['-cprint(1)', original],
                     ['-m', original], ['-msite', original], ['-', original],
                     ['-uBc', original], ['-uvm', original], ['--help', original]):
            with self.subTest(args=args):
                self.assertEqual(wrapper.redirected_arguments(args), args)

    def test_options_and_their_values_are_preserved(self):
        original = str(wrapper.ORIGINAL_FUSION)
        for options in (['-u', '-B'], ['-W', 'ignore'], ['-X', 'faulthandler'],
                        ['-uW', 'ignore'], ['-BWignore'], ['-Xdev'],
                        ['--check-hash-based-pycs', 'always']):
            with self.subTest(options=options):
                args = [*options, original, 'extra']
                self.assertEqual(wrapper.redirected_arguments(args),
                                 [*options, str(wrapper.COPY_FUSION), 'extra'])

    def test_warning_and_xoption_values_are_not_scripts(self):
        original = str(wrapper.ORIGINAL_FUSION)
        for options in (['-W'], ['-X'], ['-uW'], ['--check-hash-based-pycs']):
            args = [*options, original, '/tmp/other.py']
            self.assertEqual(wrapper.redirected_arguments(args), args)

    def test_same_basename_fake_paths_and_script_arguments_are_unchanged(self):
        original = str(wrapper.ORIGINAL_FUSION)
        for args in (['/tmp/semantic_fusion.py'], [original+'.bak'],
                     ['/tmp/other.py', original], ['-c', 'print(1)', original],
                     [str(wrapper.COPY_FUSION)], []):
            with self.subTest(args=args):
                self.assertEqual(wrapper.redirected_arguments(args), args)

    def test_relative_basename_in_another_directory_is_unchanged(self):
        self.assertEqual(wrapper.redirected_arguments(['semantic_fusion.py'], cwd='/tmp'),
                         ['semantic_fusion.py'])

    def test_exec_keeps_the_interpreter_and_process_identity(self):
        args = ['-u', str(wrapper.ORIGINAL_FUSION), '--hz', '15']
        with patch.object(os, 'execv') as execute:
            wrapper.main(args)
        execute.assert_called_once_with(sys.executable,
            [sys.executable, '-u', str(wrapper.COPY_FUSION), '--hz', '15'])

    def test_copy_launch_selects_wrapper_and_forwards_all_arguments(self):
        script = (Path(__file__).parent/'launch.sh').read_text()
        self.assertIn('export STUDENT_PYTHON="$COPY_ROOT/competition_python.py"', script)
        self.assertIn('export STUDENT_VEHICLE_PYTHON="$COPY_ROOT/competition_vehicle_python.sh"', script)
        self.assertIn('exec bash "$COPY_ROOT/../student_v3_follow_claude.sh" "$@"', script)
        self.assertEqual(wrapper.COPY_FUSION.parent, Path(__file__).resolve().parent)

    def test_vehicle_wrapper_preserves_the_openpcdet_interpreter(self):
        import subprocess
        script = Path(__file__).parent/'competition_vehicle_python.sh'
        result = subprocess.run([str(script), '-c',
            'import sys; print(sys.prefix)'], check=True, capture_output=True, text=True)
        self.assertEqual(Path(result.stdout.strip()), wrapper.COPY_ROOT.parent.parent/'.venv-openpcdet')


if __name__ == '__main__':
    unittest.main()
