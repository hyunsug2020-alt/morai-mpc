#!/home/david/miniconda3/envs/lane_tool/bin/python
"""Keep the competition interpreter while selecting this copy's perception."""
import os
from pathlib import Path
import sys


COPY_ROOT = Path(__file__).resolve().parent
ORIGINAL_FUSION = COPY_ROOT.parent / 'semantic_fusion.py'
COPY_FUSION = COPY_ROOT / 'semantic_fusion.py'
ORIGINAL_VEHICLES = COPY_ROOT.parent / 'lidar_vehicle_detection.py'
COPY_VEHICLES = COPY_ROOT / 'lidar_vehicle_detection.py'


def redirected_arguments(arguments, *, cwd=None):
    """Find the Python script operand; preserve code, modules and all arguments."""
    result = list(arguments)
    directory = Path.cwd() if cwd is None else Path(cwd)
    index = 0
    while index < len(result):
        arg = result[index]
        if arg == '--':
            index += 1
            break
        if arg == '-':
            return result
        if arg == '--check-hash-based-pycs':
            index += 2
            continue
        if arg.startswith('--check-hash-based-pycs='):
            index += 1
            continue
        if arg.startswith('--'):
            return result                    # help/version or an invalid option
        if arg.startswith('-'):
            # CPython permits clusters such as -uBc and -uW ignore. c/m stop
            # parsing at their code/module operand; W/X consume a value.
            consumed = 1
            for offset, option in enumerate(arg[1:], 1):
                if option in 'cmh?V':
                    return result
                if option in 'WX':
                    consumed = 2 if offset == len(arg)-1 else 1
                    break
            index += consumed
            continue
        break
    if index < len(result):
        script = Path(result[index])
        if not script.is_absolute():
            script = directory / script
        # Normalize relative components without accepting unrelated symlink
        # aliases or another file that merely has the same basename.
        replacements = {str(ORIGINAL_FUSION): COPY_FUSION, str(ORIGINAL_VEHICLES): COPY_VEHICLES}
        replacement = replacements.get(os.path.abspath(script))
        if replacement is not None:
            result[index] = str(replacement)
    return result


def main(arguments=None):
    args = sys.argv[1:] if arguments is None else arguments
    os.execv(sys.executable, [sys.executable, *redirected_arguments(args)])


if __name__ == '__main__':
    main()
