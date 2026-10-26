#!/usr/bin/env bash
# Stop only the launch whose recorded owner is this bundle's follower script.
set -euo pipefail
source "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/common.sh"
python3 - "$BUNDLE_ROOT" <<'PY'
import json, os, signal, sys
from pathlib import Path
root=Path(sys.argv[1])
try:
    meta=json.loads((root/'data/autodrive/drive_run.json').read_text())
    pid=int(meta['owner_pid'])
    cmd=(Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0')
    expected=os.fsencode(str(root/'autodrive/student_v3_follow_claude.sh'))
    if expected not in cmd:
        raise SystemExit('Recorded process is not this bundle\'s launch; nothing stopped')
    os.kill(pid,signal.SIGINT)
    print('Sent SIGINT to the owned launch:',pid)
except (FileNotFoundError, ProcessLookupError):
    print('No active owned launch')
PY
