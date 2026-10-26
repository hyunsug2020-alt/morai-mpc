#!/usr/bin/env bash
set -euo pipefail
COPY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$COPY_ROOT/../../.venv-openpcdet/bin/python" "$COPY_ROOT/competition_python.py" "$@"
