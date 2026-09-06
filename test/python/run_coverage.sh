#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
# Run the full Python suite and enforce the project coverage gates.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$WORKSPACE_ROOT"

PYTHON=python3
if [[ -x "$WORKSPACE_ROOT/.venv/bin/python" ]]; then
    PYTHON="$WORKSPACE_ROOT/.venv/bin/python"
elif [[ -x "$WORKSPACE_ROOT/.venv/Scripts/python.exe" ]]; then
    PYTHON="$WORKSPACE_ROOT/.venv/Scripts/python.exe"
fi

REPORT=".cache/reports/coverage/python-coverage.json"
mkdir -p "$(dirname "$REPORT")"

"$PYTHON" -m pytest test/python -n auto --dist worksteal -q --tb=short \
    --cov=src/python/micro_espectre \
    --cov=src/python/espectre_cli \
    --cov-branch \
    --cov-report="json:$REPORT"

exec "$PYTHON" test/python/check_coverage.py "$REPORT"
