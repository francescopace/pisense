#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.

set -euo pipefail

read -r line_threshold branch_threshold function_threshold < <(
  node -e '
    const policy = require("./test/web/coverage-thresholds.json").minimums;
    console.log(policy.lines, policy.branches, policy.functions);
  '
)

coverage_command=(
  node
  --experimental-test-coverage
  --test
  --test-coverage-include=docs/web/assets/js/espectre-direct.js
  "--test-coverage-lines=${line_threshold}"
  "--test-coverage-branches=${branch_threshold}"
  "--test-coverage-functions=${function_threshold}"
  test/web/*.mjs
)

if [[ -z "${WEB_COVERAGE_LOG:-}" ]]; then
  exec "${coverage_command[@]}"
fi

mkdir -p "$(dirname "$WEB_COVERAGE_LOG")"
set +e
"${coverage_command[@]}" 2>&1 | tee "$WEB_COVERAGE_LOG"
coverage_status=${PIPESTATUS[0]}
set -e
exit "$coverage_status"
