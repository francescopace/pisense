#!/bin/bash
#
# ESPectre - Code Coverage Script
#
# Runs the host-side CMake/CTest suite with coverage instrumentation and
# prints aggregated coverage, including per-segment stats.
#
# Usage:
#   ./run_coverage.sh           # Local run (prints summary)
#   ./run_coverage.sh --ci      # CI run (writes coverage artifacts)
#   CTEST_PARALLEL_LEVEL=2 ./run_coverage.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="$SCRIPT_DIR/build-coverage"
REPORT_DIR="$WORKSPACE_ROOT/.cache/reports/coverage/cpp"
LCOV_OUTPUT="$REPORT_DIR/coverage.lcov"
TMP_LCOV="$REPORT_DIR/.coverage.tmp.lcov"
SUMMARY_OUTPUT="$REPORT_DIR/coverage-summary.json"
THRESHOLDS_FILE="$SCRIPT_DIR/coverage-thresholds.json"

CI_MODE=false
case "${1:-}" in
    "") ;;
    --ci) CI_MODE=true ;;
    *) echo "error: expected --ci" >&2; exit 2 ;;
esac

detect_parallel_jobs() {
    local jobs="${CTEST_PARALLEL_LEVEL:-}"
    if [[ -n "$jobs" ]] && [[ ! "$jobs" =~ ^[1-9][0-9]*$ ]]; then
        echo "error: CTEST_PARALLEL_LEVEL must be a positive integer" >&2
        return 1
    fi
    if [[ -z "$jobs" ]] && command -v getconf >/dev/null 2>&1; then
        jobs="$(getconf _NPROCESSORS_ONLN 2>/dev/null || true)"
    fi
    if [[ -z "$jobs" ]] && command -v nproc >/dev/null 2>&1; then
        jobs="$(nproc 2>/dev/null || true)"
    fi
    if [[ -z "$jobs" ]] && command -v sysctl >/dev/null 2>&1; then
        jobs="$(sysctl -n hw.logicalcpu 2>/dev/null || true)"
    fi
    if [[ ! "$jobs" =~ ^[1-9][0-9]*$ ]]; then
        jobs=1
    fi
    printf '%s\n' "$jobs"
}

PARALLEL_JOBS="$(detect_parallel_jobs)"

detect_compiler() {
    if "${CXX:-c++}" --version 2>/dev/null | grep -qi clang; then
        echo "clang"
    else
        echo "gcc"
    fi
}

summarize_lcov() {
    python3 - "$1" "$WORKSPACE_ROOT" "$SUMMARY_OUTPUT" "$THRESHOLDS_FILE" "$CI_MODE" <<'PY'
import collections
import json
import os
import sys

lcov_path = sys.argv[1]
workspace_root = os.path.realpath(sys.argv[2])
summary_path = sys.argv[3]
thresholds_path = sys.argv[4]
ci_mode = sys.argv[5] == "true"

files = {}
current = None

def ensure_file(path):
    return files.setdefault(path, {
        "lines": {},
        "functions": {},
        "branches": {},
    })

with open(lcov_path, "r", encoding="utf-8") as handle:
    for raw_line in handle:
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("SF:"):
            current = os.path.realpath(line[3:])
            ensure_file(current)
        elif line == "end_of_record":
            current = None
        elif current is None:
            continue
        elif line.startswith("DA:"):
            # LCOV allows an optional checksum: DA:<line>,<hits>[,<checksum>]
            parts = line[3:].split(",")
            line_no = int(parts[0])
            hits = int(parts[1])
            ensure_file(current)["lines"][line_no] = max(ensure_file(current)["lines"].get(line_no, 0), hits)
        elif line.startswith("FNDA:"):
            hits, name = line[5:].split(",", 1)
            hits = int(hits)
            ensure_file(current)["functions"][name] = max(ensure_file(current)["functions"].get(name, 0), hits)
        elif line.startswith("BRDA:"):
            line_no, block_no, branch_no, hits = line[5:].split(",", 3)
            key = (int(line_no), block_no, branch_no)
            hit_value = 0 if hits == "-" else int(hits)
            ensure_file(current)["branches"][key] = max(ensure_file(current)["branches"].get(key, 0), hit_value)

def classify(path):
    rel = os.path.relpath(os.path.realpath(path), workspace_root)
    if rel.startswith("src/cpp/core/"):
        return "core"
    if rel.startswith("src/cpp/runtime/"):
        return "runtime"
    if rel.startswith("src/cpp/frontend/"):
        return "frontend"
    return "other"

def collect_totals(selected_files):
    totals = {
        "lines_total": 0,
        "lines_hit": 0,
        "func_total": 0,
        "func_hit": 0,
        "branch_total": 0,
        "branch_hit": 0,
    }
    for stats in selected_files:
        totals["lines_total"] += len(stats["lines"])
        totals["lines_hit"] += sum(1 for hits in stats["lines"].values() if hits > 0)
        totals["func_total"] += len(stats["functions"])
        totals["func_hit"] += sum(1 for hits in stats["functions"].values() if hits > 0)
        totals["branch_total"] += len(stats["branches"])
        totals["branch_hit"] += sum(1 for hits in stats["branches"].values() if hits > 0)
    return totals

def pct(hit, total):
    return 100.0 if total == 0 else (hit * 100.0) / total

all_stats = list(files.values())
overall = collect_totals(all_stats)

print("Coverage summary:")
print(f"  Lines:     {overall['lines_hit']}/{overall['lines_total']} ({pct(overall['lines_hit'], overall['lines_total']):.2f}%)")
print(f"  Functions: {overall['func_hit']}/{overall['func_total']} ({pct(overall['func_hit'], overall['func_total']):.2f}%)")
print(f"  Branches:  {overall['branch_hit']}/{overall['branch_total']} ({pct(overall['branch_hit'], overall['branch_total']):.2f}%)")

segments = collections.defaultdict(list)
for path, stats in files.items():
    segments[classify(path)].append(stats)

print("Coverage by segment:")
summary = {}
summary["overall"] = {
    "lines": round(pct(overall["lines_hit"], overall["lines_total"]), 2),
    "functions": round(pct(overall["func_hit"], overall["func_total"]), 2),
    "branches": round(pct(overall["branch_hit"], overall["branch_total"]), 2),
}
for segment in ("core", "runtime", "frontend", "other"):
    if not segments[segment]:
        continue
    totals = collect_totals(segments[segment])
    print(
        f"  {segment}: "
        f"lines {pct(totals['lines_hit'], totals['lines_total']):.2f}% | "
        f"functions {pct(totals['func_hit'], totals['func_total']):.2f}% | "
        f"branches {pct(totals['branch_hit'], totals['branch_total']):.2f}%"
    )
    summary[segment] = {
        "lines": round(pct(totals["lines_hit"], totals["lines_total"]), 2),
        "functions": round(pct(totals["func_hit"], totals["func_total"]), 2),
        "branches": round(pct(totals["branch_hit"], totals["branch_total"]), 2),
    }

with open(summary_path, "w", encoding="utf-8") as handle:
    json.dump({"version": 1, "segments": summary}, handle, indent=2, sort_keys=True)
    handle.write("\n")

if ci_mode:
    with open(thresholds_path, "r", encoding="utf-8") as handle:
        thresholds = json.load(handle)
    if thresholds.get("version") != 1:
        raise SystemExit("Unsupported C++ coverage threshold version")
    failures = []
    for segment, metrics in thresholds["segments"].items():
        for metric, minimum in metrics.items():
            actual = summary.get(segment, {}).get(metric, 0.0)
            if actual < float(minimum):
                failures.append(
                    f"{segment}.{metric}: {actual:.2f}% < {float(minimum):.2f}%"
                )
    if failures:
        raise SystemExit("C++ coverage threshold not met:\n  " + "\n  ".join(failures))
    print("Coverage thresholds satisfied")
PY
}

run_clang_coverage() {
    python3 - "$BUILD_DIR" "$WORKSPACE_ROOT" "$TMP_LCOV" "$CI_MODE" \
        "$PARALLEL_JOBS" <<'PY'
import concurrent.futures
import json
import os
import re
import subprocess
import sys

build_dir = os.path.abspath(sys.argv[1])
workspace_root = os.path.realpath(sys.argv[2])
lcov_output = os.path.abspath(sys.argv[3])
ci_mode = sys.argv[4] == "true"
parallel_jobs = max(1, int(sys.argv[5]))

profiles_dir = os.path.join(build_dir, "profiles")
os.makedirs(profiles_dir, exist_ok=True)

ctest_data = json.loads(
    subprocess.check_output(
        ["ctest", "--test-dir", build_dir, "--show-only=json-v1"],
        text=True,
    )
)

if sys.platform == "darwin":
    llvm_profdata = ["xcrun", "llvm-profdata"]
    llvm_cov = ["xcrun", "llvm-cov"]
else:
    llvm_profdata = ["llvm-profdata"]
    llvm_cov = ["llvm-cov"]

def sanitize(name):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name)

def get_property(test, key, default=None):
    for prop in test.get("properties", []):
        if prop.get("name") == key:
            return prop.get("value", default)
    return default

def parse_lcov(text):
    files = {}
    current = None
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("SF:"):
            current = os.path.realpath(line[3:])
            files.setdefault(current, {"fn": {}, "da": {}, "br": {}})
        elif line == "end_of_record":
            current = None
        elif current is None:
            continue
        elif line.startswith("FN:"):
            line_no, name = line[3:].split(",", 1)
            files[current]["fn"].setdefault(name, {"line": int(line_no), "hits": 0})
        elif line.startswith("FNDA:"):
            hits, name = line[5:].split(",", 1)
            entry = files[current]["fn"].setdefault(name, {"line": 0, "hits": 0})
            entry["hits"] = max(entry["hits"], int(hits))
        elif line.startswith("DA:"):
            # LCOV allows an optional checksum: DA:<line>,<hits>[,<checksum>]
            parts = line[3:].split(",")
            line_no = int(parts[0])
            files[current]["da"][line_no] = max(files[current]["da"].get(line_no, 0), int(parts[1]))
        elif line.startswith("BRDA:"):
            line_no, block_no, branch_no, hits = line[5:].split(",", 3)
            key = (int(line_no), block_no, branch_no)
            hit_value = 0 if hits == "-" else int(hits)
            files[current]["br"][key] = max(files[current]["br"].get(key, 0), hit_value)
    return files

def merge_records(dest, src):
    for path, stats in src.items():
        entry = dest.setdefault(path, {"fn": {}, "da": {}, "br": {}})
        for name, fn_data in stats["fn"].items():
            target = entry["fn"].setdefault(name, {"line": fn_data["line"], "hits": 0})
            if target["line"] == 0:
                target["line"] = fn_data["line"]
            target["hits"] = max(target["hits"], fn_data["hits"])
        for line_no, hits in stats["da"].items():
            entry["da"][line_no] = max(entry["da"].get(line_no, 0), hits)
        for key, hits in stats["br"].items():
            entry["br"][key] = max(entry["br"].get(key, 0), hits)

def write_lcov(path, records):
    with open(path, "w", encoding="utf-8") as handle:
        for file_path in sorted(records):
            stats = records[file_path]
            handle.write(f"SF:{file_path}\n")
            for name, fn_data in sorted(stats["fn"].items(), key=lambda item: (item[1]["line"], item[0])):
                handle.write(f"FN:{fn_data['line']},{name}\n")
            for name, fn_data in sorted(stats["fn"].items()):
                handle.write(f"FNDA:{fn_data['hits']},{name}\n")
            for line_no, hits in sorted(stats["da"].items()):
                handle.write(f"DA:{line_no},{hits}\n")
            for (line_no, block_no, branch_no), hits in sorted(stats["br"].items()):
                handle.write(f"BRDA:{line_no},{block_no},{branch_no},{hits}\n")
            handle.write("end_of_record\n")

def run_test(test):
    name = test["name"]
    command = list(test.get("command", []))
    if not command:
        return None
    cwd = get_property(test, "WORKING_DIRECTORY", build_dir)
    executable = command[0]
    if not os.path.isabs(executable):
        executable = os.path.normpath(os.path.join(cwd, executable))
        command[0] = executable

    profile_base = os.path.join(profiles_dir, sanitize(name))
    profraw = profile_base + ".profraw"
    profdata = profile_base + ".profdata"

    env = os.environ.copy()
    env["LLVM_PROFILE_FILE"] = profraw

    result = subprocess.run(command, cwd=cwd, env=env, capture_output=True, text=True)
    skip_return_code = get_property(test, "SKIP_RETURN_CODE")
    return name, executable, profraw, profdata, skip_return_code, result

combined = {}
overall_status = 0
tests = list(ctest_data.get("tests", []))

with concurrent.futures.ThreadPoolExecutor(max_workers=parallel_jobs) as executor:
    for item in executor.map(run_test, tests):
        if item is None:
            continue
        name, executable, profraw, profdata, skip_return_code, result = item
        print(f"Completed {name} ...")
        if result.stdout:
            print(result.stdout, end="")
        skipped = skip_return_code is not None and result.returncode == int(skip_return_code)
        if result.returncode != 0 and not skipped:
            overall_status = result.returncode
            if result.stderr:
                print(result.stderr, end="", file=sys.stderr)
            print(f"[FAIL] {name}", file=sys.stderr)
        elif skipped:
            print(f"[SKIP] {name}")
        else:
            print(f"[PASS] {name}")

        if not os.path.exists(profraw):
            continue

        subprocess.check_call(
            [*llvm_profdata, "merge", "-sparse", profraw, "-o", profdata]
        )
        lcov_text = subprocess.check_output(
            [
                *llvm_cov,
                "export",
                executable,
                "-instr-profile=" + profdata,
                "-format=lcov",
                workspace_root + "/src",
            ],
            text=True,
        )
        merge_records(combined, parse_lcov(lcov_text))

write_lcov(lcov_output, combined)
sys.exit(overall_status)
PY
}

COMPILER="$(detect_compiler)"
echo "ESPectre host-side coverage"
echo "Compiler: $COMPILER"
echo "Parallel jobs: $PARALLEL_JOBS"

rm -rf "$BUILD_DIR"
mkdir -p "$REPORT_DIR"
rm -f "$LCOV_OUTPUT" "$TMP_LCOV" "$SUMMARY_OUTPUT"

cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DESPECTRE_ENABLE_COVERAGE=ON
cmake --build "$BUILD_DIR" --parallel "$PARALLEL_JOBS"

TEST_RESULT=0
if [[ "$COMPILER" == "clang" ]]; then
    run_clang_coverage || TEST_RESULT=$?
    mv "$TMP_LCOV" "$LCOV_OUTPUT"
    summarize_lcov "$LCOV_OUTPUT"
else
    ctest --test-dir "$BUILD_DIR" --parallel "$PARALLEL_JOBS" \
        --output-on-failure || TEST_RESULT=$?

    if ! command -v gcovr >/dev/null 2>&1; then
        echo "Error: gcovr is required for GCC coverage"
        exit 1
    fi

    gcovr --root "$WORKSPACE_ROOT" \
          --filter "$WORKSPACE_ROOT/src/.*" \
          --exclude '.*test.*' \
          --exclude-noncode-lines \
          --gcov-ignore-parse-errors=all \
          --print-summary \
          --lcov "$LCOV_OUTPUT" \
          "$BUILD_DIR"
    summarize_lcov "$LCOV_OUTPUT"

fi

if [[ "$CI_MODE" != true ]]; then
    rm -f "$LCOV_OUTPUT" "$SUMMARY_OUTPUT"
fi

if [[ $TEST_RESULT -ne 0 ]]; then
    exit "$TEST_RESULT"
fi

echo "Coverage complete"
