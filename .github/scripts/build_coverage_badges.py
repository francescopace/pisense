#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Build Badgen endpoint documents from CI coverage reports."""

from __future__ import annotations

import argparse
import json
import os
import re
from pathlib import Path


METRIC_ORDER = ("lines", "branches", "functions")
BADGE_LABELS = {
    "cpp-runtime": "c++ coverage",
    "python": "python coverage",
    "web": "web coverage",
}
WEB_SUMMARY_PATTERN = re.compile(
    r"(?m)^# all files\s*\|\s*(?P<lines>[0-9.]+)\s*"
    r"\|\s*(?P<branches>[0-9.]+)\s*"
    r"\|\s*(?P<functions>[0-9.]+)\s*\|"
)


def _percentage(covered: int, total: int) -> float:
    return 100.0 if total == 0 else covered * 100.0 / total


def _load_metrics(kind: str, report_path: Path) -> dict[str, float]:
    if kind == "web":
        match = WEB_SUMMARY_PATTERN.search(report_path.read_text(encoding="utf-8"))
        if match is None:
            raise ValueError("web coverage summary was not found")
        return {metric: float(match.group(metric)) for metric in METRIC_ORDER}

    report = json.loads(report_path.read_text(encoding="utf-8"))
    if kind == "cpp-runtime":
        return {
            metric: float(value)
            for metric, value in report["segments"]["runtime"].items()
        }

    totals = report["totals"]
    return {
        "lines": _percentage(totals["covered_lines"], totals["num_statements"]),
        "branches": _percentage(
            totals["covered_branches"], totals["num_branches"]
        ),
    }


def _load_minimums(kind: str, thresholds_path: Path) -> dict[str, float]:
    policy = json.loads(thresholds_path.read_text(encoding="utf-8"))
    if policy.get("version") != 1:
        raise ValueError("unsupported coverage threshold schema")
    minimums = (
        policy["segments"]["runtime"]
        if kind == "cpp-runtime"
        else policy["minimums"]
    )
    return {metric: float(value) for metric, value in minimums.items()}


def build_badge(kind: str, report_path: Path, thresholds_path: Path) -> dict:
    """Return one Badgen HTTPS endpoint document."""
    metrics = _load_metrics(kind, report_path)
    minimums = _load_minimums(kind, thresholds_path)
    if set(metrics) != set(minimums):
        raise ValueError("coverage metrics do not match configured thresholds")
    passed = all(
        metrics[metric] + 1e-9 >= minimum for metric, minimum in minimums.items()
    )
    return {
        "subject": BADGE_LABELS[kind],
        "status": f"{metrics['lines']:.2f}%",
        "color": "4c1" if passed else "e05d44",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--kind", choices=tuple(BADGE_LABELS), required=True
    )
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--thresholds", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(
            build_badge(args.kind, args.report, args.thresholds),
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    if summary_path := os.environ.get("GITHUB_STEP_SUMMARY"):
        metrics = _load_metrics(args.kind, args.report)
        minimums = _load_minimums(args.kind, args.thresholds)
        with Path(summary_path).open("a", encoding="utf-8") as summary:
            summary.write(f"### {BADGE_LABELS[args.kind]}\n\n")
            summary.write("| Metric | Coverage | Minimum | Result |\n| --- | ---: | ---: | --- |\n")
            for metric in METRIC_ORDER:
                if metric in metrics:
                    passed = metrics[metric] + 1e-9 >= minimums[metric]
                    summary.write(f"| {metric} | {metrics[metric]:.2f}% | {minimums[metric]:.2f}% | "
                                  f"{'Pass' if passed else 'Below threshold'} |\n")
            summary.write("\nUses the existing coverage scope and thresholds; no branch comparison.\n")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
