#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Occupancy-floor sweep

Host-only experiment: keep the production 100 pps slot grid and ask how detector
quality changes when admitted CSI is thinned. Each point thins admitted packets
to the requested occupancy. By default the readiness floor matches that
occupancy. Pass ``--always-evaluate`` to pin the floor at one valid slot so
occupancy holes stay scored instead of dropping not-ready ticks.

Usage:
    .venv/bin/python tools/sweep_occupancy_floor.py
    .venv/bin/python tools/sweep_occupancy_floor.py --floors 70,65,60,55,50
    .venv/bin/python tools/sweep_occupancy_floor.py --floors 70,60,50,40,30 --always-evaluate
"""

from __future__ import annotations

import argparse
import sys
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterable, Iterator, Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.lib.bootstrap import setup_paths  # noqa: E402

setup_paths()

import tools.lib.dataset_metadata as dataset_metadata  # noqa: E402
import tools.lib.performance_report as performance_report  # noqa: E402
from tools.lib import temporal_csi_sampler as temporal_csi_sampler  # noqa: E402
from config import DEFAULT_SUBCARRIERS, SEGMENTATION_WINDOW_SIZE_MS  # noqa: E402
from tools.lib.temporal_csi_sampler import temporal_window_slots  # noqa: E402
from tools.lib.performance_report import (  # noqa: E402
    REPORT_DATASET_ROLES,
    _compute_ml_row_result,
    compute_classic_row_result,
    get_available_paired_datasets,
    is_low_rssi_paired_dataset,
    load_or_compute_classic_replay_rows,
    load_or_compute_ml_replay_rows,
    load_real_data_cached,
)
from tools.lib.occupancy_thinning import (  # noqa: E402
    OCCUPANCY_THIN_SEED,
    TARGET_PPS,
    admit_packets,
    capture_seed,
    mean_window_occupancy,
    thin_packets,
)

DEFAULT_FLOORS = (70, 65, 60, 55, 50)
SWEEP_SEED = OCCUPANCY_THIN_SEED
SWEEP_VERSION = 1


def _parse_floors(value: str) -> tuple[int, ...]:
    floors = []
    for part in str(value).split(","):
        token = part.strip()
        if not token:
            continue
        floor = int(token)
        if floor < 1 or floor > 100:
            raise argparse.ArgumentTypeError(
                "occupancy floors must be integers in 1-100"
            )
        floors.append(floor)
    if not floors:
        raise argparse.ArgumentTypeError("at least one occupancy floor is required")
    return tuple(floors)


@contextmanager
def occupancy_floor_slots(slots: int) -> Iterator[None]:
    """Pin the shared occupancy helper to an absolute slot count."""

    def _minimum_valid_slots(window_slots: int) -> int:
        return max(1, min(int(slots), max(1, int(window_slots))))

    modules = (
        temporal_csi_sampler,
        performance_report,
        dataset_metadata,
    )
    originals = tuple(module.minimum_valid_slots for module in modules)
    for module in modules:
        module.minimum_valid_slots = _minimum_valid_slots
    try:
        yield
    finally:
        for module, original in zip(modules, originals, strict=True):
            module.minimum_valid_slots = original


def pair_seed(dataset_id: str, occupancy_percent: int) -> int:
    return capture_seed(dataset_id, occupancy_percent)


def empty_metrics() -> dict[str, float]:
    return {
        "recall": 0.0,
        "fp_rate": 0.0,
        "f1": 0.0,
        "precision": 0.0,
        "tp": 0.0,
        "fn": 0.0,
        "fp": 0.0,
        "tn": 0.0,
        "num_baseline": 0.0,
        "num_movement": 0.0,
        "effective_alarms": 0.0,
    }


def _aggregate(rows: Sequence[dict[str, float]]) -> dict[str, float]:
    if not rows:
        return {
            **empty_metrics(),
            "pairs": 0.0,
            "min_recall": 0.0,
            "max_fp_rate": 0.0,
            "calib_fail": 0.0,
        }
    tp = sum(row["tp"] for row in rows)
    fn = sum(row["fn"] for row in rows)
    fp = sum(row["fp"] for row in rows)
    tn = sum(row["tn"] for row in rows)
    recall = tp / (tp + fn) * 100.0 if (tp + fn) else 0.0
    precision = tp / (tp + fp) * 100.0 if (tp + fp) else 0.0
    fp_rate = fp / (fp + tn) * 100.0 if (fp + tn) else 0.0
    f1 = (
        2 * (precision / 100.0) * (recall / 100.0) / ((precision + recall) / 100.0) * 100.0
        if (precision + recall)
        else 0.0
    )
    return {
        "pairs": float(len(rows)),
        "recall": recall,
        "min_recall": min(row["recall"] for row in rows),
        "fp_rate": fp_rate,
        "max_fp_rate": max(row["fp_rate"] for row in rows),
        "f1": f1,
        "precision": precision,
        "tp": float(tp),
        "fn": float(fn),
        "fp": float(fp),
        "tn": float(tn),
        "num_baseline": float(sum(row["num_baseline"] for row in rows)),
        "num_movement": float(sum(row["num_movement"] for row in rows)),
        "effective_alarms": float(sum(row.get("effective_alarms", 0.0) for row in rows)),
        "calib_fail": 0.0,
    }


def _format_detector(summary: dict[str, float], *, calib_fail: int) -> str:
    fail = f" fail={calib_fail}" if calib_fail else ""
    return (
        f"{summary['recall']:5.1f}/{summary['min_recall']:5.1f} "
        f"{summary['fp_rate']:4.1f}/{summary['max_fp_rate']:4.1f} "
        f"F1 {summary['f1']:5.1f} "
        f"eval {int(summary['num_baseline'])}/{int(summary['num_movement'])}"
        f"{fail}"
    )


def evaluate_pair(
    *,
    static_path: Path,
    motion_path: Path,
    dataset_id: str,
    occupancy_percent: int,
    target_pps: int,
    include_full: bool,
    readiness_slots: int | None,
) -> dict[str, Any]:
    static_source, motion_source = load_real_data_cached(static_path, motion_path)
    static_admitted = admit_packets(static_source, target_pps=target_pps)
    motion_admitted = admit_packets(motion_source, target_pps=target_pps)
    admitted_occupancy = mean_window_occupancy(static_admitted, target_pps=target_pps)
    target_occupancy = 1.0 if include_full else occupancy_percent / 100.0
    keep_ratio = (
        1.0
        if include_full or admitted_occupancy <= 0.0
        else min(1.0, target_occupancy / admitted_occupancy)
    )
    seed = pair_seed(dataset_id, occupancy_percent)
    static_packets = thin_packets(static_admitted, keep_ratio=keep_ratio)
    motion_packets = thin_packets(motion_admitted, keep_ratio=keep_ratio)
    window_slots = temporal_window_slots(target_pps, SEGMENTATION_WINDOW_SIZE_MS)
    floor_slots = (
        occupancy_percent
        if readiness_slots is None
        else max(1, min(int(readiness_slots), int(window_slots)))
    )
    provenance = {
        "transform": "occupancy_floor_sweep",
        "transform_version": SWEEP_VERSION,
        "target_pps": int(target_pps),
        "occupancy_percent": int(occupancy_percent),
        "keep_ratio": float(keep_ratio),
        "floor_slots": int(floor_slots),
        "window_slots": int(window_slots),
        "seed": int(seed),
        "full_supply": bool(include_full),
    }
    occupancy = mean_window_occupancy(static_packets, target_pps=target_pps)
    with occupancy_floor_slots(floor_slots):
        classic_result = compute_classic_row_result(
            load_or_compute_classic_replay_rows(
                static_path,
                motion_path,
                static_presence_packets=static_packets,
                motion_packets=motion_packets,
                selected_subcarriers=tuple(DEFAULT_SUBCARRIERS),
                replay_kind="classic_occupancy_floor_sweep",
                warmup_packets=window_slots,
                replay_provenance=provenance,
            )
        )
        static_ml = load_or_compute_ml_replay_rows(
            static_path,
            packets=static_packets,
            selected_subcarriers=tuple(DEFAULT_SUBCARRIERS),
            window_size=window_slots,
            stream_provenance={**provenance, "phase": "static_presence"},
        )
        motion_ml = load_or_compute_ml_replay_rows(
            motion_path,
            packets=motion_packets,
            selected_subcarriers=tuple(DEFAULT_SUBCARRIERS),
            window_size=window_slots,
            stream_provenance={**provenance, "phase": "motion"},
        )
        ml_metrics, _payload = _compute_ml_row_result(static_ml, motion_ml, 0.5)

    classic_metrics = None if classic_result is None else classic_result[1]
    return {
        "dataset_id": dataset_id,
        "occupancy": occupancy,
        "keep_ratio": keep_ratio,
        "static_admitted": len(static_admitted),
        "motion_admitted": len(motion_admitted),
        "static_kept": len(static_packets),
        "motion_kept": len(motion_packets),
        "classic": classic_metrics,
        "ml": ml_metrics,
        "low_rssi": is_low_rssi_paired_dataset(static_path),
    }


def print_table(
    title: str,
    points: Sequence[dict[str, Any]],
) -> None:
    print()
    print(title)
    print(
        f"{'occ':>4} {'mean%':>6}  "
        f"{'Lightweight R/minR FP/maxFP':<55}  "
        f"{'High Accuracy R/minR FP/maxFP'}"
    )
    for point in points:
        classic = point["classic"]
        ml = point["ml"]
        label = "full" if point["full_supply"] else str(point["occupancy_percent"])
        print(
            f"{label:>4} {point['mean_occupancy'] * 100.0:6.1f}  "
            f"{_format_detector(classic, calib_fail=int(point['classic_fail'])):<55}  "
            f"{_format_detector(ml, calib_fail=0)}"
        )


def run_sweep(
    floors: Sequence[int],
    *,
    include_full: bool,
    roles: Iterable[str],
    readiness_slots: int | None,
) -> list[dict[str, Any]]:
    pairs = get_available_paired_datasets(synthetic=False, roles=roles)
    if not pairs:
        raise SystemExit("No reserved real paired datasets are available.")
    points: list[dict[str, Any]] = []
    sweep_points: list[tuple[int, bool]] = []
    if include_full:
        sweep_points.append((70, True))
    sweep_points.extend((floor, False) for floor in floors)

    for occupancy_percent, full_supply in sweep_points:
        label = "full" if full_supply else str(occupancy_percent)
        classic_rows: list[dict[str, float]] = []
        ml_rows: list[dict[str, float]] = []
        classic_fail = 0
        occupancies: list[float] = []
        floor_label = (
            occupancy_percent if readiness_slots is None else readiness_slots
        )
        print(
            f"Occupancy {label} on {len(pairs)} reserved pairs "
            f"(target {TARGET_PPS} pps, readiness {floor_label} slots)"
        )
        for index, (static_path, motion_path, _num_sc, chip, dataset_id) in enumerate(
            pairs, start=1
        ):
            print(
                f"  [{index}/{len(pairs)}] {chip} {dataset_id}",
                flush=True,
            )
            result = evaluate_pair(
                static_path=static_path,
                motion_path=motion_path,
                dataset_id=dataset_id,
                occupancy_percent=occupancy_percent,
                target_pps=TARGET_PPS,
                include_full=full_supply,
                readiness_slots=readiness_slots,
            )
            occupancies.append(float(result["occupancy"]))
            if result["classic"] is None:
                classic_fail += 1
            else:
                classic_rows.append(result["classic"])
            ml_rows.append(result["ml"])
        points.append(
            {
                "occupancy_percent": occupancy_percent,
                "full_supply": full_supply,
                "mean_occupancy": (
                    sum(occupancies) / len(occupancies) if occupancies else 0.0
                ),
                "classic": _aggregate(classic_rows),
                "ml": _aggregate(ml_rows),
                "classic_fail": classic_fail,
            }
        )
    return points


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Thin reserved selection and holdout pairs to target occupancy "
            "and score Lightweight and High Accuracy on the fixed 100 pps grid."
        )
    )
    parser.add_argument(
        "--floors",
        type=_parse_floors,
        default=DEFAULT_FLOORS,
        help="Comma-separated occupancy percents, default 70,65,60,55,50",
    )
    parser.add_argument(
        "--no-full",
        action="store_true",
        help="Skip the unthinned production-floor control",
    )
    parser.add_argument(
        "--always-evaluate",
        action="store_true",
        help=(
            "Pin the readiness floor to one valid slot so occupancy holes "
            "stay scored instead of dropping not-ready ticks"
        ),
    )
    args = parser.parse_args(argv)
    readiness_slots = 1 if args.always_evaluate else None
    points = run_sweep(
        args.floors,
        include_full=not args.no_full,
        roles=REPORT_DATASET_ROLES,
        readiness_slots=readiness_slots,
    )
    floor_note = (
        "readiness floor 1 slot, occupancy holes stay scored"
        if args.always_evaluate
        else "matching readiness floor"
    )
    print_table(
        "Reserved selection+holdout occupancy-floor sweep "
        f"(fixed {TARGET_PPS} pps grid, {floor_note})",
        points,
    )
    print()
    print(
        "R/minR is pooled recall / worst-pair recall. "
        "FP/maxFP is pooled false-positive rate / worst-pair FP. "
        "eval is scored idle/motion ticks. "
        "full is unthinned admitted packets. "
        + (
            "Not-ready ticks are scored because the readiness floor is 1 slot."
            if args.always_evaluate
            else "full uses the production 70-slot floor; other rows match the occupancy percent."
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
