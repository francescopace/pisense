# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Packet-rate adaptation regression tests

Regression coverage for 60-second prefixes of every explicit
static-presence/motion pair whose metadata reports
``average_packet_rate >= 500``. Each high-rate pair is decimated across the
supported operating region, and the test checks that:

- the derived detector timing follows the measured cadence,
- evaluation ticks stay time-based instead of packet-count based, and
- both Lightweight and High Accuracy keep good replay quality on the slower streams.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import pytest

from config import DEFAULT_SUBCARRIERS
from runtime_policy import derive_detector_timing
from support.chip_matrix import DETECTION_CHIPS, chip_label
from support.dataset_cases import DATA_DIR, DATASET_INFO_PATH
from tools.lib.dataset_metadata import measure_packet_interval_us
from tools.lib.performance_report import (
    _compute_ml_row_result,
    compute_classic_row_result,
    compute_classic_packet_result,
    load_or_compute_classic_replay_rows,
    load_or_compute_ml_replay_rows,
    load_real_data_cached,
)


MINIMUM_SOURCE_AVERAGE_PACKET_RATE = 500.0
REPLAY_DURATION_SECONDS = 60
TARGET_PPS = (120, 100, 80)


@dataclass(frozen=True)
class PacketRateSourcePair:
    chip: str
    pair_id: str
    static_filename: str
    motion_filename: str
    source_pps: int
    average_packet_rate: float


def _dataset_path(label: str, filename: str) -> Path:
    return DATA_DIR / label / filename


def _entry_average_packet_rate(entry: dict[str, object]) -> float:
    value = entry.get("average_packet_rate")
    try:
        resolved = float(value)
    except (TypeError, ValueError):
        resolved = 0.0
    if resolved > 0.0:
        return resolved
    duration_ms = float(entry.get("duration_ms", 0.0) or 0.0)
    num_packets = int(entry.get("num_packets", 0) or 0)
    if duration_ms > 0.0 and num_packets > 0:
        return num_packets * 1000.0 / duration_ms
    return 0.0


def _entry_nominal_packet_rate(entry: dict[str, object]) -> int | None:
    value = entry.get("nominal_packet_rate")
    try:
        resolved = int(value)
    except (TypeError, ValueError):
        return None
    if resolved > 0:
        return resolved
    return None


@lru_cache(maxsize=1)
def _source_pairs() -> tuple[PacketRateSourcePair, ...]:
    if not DATASET_INFO_PATH.exists():
        return ()
    with DATASET_INFO_PATH.open("r", encoding="utf-8") as handle:
        dataset_info = json.load(handle)

    motion_by_filename = {
        str(entry.get("filename")): entry
        for entry in dataset_info.get("files", {}).get("motion", [])
        if entry.get("filename")
    }
    pairs: list[PacketRateSourcePair] = []
    for static_entry in dataset_info.get("files", {}).get("static_presence", []):
        static_filename = str(static_entry.get("filename") or "")
        if not static_filename:
            continue
        average_packet_rate = _entry_average_packet_rate(static_entry)
        if average_packet_rate < MINIMUM_SOURCE_AVERAGE_PACKET_RATE:
            continue
        motion_filename = str(static_entry.get("optimal_pair_motion_file") or "")
        if not motion_filename:
            continue
        motion_entry = motion_by_filename.get(motion_filename)
        if motion_entry is None:
            continue
        if _entry_average_packet_rate(motion_entry) < MINIMUM_SOURCE_AVERAGE_PACKET_RATE:
            continue
        source_pps = _entry_nominal_packet_rate(static_entry)
        if source_pps is None:
            continue
        pair_id = Path(static_filename).stem
        pairs.append(
            PacketRateSourcePair(
                chip=chip_label(str(static_entry.get("chip", ""))),
                pair_id=pair_id,
                static_filename=static_filename,
                motion_filename=motion_filename,
                source_pps=source_pps,
                average_packet_rate=average_packet_rate,
            )
        )
    pairs.sort(key=lambda pair: (pair.source_pps, pair.static_filename, pair.motion_filename))
    return tuple(pairs)


def _pair_params() -> list[object]:
    pairs = _source_pairs()
    params: list[object] = []
    for chip in DETECTION_CHIPS:
        matches = [pair for pair in pairs if pair.chip == chip]
        if not matches:
            params.append(
                pytest.param(
                    None,
                    marks=pytest.mark.skip(
                        reason=f"No eligible packet_rate dataset for chip {chip}"
                    ),
                    id=f"{chip.lower()}_no_eligible_packet_rate_dataset",
                )
            )
            continue
        params.extend(
            pytest.param(
                pair,
                id=f"{chip.lower()}_{pair.source_pps}pps_{pair.pair_id}",
            )
            for pair in matches
        )
    return params


@lru_cache(maxsize=None)
def _load_source_pair(
    pair_spec: PacketRateSourcePair,
) -> tuple[tuple[dict, ...], tuple[dict, ...]]:
    static_path = _dataset_path("static_presence", pair_spec.static_filename)
    motion_path = _dataset_path("motion", pair_spec.motion_filename)
    assert static_path.exists(), f"Missing regression dataset: {static_path}"
    assert motion_path.exists(), f"Missing regression dataset: {motion_path}"
    return load_real_data_cached(static_path, motion_path)


def _decimate_packets(
    packets: tuple[dict, ...],
    *,
    source_pps: int,
    target_pps: int,
) -> tuple[dict, ...]:
    """Select packets at the target cadence and resequence them contiguously."""
    if target_pps >= source_pps:
        return packets

    stride = float(source_pps) / float(target_pps)
    interval_us = int(round(1_000_000.0 / float(target_pps)))
    decimated = []
    cursor = 0.0
    next_seq_num = None
    next_device_ticks_us = None
    next_wifi_rx_ts_us = None

    while True:
        source_index = int(round(cursor))
        if source_index >= len(packets):
            break

        packet = dict(packets[source_index])
        if next_seq_num is None:
            next_seq_num = int(packet.get("seq_num", packet.get("stream_seq_num", 0)) or 0)
        else:
            next_seq_num += 1
        packet["seq_num"] = next_seq_num
        packet["stream_seq_num"] = next_seq_num
        if "device_ticks_us" in packet and packet["device_ticks_us"] is not None:
            if next_device_ticks_us is None:
                next_device_ticks_us = int(packet["device_ticks_us"])
            else:
                next_device_ticks_us += interval_us
            packet["device_ticks_us"] = next_device_ticks_us
        if "wifi_rx_ts_us" in packet and packet["wifi_rx_ts_us"] is not None:
            if next_wifi_rx_ts_us is None:
                next_wifi_rx_ts_us = int(packet["wifi_rx_ts_us"])
            else:
                next_wifi_rx_ts_us = (next_wifi_rx_ts_us + interval_us) % (1 << 32)
            packet["wifi_rx_ts_us"] = next_wifi_rx_ts_us
        decimated.append(packet)
        cursor += stride

    return tuple(decimated)


@lru_cache(maxsize=None)
def _decimated_pair(
    pair_spec: PacketRateSourcePair,
    target_pps: int,
) -> tuple[tuple[dict, ...], tuple[dict, ...]]:
    static_packets, motion_packets = _load_source_pair(pair_spec)
    source_packet_limit = pair_spec.source_pps * REPLAY_DURATION_SECONDS
    static_packets = static_packets[:source_packet_limit]
    motion_packets = motion_packets[:source_packet_limit]
    return (
        _decimate_packets(static_packets, source_pps=pair_spec.source_pps, target_pps=target_pps),
        _decimate_packets(motion_packets, source_pps=pair_spec.source_pps, target_pps=target_pps),
    )


@lru_cache(maxsize=None)
def _rate_summary(pair_spec: PacketRateSourcePair, target_pps: int) -> dict[str, object]:
    static_packets, motion_packets = _decimated_pair(pair_spec, target_pps)
    interval_us = measure_packet_interval_us(static_packets)
    timing = derive_detector_timing(interval_us)
    window_packets = timing["window_packets"]
    static_path = _dataset_path("static_presence", pair_spec.static_filename)
    motion_path = _dataset_path("motion", pair_spec.motion_filename)
    replay_provenance = {
        "transform": "packet_rate_decimation",
        "transform_version": 1,
        "pair_id": pair_spec.pair_id,
        "source_pps": pair_spec.source_pps,
        "target_pps": target_pps,
        "duration_seconds": REPLAY_DURATION_SECONDS,
    }

    classic_packet_result = compute_classic_packet_result(
        static_packets,
        motion_packets,
        tuple(DEFAULT_SUBCARRIERS),
        window_packets,
    )
    assert classic_packet_result is not None, (
        f"Lightweight packet replay calibration failed at {target_pps} pps"
    )
    classic_packet_threshold, classic_packet_metrics = classic_packet_result

    classic_result = compute_classic_row_result(
        load_or_compute_classic_replay_rows(
            static_path,
            motion_path,
            static_presence_packets=static_packets,
            motion_packets=motion_packets,
            selected_subcarriers=tuple(DEFAULT_SUBCARRIERS),
            replay_kind="classic_packet_rate_adaptation",
            warmup_packets=window_packets,
            replay_provenance=replay_provenance,
        )
    )
    assert classic_result is not None, (
        f"Lightweight row replay calibration failed at {target_pps} pps"
    )
    classic_threshold, classic_metrics = classic_result

    static_ml_rows = load_or_compute_ml_replay_rows(
        static_path,
        packets=static_packets,
        selected_subcarriers=tuple(DEFAULT_SUBCARRIERS),
        window_size=window_packets,
        stream_provenance={**replay_provenance, "phase": "static_presence"},
    )
    motion_ml_rows = load_or_compute_ml_replay_rows(
        motion_path,
        packets=motion_packets,
        selected_subcarriers=tuple(DEFAULT_SUBCARRIERS),
        window_size=window_packets,
        stream_provenance={**replay_provenance, "phase": "motion"},
    )
    ml_metrics, _feature_payload = _compute_ml_row_result(
        static_ml_rows,
        motion_ml_rows,
        0.5,
    )

    return {
        "pair_id": pair_spec.pair_id,
        "source_pps": pair_spec.source_pps,
        "average_packet_rate": pair_spec.average_packet_rate,
        "target_pps": target_pps,
        "static_packets": len(static_packets),
        "motion_packets": len(motion_packets),
        "interval_us": interval_us,
        "timing": timing,
        "classic_threshold": classic_threshold,
        "classic": classic_metrics,
        "classic_packet_threshold": classic_packet_threshold,
        "classic_packet": classic_packet_metrics,
        "ml": ml_metrics,
    }


def _format_compact_summary_table(summaries: list[dict[str, object]]) -> str:
    """Return one fixed-width summary table for the full packet-rate sweep."""
    headers = (
        "pair",
        "src",
        "pps",
        "timing",
        "Lightweight R/FP",
        "ML R/FP",
        "eval idle/motion",
    )
    rows = []
    for summary in summaries:
        timing = summary["timing"]
        classic = summary["classic"]
        ml = summary["ml"]
        rows.append(
            (
                f"{summary['pair_id']}",
                f"{summary['source_pps']}",
                f"{summary['target_pps']}",
                f"w{timing['window_packets']} l{timing['lag']} a{timing['autocorr_lag']}",
                f"{classic['recall']:.1f}% / {classic['fp_rate']:.1f}%",
                f"{ml['recall']:.1f}% / {ml['fp_rate']:.1f}%",
                f"{classic['num_baseline']} / {classic['num_movement']}",
            )
        )

    widths = [
        max(len(str(value)) for value in (header, *(row[index] for row in rows)))
        for index, header in enumerate(headers)
    ]

    def render_row(values: tuple[str, ...]) -> str:
        return " | ".join(
            str(value).ljust(widths[index]) for index, value in enumerate(values)
        )

    separator = "-+-".join("-" * width for width in widths)
    body = [render_row(headers), separator]
    body.extend(render_row(row) for row in rows)
    return "\n".join(body)


@pytest.mark.parametrize("pair_spec", _pair_params())
def test_packet_rate_adaptation_regression_matrix(pair_spec: PacketRateSourcePair) -> None:
    """Validate the supported packet-rate range on bounded replay prefixes."""
    summaries = [_rate_summary(pair_spec, target_pps) for target_pps in TARGET_PPS]
    baseline_counts = [summary["classic"]["num_baseline"] for summary in summaries]
    movement_counts = [summary["classic"]["num_movement"] for summary in summaries]
    print(
        "\nPacket-rate adaptation summary "
        f"for {pair_spec.pair_id} (nominal={pair_spec.source_pps} pps, "
        f"average={pair_spec.average_packet_rate:.1f} pps)"
    )
    print(_format_compact_summary_table(summaries))

    assert min(baseline_counts) >= 220
    assert max(baseline_counts) <= 245
    assert max(baseline_counts) - min(baseline_counts) <= 20

    assert min(movement_counts) >= 220
    assert max(movement_counts) <= 245
    assert max(movement_counts) - min(movement_counts) <= 20

    for summary, target_pps in zip(summaries, TARGET_PPS, strict=True):
        timing = summary["timing"]
        expected_interval_us = int(round(1_000_000.0 / float(target_pps)))
        expected = derive_detector_timing(expected_interval_us)

        assert abs(int(summary["interval_us"]) - expected_interval_us) <= 1, (
            f"{target_pps} pps measured interval {summary['interval_us']} "
            f"instead of {expected_interval_us}"
        )

        for field in ("window_packets", "lag", "autocorr_lag"):
            expected_value = expected[field]
            assert timing[field] == expected_value, (
                f"{target_pps} pps resolved {field}={timing[field]} "
                f"instead of {expected_value}"
            )

        assert summary["classic_threshold"] == pytest.approx(
            summary["classic_packet_threshold"], abs=1e-12
        )
        for key in (
            "tp",
            "fn",
            "tn",
            "fp",
            "num_baseline",
            "num_movement",
            "effective_alarms",
            "false_motion_evaluations",
        ):
            assert summary["classic"][key] == summary["classic_packet"][key]
        for key in ("recall", "precision", "fp_rate", "f1"):
            assert summary["classic"][key] == pytest.approx(
                summary["classic_packet"][key], abs=1e-12
            )

        classic = summary["classic"]
        ml = summary["ml"]
        assert classic["recall"] >= 95.0, (
            f"Lightweight recall regressed at {target_pps} pps: {classic['recall']:.1f}%"
        )
        classic_fp_limit = 1.2 if target_pps <= 80 else 1.0
        assert classic["fp_rate"] <= classic_fp_limit, (
            f"Lightweight FP rate regressed at {target_pps} pps: {classic['fp_rate']:.1f}%"
        )
        assert ml["recall"] >= 95.0, (
            f"ML recall regressed at {target_pps} pps: {ml['recall']:.1f}%"
        )
        assert ml["fp_rate"] <= 1.0, (
            f"ML FP rate regressed at {target_pps} pps: {ml['fp_rate']:.1f}%"
        )
