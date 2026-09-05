#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Data Quality Analysis
Verifies data integrity, analyzes SNR statistics, and checks turbulence variance

Usage:
    python tools/analyze_raw_data.py           # Analyze all available datasets
    python tools/analyze_raw_data.py --chip C6 # Analyze only C6 dataset
    python tools/analyze_raw_data.py --chip S3 # Analyze only S3 dataset

Author: Francesco Pace <francesco.pace@gmail.com>
"""

import argparse
import sys
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.lib.bootstrap import setup_paths  # noqa: F401

from tools.lib.csi_analysis import calculate_spatial_turbulence
from tools.lib.csi_io import load_npz_as_packets, load_static_presence_and_motion
from tools.lib.dataset_metadata import (
    load_dataset_info,
    resolve_entry_path,
    resolve_explicit_pair,
    select_dataset_interactively,
)


def format_variance(value: float, width: int = 12) -> str:
    """
    Format variance for readability.

    Values below 0.01 are shown in scientific notation to avoid displaying 0.00.
    """
    if width <= 0:
        if value < 0.01:
            return f"{value:.2e}"
        return f"{value:.2f}"
    if value < 0.01:
        return f"{value:>{width}.2e}"
    return f"{value:>{width}.2f}"


def discover_available_chips() -> list:
    """
    Discover all chip types that have both static-presence and motion data.
    
    Returns:
        list: Sorted list of chip names (e.g., ['C6', 'S3'])
    """
    info = load_dataset_info()
    available = {
        str(entry.get('chip', '')).upper()
        for entry in info.get('files', {}).get('static_presence', [])
        if entry.get('optimal_pair_motion_file')
    }
    return sorted(chip for chip in available if chip)


def analyze_packets(packets, label_name):
    """Analyze a list of packets and return statistics"""
    print(f"\n{'='*70}")
    print(f"  Analyzing: {label_name}")
    print(f"{'='*70}")
    
    if not packets:
        print("Error: No packets found")
        return None
    
    # Extract label from first packet
    label = packets[0].get('label', 'unknown')
    
    print("\nDataset Information:")
    print(f"  Label: {label}")
    print(f"  Total Packets: {len(packets)}")
    
    # Calculate turbulence and RSSI for each packet
    turbulences = []
    rssi_values = []
    
    for pkt in packets:
        turb = calculate_spatial_turbulence(
            pkt['csi_data']
        )
        turbulences.append(turb)
        rssi_values.append(pkt.get('rssi', 0))
    
    print("\nRSSI Statistics:")
    print(f"  Mean: {np.mean(rssi_values):.2f} dBm")
    print(f"  Std:  {np.std(rssi_values):.2f} dBm")
    
    print("\nTurbulence Statistics:")
    print(f"  Mean: {np.mean(turbulences):.2f}")
    print(f"  Std:  {np.std(turbulences):.2f}")
    
    turb_variance = np.var(turbulences)
    print(f"\nTurbulence Variance: {format_variance(turb_variance, width=0)}")
    print("  (Higher motion/static variance ratio means clearer separation)")
    
    return {
        'label_name': label,
        'packet_count': len(packets),
        'turb_mean': np.mean(turbulences),
        'turb_std': np.std(turbulences),
        'turb_variance': turb_variance,
        'rssi_mean': np.mean(rssi_values),
        'rssi_std': np.std(rssi_values)
    }


def compute_packet_stats(packets):
    """Compute turbulence statistics without verbose prints."""
    if not packets:
        return None

    label = str(packets[0].get('label', 'unknown'))
    turbulences = []
    rssi_values = []

    for pkt in packets:
        turb = calculate_spatial_turbulence(
            pkt['csi_data']
        )
        turbulences.append(turb)
        rssi_values.append(pkt.get('rssi', 0))

    return {
        'label_name': label,
        'packet_count': len(packets),
        'turb_mean': float(np.mean(turbulences)),
        'turb_std': float(np.std(turbulences)),
        'turb_variance': float(np.var(turbulences)),
        'rssi_mean': float(np.mean(rssi_values)),
        'rssi_std': float(np.std(rssi_values)),
    }


def analyze_all_pairs_from_dataset_info() -> list:
    """
    Analyze all explicit static-presence/motion pairs from dataset_info.json.

    Returns:
        list: table rows sorted by chip asc and ratio desc
    """
    info = load_dataset_info()
    files = info.get('files', {})
    static_presence_entries = files.get('static_presence', [])
    motion_entries = files.get('motion', [])
    motion_by_name = {m.get('filename'): m for m in motion_entries}

    rows = []
    for baseline in static_presence_entries:
        static_presence_name = baseline.get('filename')
        motion_name = baseline.get('optimal_pair_motion_file')

        if not static_presence_name or not motion_name:
            continue
        if motion_name not in motion_by_name:
            continue

        static_presence_path = resolve_entry_path('static_presence', baseline)
        motion_path = resolve_entry_path('motion', motion_by_name[motion_name])
        if not static_presence_path.exists() or not motion_path.exists():
            continue

        static_presence_packets = load_npz_as_packets(static_presence_path)
        motion_packets = load_npz_as_packets(motion_path)
        static_presence_stats = compute_packet_stats(static_presence_packets)
        motion_stats = compute_packet_stats(motion_packets)
        if static_presence_stats is None or motion_stats is None:
            continue

        static_presence_ok = static_presence_stats['label_name'].lower() == 'static_presence'
        motion_ok = motion_stats['label_name'].lower() == 'motion'
        variance_ok = static_presence_stats['turb_variance'] < motion_stats['turb_variance']

        static_presence_var = static_presence_stats['turb_variance']
        motion_var = motion_stats['turb_variance']
        ratio = motion_var / static_presence_var if static_presence_var > 0 else 0.0

        # Temporal gap between static-presence end and motion start
        static_presence_start = datetime.fromisoformat(baseline['collected_at'])
        motion_start = datetime.fromisoformat(
            motion_by_name[motion_name]['collected_at']
        )
        static_presence_end = static_presence_start + timedelta(milliseconds=int(baseline.get('duration_ms', 0)))
        gap_seconds = (motion_start - static_presence_end).total_seconds()

        status = "PASS" if (static_presence_ok and motion_ok and variance_ok) else "FAIL"
        rows.append({
            'chip': str(baseline.get('chip', '?')).upper(),
            'pair': f"{static_presence_name} / {motion_name}",
            'static_presence_var': static_presence_var,
            'motion_var': motion_var,
            'ratio': ratio,
            'gap_seconds': gap_seconds,
            'status': status,
        })

    return sorted(rows, key=lambda r: (r['chip'], -r['ratio']))


def print_pairs_table(rows: list):
    """Print compact table for all historical pairs."""
    print(f"\n{'='*70}")
    print("  HISTORICAL PAIRS TABLE (dataset_info.json)")
    print(f"{'='*70}")
    print(
        f"\n{'Chip':<6} {'Static Var':>12} {'Motion Var':>12} "
        f"{'Ratio':>8} {'Gap end->start':>15} {'Status':<8} File pair"
    )
    print(f"{'-'*6} {'-'*12} {'-'*12} {'-'*8} {'-'*15} {'-'*8} {'-'*50}")

    pass_count = 0
    for row in rows:
        if row['status'] == "PASS":
            pass_count += 1
        print(
            f"{row['chip']:<6} {format_variance(row['static_presence_var'])} {format_variance(row['motion_var'])} "
            f"{row['ratio']:>7.2f}x {row['gap_seconds']:>14.2f}s {row['status']:<8} {row['pair']}"
        )

    fail_count = len(rows) - pass_count
    print()
    print(f"SUMMARY: total={len(rows)} pass={pass_count} fail={fail_count}")
    if fail_count == 0:
        print("VERDICT: All historical paired datasets are valid")
    else:
        print("VERDICT: Some historical pairs have issues")
    print()


def analyze_chip(chip: str | None = None, *, dataset: str | None = None, interactive: bool = False) -> dict:
    """
    Analyze dataset for a specific chip.
    
    Args:
        chip: Chip type (C6, S3, etc.)
    
    Returns:
        dict with analysis results or None if failed
    """
    print(f"\n{'#'*70}")
    print(f"#  CHIP: {chip or 'AUTO'}")
    print(f"{'#'*70}")
    
    try:
        if interactive:
            selected = select_dataset_interactively(
                chip=chip,
                num_sc=64,
                require_pair=True,
                prompt='Select dataset for raw-data analysis',
            )
            pair = resolve_explicit_pair(dataset=selected.path.name, num_sc=64)
        else:
            pair = resolve_explicit_pair(dataset=dataset, chip=chip, num_sc=64)
        static_presence_path = pair.static_presence.path
        motion_path = pair.motion.path
        chip_name = pair.chip
        print("\nDataset files:")
        print(f"  Static presence: {static_presence_path.name}")
        print(f"  Motion:          {motion_path.name}")
        
        static_presence_packets, motion_packets = load_static_presence_and_motion(
            static_presence_file=static_presence_path,
            motion_file=motion_path,
            chip=chip_name,
            dataset=dataset,
        )
    except FileNotFoundError as e:
        print(f"\nError: {e}")
        return None
    
    static_presence_stats = analyze_packets(static_presence_packets, f"{chip} static presence")
    motion_stats = analyze_packets(motion_packets, f"{chip} motion")
    
    if static_presence_stats is None or motion_stats is None:
        return None
    
    # Validation
    static_presence_ok = static_presence_stats['label_name'].lower() == 'static_presence'
    motion_ok = motion_stats['label_name'].lower() == 'motion'
    variance_ok = static_presence_stats['turb_variance'] < motion_stats['turb_variance']
    
    result = {
        'chip': chip_name,
        'static_presence': static_presence_stats,
        'motion': motion_stats,
        'labels_ok': static_presence_ok and motion_ok,
        'variance_ok': variance_ok,
        'valid': static_presence_ok and motion_ok and variance_ok
    }
    
    return result


def print_summary(results: list):
    """Print summary table for all analyzed chips"""
    print(f"\n{'='*70}")
    print("  SUMMARY")
    print(f"{'='*70}")
    
    # Header
    print(f"\n{'Chip':<6} {'Static Var':>12} {'Motion Var':>12} {'Ratio':>8} {'Status':<10}")
    print(f"{'-'*6} {'-'*12} {'-'*12} {'-'*8} {'-'*10}")
    
    all_valid = True
    for r in results:
        if r is None:
            continue
        
        static_presence_var = r['static_presence']['turb_variance']
        motion_var = r['motion']['turb_variance']
        ratio = motion_var / static_presence_var if static_presence_var > 0 else 0
        
        if r['valid']:
            status = "OK"
        elif not r['labels_ok']:
            status = "LABEL ERR"
            all_valid = False
        else:
            status = "SWAPPED?"
            all_valid = False
        
        print(f"{r['chip']:<6} {format_variance(static_presence_var)} {format_variance(motion_var)} {ratio:>8.1f}x {status:<10}")
    
    print()
    
    if all_valid:
        print("VERDICT: All datasets are correctly labeled and contain expected data")
    else:
        print("VERDICT: Some datasets have issues (see status column)")
    
    print()


def main():
    raw_args = __import__('sys').argv[1:]
    chip_explicit = '--chip' in raw_args
    parser = argparse.ArgumentParser(
        description='Analyze raw CSI data quality for all available datasets'
    )
    parser.add_argument(
        '--chip',
        type=str,
        help='Analyze only this chip type (e.g., C6, S3). Default: analyze all'
    )
    parser.add_argument(
        '--dataset',
        type=str,
        help='Dataset filename, stem, or dataset id; pair is resolved from metadata'
    )
    parser.add_argument(
        '--interactive',
        action='store_true',
        help='Choose the dataset interactively from dataset_info.json'
    )
    args = parser.parse_args()
    
    print("\n" + "=" * 70)
    print("  Data File Verification Tool")
    print("=" * 70)
    
    if args.chip or args.dataset or args.interactive:
        chip = args.chip.upper() if args.chip and chip_explicit and not args.dataset else None
        result = analyze_chip(chip, dataset=args.dataset, interactive=args.interactive)
        if result is not None:
            print_summary([result])
        return

    # Default mode: export historical table from dataset_info pairs
    rows = analyze_all_pairs_from_dataset_info()
    if not rows:
        print("\nError: No valid static-presence/motion pairs found in dataset_info.json")
        return
    print_pairs_table(rows)


if __name__ == '__main__':
    main()
