# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Dataset quality validation orchestration."""

from . import core
import datetime

import numpy as np

from tools.lib import dataset_metadata
from .capture import (
    validate_capture_file,
)
from .catalog import (
    _domain_summary_rows,
    _entry_matches_chip,
    _is_excluded_entry,
    load_dataset_info,
    save_dataset_info,
    should_recommend_dataset_metadata_refresh,
    validate_metadata_completeness,
)
from .core import (
    ValidationResult,
    _issue_results,
    _result_counts,
    _tag_results,
)
from .pairing import (
    _collect_excluded_idle_rows,
    _collect_excluded_pair_rows,
    _evaluate_pair_capture,
    _excluded_idle_unusable_results,
    refresh_metadata,
    summarize_pair_rows,
)
from .readiness import (
    validate_empty_sanity,
    validate_ml_readiness,
    validate_quiet_test_recordings,
)
from .references import (
    _build_idle_reference_records,
)
from .rendering import (
    _EMPTY_SCORE_TABLE,
    _EXCLUDED_IDLE_SCORE_TABLE,
    _EXCLUDED_PAIR_SCORE_TABLE,
    _LONG_TEST_SCORE_TABLE,
    _PAIR_SCORE_TABLE,
    _PRESENCE_SCORE_TABLE,
    _render_score_table,
)
from .report import (
    _generate_report,
)
from .severity import (
    PER_FILE_QUALITY_LABELS,
    _table_review_profiles,
)

def run_validation(
    chip_filter=None,
    generate_report=True,
    use_cache=True,
    refresh_pair_metadata=True,
    diagnostic_all_phy=False,
):
    """Run full dataset validation."""

    core.configure_validation_mode(diagnostic_all_phy=diagnostic_all_phy)

    print("ESPectre Dataset Quality Validation")
    print(f"Data: {core.DATA_DIR}")
    print(f"Evaluation view: {core._report_evaluation_view()}")
    print(f"Time-aware ML row cache: {'enabled' if use_cache else 'disabled'}")
    if chip_filter:
        print(f"Chip filter: {chip_filter}")

    # Load dataset info
    if core.DATASET_INFO.exists():
        dataset_info = load_dataset_info()
        print(f"dataset_info.json updated_at={dataset_info.get('updated_at', 'unknown')}")
    else:
        print("⚠️  dataset_info.json not found, scanning files directly")
        dataset_info = {'files': {'empty': [], 'static_presence': [], 'motion': []}}

    if core.DATASET_INFO.exists() and refresh_pair_metadata:
        refreshed_info, refreshed_pairs = refresh_metadata(dataset_info, chip_filter=chip_filter)
        summarize_pair_rows(refreshed_pairs)
        if refreshed_info != dataset_info:
            refreshed_info["updated_at"] = datetime.datetime.now().isoformat(
                timespec="microseconds"
            )
            save_dataset_info(refreshed_info)
            print(f"Wrote {core.DATASET_INFO}")
        else:
            print("Metadata unchanged")
        dataset_info = refreshed_info
    elif core.DATASET_INFO.exists():
        print("Metadata refresh disabled; preserving explicit pair references")

    all_results = []
    pair_results = []
    missing_motion_pair_count = 0
    printed_issues_heading = False

    def _emit_issues(results, *, heading):
        nonlocal printed_issues_heading
        issues = _issue_results(results)
        all_results.extend(results)
        if not issues:
            return
        if not printed_issues_heading:
            print("\nIssues (WARN/FAIL only)")
            printed_issues_heading = True
        print(heading)
        for result in issues:
            print(f"   {result}")

    idle_reference_records = _build_idle_reference_records(
        dataset_info,
        chip_filter=chip_filter,
        use_cache=use_cache,
    )

    # ------------------------------------------------------------------
    # Phase 1: Validate required dataset_info metadata
    # ------------------------------------------------------------------
    metadata_results = validate_metadata_completeness(
        dataset_info,
        chip_filter=chip_filter,
    )
    _tag_results(metadata_results, 'integrity')
    _emit_issues(metadata_results, heading="Metadata completeness")

    # ------------------------------------------------------------------
    # Phase 2: Load all NPZ files once, validate integrity & quality
    # ------------------------------------------------------------------
    # Shared NPZ loaders keep the materialized arrays and packet streams warm
    # across later validation phases.
    validated_paths = set()
    for label in PER_FILE_QUALITY_LABELS:
        entries = sorted(
            dataset_info.get("files", {}).get(label, []),
            key=lambda entry: str(entry.get("filename", "")),
        )
        for entry in entries:
            if _is_excluded_entry(entry) or not _entry_matches_chip(entry, chip_filter):
                continue
            npz_file = dataset_metadata.resolve_entry_path(label, entry)
            resolved_path = npz_file.resolve()
            if resolved_path in validated_paths:
                continue
            validated_paths.add(resolved_path)

            file_results = validate_capture_file(
                npz_file,
                low_rssi=bool(entry.get("low_rssi")),
                target_pps=entry.get("nominal_packet_rate"),
            )
            _tag_results(file_results, 'integrity')

            _emit_issues(
                file_results,
                heading=str(npz_file.relative_to(core.DATA_DIR)),
            )

    # ------------------------------------------------------------------
    # Phase 3: Pair validation (static presence <-> motion)
    # ------------------------------------------------------------------
    static_entries = [
        entry
        for entry in dataset_info.get("files", {}).get("static_presence", [])
        if not _is_excluded_entry(entry)
    ]
    motion_entries_by_name = {
        str(item.get("filename", "")): item
        for item in dataset_info.get("files", {}).get("motion", [])
        if not _is_excluded_entry(item)
    }
    if static_entries and motion_entries_by_name:
        for entry in static_entries:
            if not _entry_matches_chip(entry, chip_filter):
                continue

            bl_name = str(entry.get("filename", ""))
            bl_file = dataset_metadata.resolve_entry_path("static_presence", entry)
            mv_name = str(entry.get("optimal_pair_motion_file", ""))
            motion_entry = motion_entries_by_name.get(mv_name)
            best_mv = (
                dataset_metadata.resolve_entry_path("motion", motion_entry)
                if motion_entry is not None
                else None
            )

            if not bl_file.exists():
                _emit_issues(
                    _tag_results(
                        [ValidationResult(
                            "pair_static_missing",
                            "WARN",
                            f"Static-presence file missing: {bl_name}",
                        )],
                        "feature_space",
                    ),
                    heading="Pair validation",
                )
                continue
            if best_mv is None or not best_mv.exists():
                missing_motion_pair_count += 1
                _emit_issues(
                    _tag_results(
                        [ValidationResult(
                            "pair_motion_missing",
                            "WARN",
                            f"No motion pair for: {bl_file.name}",
                        )],
                        "feature_space",
                    ),
                    heading="Pair validation",
                )
                continue

            motion_entry = motion_entry or {}
            pair_res, pair_row, bl_file, mv_file = _evaluate_pair_capture(
                entry,
                motion_entry,
                idle_reference_records=idle_reference_records,
                use_cache=use_cache,
            )
            if pair_row is None:
                _emit_issues(
                    _tag_results(pair_res, "feature_space"),
                    heading=f"Pair {bl_file.name} ↔ {mv_file.name}",
                )
                continue
            _tag_results(pair_res, 'feature_space')
            _emit_issues(
                pair_res,
                heading=f"Pair {bl_file.name} ↔ {mv_file.name}",
            )
            pair_results.append(pair_row)

    # ------------------------------------------------------------------
    # Phase 4: Empty sanity
    # ------------------------------------------------------------------
    empty_results, empty_score_rows, presence_score_rows = validate_empty_sanity(
        dataset_info,
        chip_filter=chip_filter,
        use_cache=use_cache,
    )
    for result in empty_results:
        result.domain = (
            'feature_space'
            if result.name.startswith(('empty_quality/', 'presence_quality/'))
            else 'label_sanity'
        )
    _emit_issues(empty_results, heading="Empty / presence sanity")

    # ------------------------------------------------------------------
    # Phase 5: Quiet-test sanity
    # ------------------------------------------------------------------
    quiet_test_results, quiet_score_rows = validate_quiet_test_recordings(
        dataset_info,
        chip_filter=chip_filter,
        use_cache=use_cache,
    )
    for result in quiet_test_results:
        result.domain = (
            'feature_space' if result.name.startswith('quiet_test_idle/')
            else 'long_recording'
        )
    _emit_issues(quiet_test_results, heading="Quiet-test sanity")

    # ------------------------------------------------------------------
    # Phase 6: ML readiness
    # ------------------------------------------------------------------
    ml_results = validate_ml_readiness(
        dataset_info,
        chip_filter=chip_filter,
        use_cache=use_cache,
    )
    _tag_results(ml_results, 'ml')
    _emit_issues(ml_results, heading="ML readiness")

    excluded_pair_rows = _collect_excluded_pair_rows(
        dataset_info,
        chip_filter=chip_filter,
        idle_reference_records=idle_reference_records,
        use_cache=use_cache,
    )
    excluded_idle_rows = _collect_excluded_idle_rows(
        dataset_info,
        chip_filter=chip_filter,
        idle_reference_records=idle_reference_records,
        use_cache=use_cache,
    )
    excluded_idle_results = _excluded_idle_unusable_results(excluded_idle_rows)
    _tag_results(excluded_idle_results, 'feature_space')
    _emit_issues(excluded_idle_results, heading="Excluded idle diagnostics")

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    counts = _result_counts(all_results)
    fail_count = counts['FAIL']
    review_profiles = _table_review_profiles(
        pair_results,
        presence_score_rows,
        empty_score_rows,
        quiet_score_rows,
    )

    if not printed_issues_heading:
        print("\nNo WARN/FAIL checks")

    print("\nSummary")
    print(
        f"  PASS {counts['PASS']}  WARN {counts['WARN']}  "
        f"FAIL {counts['FAIL']}  total {len(all_results)}"
    )
    print("  | Domain                    | PASS | WARN | FAIL |")
    print("  |---------------------------|-----:|-----:|-----:|")
    for label, domain_counts in _domain_summary_rows(all_results):
        print(
            f"  | {label:<25} | "
            f"{domain_counts['PASS']:>4} | {domain_counts['WARN']:>4} | "
            f"{domain_counts['FAIL']:>4} |"
        )

    if pair_results or quiet_score_rows or empty_score_rows or presence_score_rows:
        print("\nIndicative scores (review only)")
        for line in _render_score_table(
            pair_results,
            _PAIR_SCORE_TABLE,
            review_profiles=review_profiles,
        ):
            print(line)
        if pair_results:
            mean_pair = float(np.mean([p['feature_score'] for p in pair_results]))
            print(f"  Pair mean score: {mean_pair:.1f}/100")
        for rows, table_spec in (
            (presence_score_rows, _PRESENCE_SCORE_TABLE),
            (empty_score_rows, _EMPTY_SCORE_TABLE),
            (quiet_score_rows, _LONG_TEST_SCORE_TABLE),
        ):
            for line in _render_score_table(
                rows,
                table_spec,
                review_profiles=review_profiles,
            ):
                print(line)

    if excluded_pair_rows:
        print("\nExcluded pair diagnostics (informational only)")
        for line in _render_score_table(
            excluded_pair_rows,
            _EXCLUDED_PAIR_SCORE_TABLE,
            review_profiles=review_profiles,
        ):
            print(line)

    if excluded_idle_rows:
        print("\nExcluded idle diagnostics (informational only)")
        for line in _render_score_table(
            excluded_idle_rows,
            _EXCLUDED_IDLE_SCORE_TABLE,
            review_profiles=review_profiles,
        ):
            print(line)

    if should_recommend_dataset_metadata_refresh(
        all_results,
        missing_motion_pair_count=missing_motion_pair_count,
    ):
        print("\n💡 Pair metadata still incomplete after automatic refresh:")
        print("   Check chip, subcarrier, device_id, and collected_at alignment")
        print("   between static_presence and motion captures.")

    if generate_report:
        _generate_report(
            pair_results,
            all_results,
            quiet_score_rows,
            empty_score_rows,
            presence_score_rows,
            excluded_pair_rows,
            excluded_idle_rows,
            review_profiles,
            chip_filter=chip_filter,
        )
        print(f"\nReport: {core.REPORT_OUTPUT}")

    if fail_count > 0:
        print("\n❌ Validation FAILED")
        return 1
    print("\n✅ Validation PASSED")
    return 0
