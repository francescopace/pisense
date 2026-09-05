# ML Training Guide

Train, evaluate, and promote the production High-Accuracy detector after collecting labeled CSI datasets.

Use [ML_DATA_COLLECTION.md](ML_DATA_COLLECTION.md) to collect and validate `empty`, `static_presence`, and `motion` recordings. This guide covers the training workflow, dataset roles, model-selection gates, exported artifacts, and required validation. Use `python tools/train_ml_model.py --help` for the complete option reference, [FEATURES.md](FEATURES.md) for feature evidence, and [performance/README.md](performance/README.md) for current detector results.

This guide is for ML contributors and assumes familiarity with supervised classification. Project-specific terms:

- **Lineage:** recordings that share enough provenance to remain in one validation group.
- **Grouped cross-validation:** fitting and evaluation folds keep each lineage together to reduce leakage.
- **OOF:** out-of-fold predictions produced for samples not used to fit that fold.
- **Replay gate:** a production-aligned recorded-data check that can block model promotion.
- **Promotion:** replacing the exported Python and C++ runtime weights after every required gate passes.

## Prerequisites

Install the ML training stack:

```bash
pip install -r requirements-ml.txt
```

The repository and ML workflows target Python `3.14`.

Before training, validate the corpus and refresh its generated quality report:

```bash
python tools/validate_dataset_quality.py
```

The production trainer admits only the HT20 sensing contract: `phy_mode=ht`, `ltf_type=ht-ltf`, `channel_width=20`, and the stored 64-subcarrier HT20 layout. Historical captures without per-record PHY metadata are admitted only when their payload already matches that layout. Incompatible files fail explicitly.

## Dataset Roles

`data/dataset_info.json` assigns each recording one role:

| Role | Purpose | Used to fit weights |
| --- | --- | --- |
| `train` | Training matrix and lineage-grouped cross-validation | Yes |
| `selection` | Candidate comparison and deployment safety gates | No |
| `holdout` | Final validation of the selected winner | No |
| `exclude` | Retained provenance or diagnostics outside model selection | No |

The trainer treats entries without an explicit role as `exclude`, so an incomplete catalog cannot enter fitting or replay by accident. The quality validator is stricter: every entry must declare `dataset_role`, including entries intentionally kept in `exclude`. It never assigns roles automatically.

An `empty` recording marked `long_recording: true` never enters the training matrix. When its role is `selection` or `holdout`, the quiet gate evaluates the complete recording and can block promotion. A long recording in `exclude` remains available only for explicit diagnostics and the generated quality report.

A normal one-candidate production run evaluates its final candidate on the configured selection and holdout replays before export. Seed search evaluates every candidate on `selection`, chooses one winner, and opens `holdout` only for that winner. Repeatedly consulting holdout results while changing the model turns the holdout into selection data and invalidates its role.

The split policy and its rationale are recorded in [2026-06-30-separate-ml-training-data-from-promotion-replays.md](adr/2026-06-30-separate-ml-training-data-from-promotion-replays.md).

## Production Training Workflow

Inspect the admitted corpus and split first:

```bash
python tools/train_ml_model.py --info
```

Without `--features`, the trainer uses the promoted Subband 8F production order: `turb_iqr_over_mean_aggr`, `turb_autocorr`, `turb_zcr`, `l1_delta_lag_ratio`, `chan_shape_spread_subband`, `chan_shape_coherent_innovation_energy`, `chan_shape_excess_path`, and `chan_shape_subband_kendall_lag_excess`.

Use read-only variants while investigating a change:

```bash
python tools/train_ml_model.py --augment --no-export
python tools/train_ml_model.py --augment --seed SEED --evaluate-selection
```

`--no-export` runs training and grouped CV without replacing artifacts or opening deployment replays. `--evaluate-selection` adds the clean and occupancy-70% selection gates while keeping holdout sealed. Pass an explicit seed for a controlled comparison; when omitted, the trainer reuses the seed embedded in the exported model when available.

After the feature set, configuration, and seed are fixed, promote a runtime-supported finalist with one production run:

```bash
python tools/train_ml_model.py --augment --seed SEED
```

The production run performs grouped cross-validation, fits the candidate, opens the final selection and holdout gates, compares the candidate with the exported baseline, and exports new artifacts only when every promotion requirement passes.

For a host-only finalist that cannot be exported, open the same final gates once without replacing runtime artifacts:

```bash
python tools/train_ml_model.py --augment --seed SEED --evaluate-gates
```

Treat either final result as the validation of a fixed candidate, not as feedback for another tuning round.

For seed search:

```bash
python tools/train_ml_model.py --augment --seed-search-until-improvement TRIALS
python tools/train_ml_model.py --augment --seed-search-until-improvement TRIALS --no-export
```

Seed search writes its report after every trial to `data/auto_generated/mlp_seed_search.json` by default. Runtime-supported feature sets may export the selected winner unless `--no-export` is present. Searches containing host-only candidate features always remain in memory.

Use the same corpus, roles, features, preprocessing, augmentation components, and seed when comparing two training changes unless one of those variables is the subject of the experiment. A result produced without `--augment` is not directly comparable with the promoted augmented workflow.

## Training Contract

The binary target maps `empty` and `static_presence` to IDLE and `motion` to MOTION. Training uses the canonical `stream_dense` contract: it follows the runtime streaming feature path and timing resets, then emits one training row per packet after warmup. The host extractor admits CSI on the same temporal slot grid as the detectors, skips invalid missing slots in turbulence statistics, and bins trajectory features from the admitted packet timestamp.

The trainer:

1. loads admitted `train` recordings and applies the requested timing-quality policy;
2. builds the runtime-aligned feature stream;
3. runs blocked, lineage-grouped cross-validation so related recordings cannot cross folds;
4. reports overall, worst-group, and worst-five-tail metrics;
5. fits the final model on the complete training matrix;
6. evaluates paired and quiet deployment replays; and
7. exports runtime artifacts only after the promotion gates pass.

The current production feature definitions, topology, and retained evidence belong in [FEATURES.md](FEATURES.md). Runtime detector behavior belongs in [ALGORITHMS.md](ALGORITHMS.md). Changing a production feature, its subcarrier band, preprocessing, or runtime arithmetic requires aligned Python and C++ implementations followed by retraining and parity validation.

### Timing-quality policies

Timing quality is provenance, not a model input. The supported policies are:

```bash
python tools/train_ml_model.py --augment --timing-quality-policy keep
python tools/train_ml_model.py --augment --timing-quality-policy exclude-fail
python tools/train_ml_model.py --augment --timing-quality-policy downweight-warn
python tools/train_ml_model.py --augment --timing-quality-policy exclude-fail-downweight-warn
```

Use `--timing-warn-weight` only with a policy that downweights degraded recordings. The performance report groups replay results by timing-quality bucket so policy changes can be evaluated against the underlying provenance.

## Training Augmentation

Bare `--augment` enables the `base,drift,burst-loss` recipe:

- `base` applies moderate feature jitter, packet-domain noise, loss, and stutter, and a stable packet-rate scale from `0.7` to `1.0` with a 70 pps floor that matches the temporal-admission occupancy envelope;
- `drift` injects a slow correlated packet-domain drift episode; and
- `burst-loss` injects short packet-drop bursts.

The exported High Accuracy artifact uses the occupancy-70% `base` scale `0.7-1.0` with a 70 pps floor.

```bash
python tools/train_ml_model.py --augment --seed 656446646 --evaluate-selection
```

Production training builds two deterministic packet views with seeds `20260807` and `20260808`, then keeps alternating row positions from the two views within each source recording. This produces approximately one augmented row set rather than doubling the synthetic sample count, while exposing the model to the complementary false-positive and weak-recall stress tails of both seeds. The seed order and per-file modulo assignment are fixed; model seeds do not alter packet augmentation.

Augmentation is train-only for fitting and promotion gates. Cross-validation scoring, selection, holdout, and runtime inference use clean replay features. The generated performance report additionally labels a non-gating robustness diagnostic that applies the same two-seed packet recipe to the combined `selection + holdout` corpus and compares the exported ML and Lightweight detectors on matching alternating replay positions.

Stable rate scaling is not packet loss. It selects samples across the source interval and rewrites timestamps and sequence numbers to a clean lower cadence. The extractor then admits packets on the same temporal slot grid as the detectors, using recorded `csi_target_pps` or a legacy interval fallback; the window duration stays `1000 ms`, and slot count comes from that target rather than from a later measured-rate resize. Loss and burst-loss augmentations retain gaps as missing slots and contamination, not as a new cadence.

Explicit component lists are useful for controlled ablations:

```bash
python tools/train_ml_model.py --augment base --no-export
python tools/train_ml_model.py --augment base,drift --no-export
python tools/train_ml_model.py --augment base,drift,burst-loss --no-export
```

Historical augmentation comparisons and their measured outcomes belong in ADRs and [FEATURES.md](FEATURES.md), not in this guide.

## Model Selection And Promotion

Promotion is safety-first. The current stable gate policy is:

| Replay class | Recall | Raw FP | Effective alarms |
| --- | ---: | ---: | ---: |
| Normal-link paired replay | `>95%` | `<5%` | At most one per static-presence replay |
| Low-RSSI paired stress replay | `>90%` | `<10%` | Must not regress against the exported baseline |
| Quiet `empty` replay | N/A | `<5%` | Zero |
| Occupancy-70% paired replay | same absolute cuts | same absolute cuts | same alarm rules, after deterministic thinning of reserved pairs to the production occupancy envelope |
| Occupancy-70% quiet replay | N/A | `<5%` | Zero, on the same thinned empty reserved set |

The occupancy-70% gate keeps the production readiness floor. It thins admitted CSI on the fixed `100 pps` grid until mean occupancy is about `70%`, then scores the candidate. Clean reserved replays remain mandatory. Uniform thinning is a conservative proxy for the live envelope; it is not a BLE-coexist test.

Passing the absolute targets is necessary but not sufficient. A candidate must also avoid material per-recording regressions against the exported model. Among safe candidates, the trainer compares paired replay quality, worst-session and worst-chip behavior, worst-five tails, and blocked out-of-fold metrics. Synthetic sessions may protect against regressions but cannot justify promotion over real-data evidence.

`--force-promote --seed SEED` bypasses gate failures and exports a fixed candidate while still printing the failed checks. Reserve it for a deliberate baseline reset whose rationale and evidence will be recorded separately.

Mutable performance numbers belong in [performance/README.md](performance/README.md). Durable feature and model decisions belong in [FEATURES.md](FEATURES.md) and the relevant ADR.

## Research And Diagnostic Workflows

Architecture and false-positive-weight campaigns are read-only and write JSON reports:

```bash
python tools/train_ml_model.py --augment --experiment
python tools/train_ml_model.py --augment --experiment --experiment-architectures "16,8;24,12;32,16"
python tools/train_ml_model.py --augment --experiment-fp-weights "1,1.5,2,2.5,3"
```

Feature diagnostics also leave runtime artifacts unchanged:

```bash
python tools/train_ml_model.py --correlation
python tools/train_ml_model.py --augment --shap 500 --seed SEED --no-export
python tools/train_ml_model.py --augment --ablation-feature FEATURE_OR_JOINT_REMOVAL --seed SEED
```

Candidate features live in `tools/lib/candidate_features.py`. They may be selected with `--features`, but they cannot be exported until they have matching Python and C++ runtime implementations and a published feature ID. Retired candidate evidence remains in `docs/FEATURES.md`; retired implementations are not kept executable solely for historical comparisons. Use `--no-export` for CV-only work, `--evaluate-selection` while comparing candidates, and `--evaluate-gates` once for the fixed finalist.

Trajectory-bin experiments use the same host streaming path and keep the production `80 ms` default unless explicitly overridden:

```bash
python tools/train_ml_model.py --augment --seed SEED --trajectory-bin-ms 50 --evaluate-selection
```

Non-default bins are read-only and cannot export runtime artifacts. Their host feature columns use a bin-specific cache identity, while exported-baseline comparisons always retain the canonical production bin.

Use leave-one-group-out diagnostics to estimate transfer to unseen rooms or chips:

```bash
python tools/train_ml_model.py --augment --cross-environment
python tools/train_ml_model.py --augment --cross-chip
```

These commands train diagnostic folds and never export a promotable model. Grouped CV can still contain the same room or chip on both sides of a fold, so it does not replace these transfer checks.

Use the gain-stress gate to inspect the current exported artifacts without training:

```bash
python tools/train_ml_model.py --gain-stress-gate
python tools/train_ml_model.py --gain-stress-gate --environment bedroom
```

The gain-stress gate measures sensitivity to explicit amplitude-gain dimensions. It does not model low-RSSI feature-floor drift; validate weak links with real `low_rssi` recordings.

## Exported Artifacts

A successful promotion updates:

- `tools/lib/ml_weights.py`;
- `src/cpp/core/ml_weights.h`; and
- `data/auto_generated/ml_test_data.npz`.

The exported weight files store the training seed, timestamp, feature order, scaler, topology, and complete runtime arrays. They do not embed the training corpus revision or the full selection policy. Record the exact `dataset_info.json` revision, admitted roles, timing-quality policy, augmentation recipe, and fitting parameters in the current production section of [FEATURES.md](FEATURES.md). `ml_test_data.npz` is an inference-regression artifact, not a model-selection score.

Do not edit generated weight files manually. Export them through the trainer so Python, C++, and regression data remain aligned.

## Cache Maintenance

Training and replay tools persist runtime-aligned feature artifacts under `.cache/npz/`. Runtime-supported replay rows remain cached as complete matrices. Host-side experiments use a shared row spine (`packet_index`, evaluation index, reset index, and evaluation cadence) plus one independently keyed column per feature. Each column identity contains its feature-local formula and provider contract, so adding a sibling variant computes only the new column; existing columns remain reusable across reordered feature sets and model comparisons. Shared provider-contract changes invalidate only columns owned by that provider. The row-spine check rejects columns whose replay coordinates diverge rather than silently combining incompatible data.

Cache keys include source data, timing behavior, implementation identity, and augmentation provenance. Lightweight source-admission metadata is cached separately, and warm host-column hits do not materialize the CSI packet stream. Mixed production augmentation remains cached per source under `ml_training_augmentation_rows`; host-only augmented views also persist their full row spine and feature columns before deterministic row selection. Producers take a per-artifact process lock and recheck after acquiring it, preventing concurrent seed searches or reports from rebuilding the same cold key. Long cache fills print periodic `[npz-cache]` progress on stderr for hits, misses, in-progress builds, and writes. Progress is on when stderr is a TTY; set `ESPECTRE_NPZ_CACHE_PROGRESS=0` to disable it or `=1` to force it, and optionally `ESPECTRE_NPZ_CACHE_PROGRESS_INTERVAL_S` to change the default `10` second heartbeat. Cache entries, seed-search JSON, generated reports, dataset metadata, and individual exports use atomic replacement. The three model outputs are staged and published as one rollback-capable set, and seed-search rollback removes outputs that did not exist before the search.

Use `--no-cache` for a cold feature-row diagnostic run; source-admission metadata remains cached because it does not change the matrix. Use `python tools/prune_npz_cache.py` to remove artifacts that are no longer reachable, and add an explicit age or size limit when historical parameter variants should also be retired. Generated report freshness checks include a digest of detector, model, tool, and capture inputs in addition to `dataset_info.json`. Detailed cache behavior and pruning options belong in [tools/README.md](../tools/README.md).

## Required Validation

After changing detection logic, features, preprocessing, or exported weights, run both required parity gates:

```bash
cmake -S test/cpp -B test/cpp/build
cmake --build test/cpp/build
ctest --test-dir test/cpp/build -R test_motion_detection --output-on-failure
.venv/bin/pytest test/python/performance/test_validation_real_data.py::TestPerformanceMetrics -v
```

Validate long quiet recordings and regenerate the performance report:

```bash
.venv/bin/pytest test/python/performance/test_validation_long_recordings.py -v
.venv/bin/python tools/generate_performance_report.py
.venv/bin/python tools/generate_performance_report.py --check-current
```

When the corpus, roles, or dataset-quality logic changes, also regenerate and verify the quality report:

```bash
.venv/bin/python tools/validate_dataset_quality.py
.venv/bin/python tools/validate_dataset_quality.py --check-current
.venv/bin/pytest test/python/host/dataset/test_dataset_quality_validation.py -v
```

Do not claim a promotion is complete until the generated reports are current, every required Python/C++ gate passes, and [FEATURES.md](FEATURES.md) identifies the promoted run's corpus revision and training contract.

## Related Documentation

- [ML_DATA_COLLECTION.md](ML_DATA_COLLECTION.md): collection and labeling workflow
- [FEATURES.md](FEATURES.md): production feature set, research ledger, and retained evidence
- [ALGORITHMS.md](ALGORITHMS.md#high-accuracy-implementation-highaccuracydetector): runtime detector behavior
- [performance/README.md](performance/README.md): generated current performance
- [tools/README.md](../tools/README.md): complete tool reference and cache operations
- [2026-06-30-separate-ml-training-data-from-promotion-replays.md](adr/2026-06-30-separate-ml-training-data-from-promotion-replays.md): split and promotion rationale
