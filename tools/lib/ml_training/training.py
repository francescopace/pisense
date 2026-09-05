# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Model fitting, cross-validation, and training orchestration."""

from __future__ import annotations

from tools.lib.bootstrap import setup_paths

setup_paths()

import os
import sys
import argparse
import copy
import json
import numpy as np
import random
import re
import shutil
import tempfile
from pathlib import Path
from tools.lib.atomic_io import atomic_write_set, atomic_write_text
from contextlib import contextmanager
from datetime import datetime
from time import perf_counter
from tools.lib.dataset_metadata import DATA_DIR
from tools.lib.csi_features import (
    DEFAULT_FEATURES,
)

try:
    import torch
    import torch.nn as nn
except ImportError:
    torch = None
    nn = None

from .augmentation import (
    FIXED_PACKET_AUGMENTATION_SEEDS,
    _append_augmented_training_rows,
    _stable_text_seed,
    derive_seed,
    format_augmentation_config,
    parse_augmentation_components,
    resolve_training_augmentation,
    training_packet_augmentation_seeds,
)

from .dataset import (
    CPP_DIR,
    DEFAULT_TIMING_QUALITY_POLICY,
    DEFAULT_TIMING_WARN_WEIGHT,
    GENERATED_DATA_DIR,
    REFERENCE_SRC_DIR,
    apply_positive_chip_boost,
    format_duration,
    is_motion_label,
    load_all_data,
    load_dataset_info,
    load_training_matrix,
    parse_chip_filter,
    parse_environment_filter,
)

from .evaluation import (
    DEFAULT_REPORT_GROUP_KEYS,
    build_candidate_key,
    build_group_report,
    compare_robust_cv,
    evaluate_candidate_gain_stress,
    evaluate_exported_occupancy_paired_gate,
    evaluate_exported_paired_gate,
    evaluate_occupancy_paired_gate,
    evaluate_occupancy_quiet_gate,
    evaluate_paired_gate,
    evaluate_probabilities,
    evaluate_quiet_gate,
    in_memory_gate_result,
    print_gain_stress_summary,
    run_exported_ml_gates,
)

from .export import (
    CPP_FEATURE_IDS,
    DEFAULT_HIDDEN_LAYERS,
    TorchMLP,
    _backup_artifacts,
    _restore_artifacts,
    describe_torch_device,
    ensure_torch_available,
    export_cpp_weights,
    export_micropython,
    export_test_data,
    predict_probabilities,
    resolve_torch_device,
)

from .feature_cache import (
    DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
    TRAINING_FEATURES,
    TRAINING_SAMPLE_CONTRACT,
)

from .preprocessing import (
    DEFAULT_BLOCK_GROUP_KEY,
    DEFAULT_SCALER_MODE,
    augment_normalized_features,
    build_block_mask,
    build_preprocessor,
    distribute_samples,
    fit_preprocessor,
    normalized_feature_bounds,
    select_balanced_shap_indices,
    slice_sample_context,
)

@contextmanager
def suppress_stderr():
    """
    Context manager to suppress stderr output at the file descriptor level.

    Some native libraries write directly to the C-level stderr, bypassing
    Python's sys.stderr.
    """
    # Save the original stderr file descriptor
    stderr_fd = sys.stderr.fileno()
    saved_stderr_fd = os.dup(stderr_fd)

    # Open /dev/null and redirect stderr to it
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, stderr_fd)
    os.close(devnull)

    try:
        yield
    finally:
        # Restore the original stderr
        os.dup2(saved_stderr_fd, stderr_fd)
        os.close(saved_stderr_fd)


def set_global_determinism(seed, torch_module=None):
    """
    Best-effort deterministic runtime configuration for a fixed seed.

    This resets Python, NumPy, and PyTorch RNG state immediately before
    stochastic training steps. `PYTHONHASHSEED` only affects new processes,
    but setting it here still documents the intended seed in subprocesses.
    """
    if seed is None:
        return

    seed = int(seed)
    os.environ['PYTHONHASHSEED'] = str(seed)
    random.seed(seed)
    np.random.seed(seed)

    torch_mod = torch_module if torch_module is not None else torch
    if torch_mod is None:
        return

    torch_mod.manual_seed(seed)
    if torch_mod.cuda.is_available():
        torch_mod.cuda.manual_seed_all(seed)
    try:
        torch_mod.use_deterministic_algorithms(True)
    except (AttributeError, RuntimeError, ValueError):
        pass
    try:
        torch_mod.backends.cudnn.deterministic = True
        torch_mod.backends.cudnn.benchmark = False
    except AttributeError:
        pass


def generate_random_training_seed():
    """Return a fresh non-negative 31-bit training seed."""
    from numpy.random import SeedSequence
    return int(SeedSequence().entropy % (2**31))


def resolve_training_seed(seed=None, trailing_newline=False, prefer_exported=True):
    """
    Resolve and print the seed used for a training/evaluation run.

    Priority when ``seed`` is omitted:
      1. seed embedded in the current exported model (if prefer_exported)
      2. a freshly generated random seed
    """
    suffix = "\n" if trailing_newline else ""
    if seed is not None:
        seed = int(seed)
        print(f"Using provided seed: {seed}{suffix}")
        return seed

    if prefer_exported:
        exported = read_exported_seed()
        if exported is not None:
            print(f"Using exported model seed: {exported}{suffix}")
            return int(exported)

    seed = generate_random_training_seed()
    if prefer_exported:
        print(f"No exported model seed found; generated random seed: {seed}{suffix}")
    else:
        print(f"Generated random seed: {seed}{suffix}")
    return seed


DEFAULT_FP_WEIGHT = 1.75


DEFAULT_BATCH_SIZE = 1024


DEFAULT_ARCHITECTURE_SWEEP = (
    {'name': 'Legacy (16-8)', 'layers': [16, 8]},
    {'name': 'Promoted default (24-12)', 'layers': [24, 12]},
    {'name': 'Shallow (24)', 'layers': [24]},
    {'name': 'Wider reference (32-16)', 'layers': [32, 16]},
    {'name': 'Deep (24-12-6)', 'layers': [24, 12, 6]},
)


DEFAULT_EXPERIMENT_OUTPUT = GENERATED_DATA_DIR / 'mlp_architecture_experiment.json'


DEFAULT_FP_WEIGHT_EXPERIMENT_OUTPUT = GENERATED_DATA_DIR / 'mlp_fp_weight_experiment.json'


DEFAULT_SEED_SEARCH_OUTPUT = GENERATED_DATA_DIR / 'mlp_seed_search.json'


DEFAULT_FP_WEIGHT_SWEEP = (1.0, 1.5, 1.75, 2.0, 2.5, 3.0)


DEFAULT_EXPERIMENT_SCREENING_SEED = 20260519


DEFAULT_EXPERIMENT_INITIAL_SEEDS = (20260518, 20260519, 20260520)


DEFAULT_EXPERIMENT_FINAL_SEEDS = (20260518, 20260519, 20260520, 20260521, 20260522)


DEFAULT_MAX_EPOCHS = 100


DEFAULT_EARLY_STOP_PATIENCE = 8


DEFAULT_LR_PATIENCE = 4


DEFAULT_PRIMARY_GROUP_KEY = 'lineage_group'


DEFAULT_CV_FOLDS = 3


DEFAULT_SHAP_BACKGROUND_SAMPLES = 100


def parse_hidden_layers(value):
    """Parse comma-separated hidden layer widths into a positive integer list."""
    if value is None:
        return None
    if isinstance(value, (list, tuple)):
        layers = [int(v) for v in value]
    else:
        text = str(value).strip()
        if not text:
            return None
        try:
            layers = [int(part.strip()) for part in text.split(',') if part.strip()]
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                "hidden layers must be a comma-separated list of integers, e.g. 24,12"
            ) from exc
    if not layers or any(layer <= 0 for layer in layers):
        raise argparse.ArgumentTypeError(
            "hidden layers must contain one or more positive integers"
        )
    return layers


def format_hidden_layers(layers):
    """Return hidden layers as a stable dash-separated string."""
    return '-'.join(str(int(layer)) for layer in layers)


def normalize_architecture_specs(architectures):
    """Normalize architecture definitions into {name, layers} dicts."""
    specs = []
    seen = set()
    for arch in architectures:
        if isinstance(arch, dict):
            layers = parse_hidden_layers(arch.get('layers'))
            name = str(arch.get('name') or f"MLP ({format_hidden_layers(layers)})")
        else:
            layers = parse_hidden_layers(arch)
            name = f"MLP ({format_hidden_layers(layers)})"
        key = tuple(layers)
        if key in seen:
            continue
        seen.add(key)
        specs.append({
            'name': name,
            'layers': list(layers),
        })
    return specs


def parse_architecture_sweep(value):
    """Parse semicolon-separated hidden-layer specs for --experiment-architectures."""
    if value is None:
        return None
    if isinstance(value, (list, tuple)):
        return normalize_architecture_specs(value)

    text = str(value).strip()
    if not text:
        return None

    specs = []
    for idx, chunk in enumerate(text.split(';'), start=1):
        item = chunk.strip()
        if not item:
            continue
        if '=' in item:
            name, layer_text = item.split('=', 1)
            layers = parse_hidden_layers(layer_text)
            specs.append({'name': name.strip() or f"MLP #{idx}", 'layers': layers})
        else:
            layers = parse_hidden_layers(item)
            specs.append({'name': f"MLP ({format_hidden_layers(layers)})", 'layers': layers})
    if not specs:
        raise argparse.ArgumentTypeError(
            "experiment architectures must contain one or more layer specs, e.g. 16,8;24,12;32,16"
        )
    return normalize_architecture_specs(specs)


def parse_fp_weight_sweep(value):
    """Parse a comma-separated, positive FP-weight sweep."""
    if value is None:
        return None
    if isinstance(value, (list, tuple)):
        values = [float(item) for item in value]
    else:
        try:
            values = [float(item.strip()) for item in str(value).split(',') if item.strip()]
        except ValueError as exc:
            raise argparse.ArgumentTypeError("FP weights must be comma-separated numbers") from exc
    if not values or any(value <= 0.0 for value in values):
        raise argparse.ArgumentTypeError("FP weights must contain one or more positive values")
    return list(dict.fromkeys(values))


def parse_positive_chip_boost(value):
    """
    Parse chip=multiplier pairs for motion-sample boosting.

    Example:
        ESP32=1.2,S3=1.1
    """
    if value is None:
        return None
    if isinstance(value, dict):
        boosts = {}
        for chip, factor in value.items():
            chip_name = str(chip).strip().upper()
            factor_value = float(factor)
            if not chip_name:
                raise argparse.ArgumentTypeError("chip name cannot be empty in positive chip boost")
            if factor_value <= 0.0:
                raise argparse.ArgumentTypeError("positive chip boost factors must be > 0")
            boosts[chip_name] = factor_value
        return boosts or None
    text = str(value).strip()
    if not text:
        return None

    boosts = {}
    for part in text.split(','):
        item = part.strip()
        if not item:
            continue
        if '=' not in item:
            raise argparse.ArgumentTypeError(
                "positive chip boost must use CHIP=FACTOR entries, e.g. ESP32=1.2"
            )
        chip, factor = item.split('=', 1)
        chip = chip.strip().upper()
        if not chip:
            raise argparse.ArgumentTypeError("chip name cannot be empty in positive chip boost")
        try:
            factor_value = float(factor.strip())
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"invalid boost factor for {chip!r}: {factor!r}"
            ) from exc
        if factor_value <= 0.0:
            raise argparse.ArgumentTypeError(
                "positive chip boost factors must be > 0"
            )
        boosts[chip] = factor_value
    return boosts or None


def build_model(hidden_layers=None, num_features=12, use_dropout=True, dropout_rate=0.2,
                seed=None):
    """
    Build a PyTorch MLP model.

    Dropout layers are added during training for regularization but are
    automatically disabled during inference (and don't affect exported weights).

    Args:
        hidden_layers: List of hidden layer sizes
        num_features: Number of input features
        use_dropout: Whether to add dropout layers (for training only)
        dropout_rate: Dropout rate (0.0-1.0)
        seed: Optional base seed for deterministic initializers

    Returns:
        TorchMLP instance
    """
    ensure_torch_available()
    return TorchMLP(
        num_features=num_features,
        hidden_layers=hidden_layers,
        use_dropout=use_dropout,
        dropout_rate=dropout_rate,
        seed=seed,
    )


def _compute_weighted_bce(logits, targets, sample_weights=None):
    """Binary cross-entropy on logits with optional per-sample weights."""
    losses = torch.nn.functional.binary_cross_entropy_with_logits(
        logits,
        targets,
        reduction='none',
    )
    if sample_weights is not None:
        losses = losses * sample_weights
    return losses.mean()


def train_model(X, y, hidden_layers=None, max_epochs=DEFAULT_MAX_EPOCHS, use_dropout=True,
                class_weight=None, fp_weight=DEFAULT_FP_WEIGHT, sample_weight=None,
                batch_size=DEFAULT_BATCH_SIZE, verbose=0, seed=None,
                feature_augmentation=None, feature_bounds=None):
    """
    Train a neural network model with best practices.

    Uses early stopping, learning rate reduction, dropout regularization,
    and optional class weighting for imbalanced datasets.

    Args:
        X: Feature matrix (normalized)
        y: Labels
        hidden_layers: List of hidden layer sizes
        max_epochs: Maximum training epochs (early stopping will cut short)
        use_dropout: Whether to add dropout layers
        class_weight: Class weight dict (e.g., {0: 1.0, 1: 2.0}) or None for auto
        fp_weight: Multiplier for class 0 (IDLE) weight to penalize false positives.
                   Values >1.0 make the model more conservative (fewer FP, lower recall).
        sample_weight: Optional per-sample weights
        batch_size: Mini-batch size for SGD/Adam updates
        verbose: Training verbosity
        seed: Optional base seed for deterministic training
        feature_augmentation: Optional normalized feature-space perturbation policy.
        feature_bounds: Optional normalized lower/upper bounds for augmented rows.

    Returns:
        Trained TorchMLP model
    """
    torch_mod = ensure_torch_available()

    if hidden_layers is None:
        hidden_layers = list(DEFAULT_HIDDEN_LAYERS)

    # Auto-compute class weights if not provided
    if class_weight is None:
        n_total = len(y)
        n_pos = np.sum(y == 1)
        n_neg = n_total - n_pos
        if n_pos > 0 and n_neg > 0:
            # Balanced class weights: higher weight for minority class
            class_weight = {
                0: n_total / (2 * n_neg),
                1: n_total / (2 * n_pos)
            }

    # Apply FP penalty: increase weight for class 0 (IDLE)
    # This makes misclassifying baseline as motion more costly
    if fp_weight != 1.0 and class_weight is not None:
        class_weight[0] *= fp_weight

    # Merge class weights into sample_weight when both are requested so the
    # optimizer sees a single per-sample weighting term.
    if sample_weight is not None and class_weight is not None:
        sample_weight = np.asarray(sample_weight, dtype=np.float32).copy()
        class_multiplier = np.where(np.asarray(y) == 1, class_weight[1], class_weight[0])
        sample_weight *= class_multiplier.astype(np.float32)
        class_weight = None

    # Determine number of features from input shape
    X = np.asarray(X, dtype=np.float32)
    y = np.asarray(y, dtype=np.float32)
    num_features = X.shape[1] if hasattr(X, 'shape') else len(X[0])
    set_global_determinism(seed, torch_module=torch_mod)
    device = resolve_torch_device(torch_module=torch_mod)
    model = build_model(
        hidden_layers=hidden_layers,
        num_features=num_features,
        use_dropout=use_dropout,
        seed=seed,
    ).to(device)

    # Keep a stratified validation split instead of relying on implicit slicing.
    from sklearn.model_selection import train_test_split as _val_split
    split_kwargs = dict(test_size=0.1, random_state=42, stratify=np.asarray(y))
    if sample_weight is not None:
        sample_weight = np.asarray(sample_weight, dtype=np.float32)
        X_t, X_v, y_t, y_v, sw_t, sw_v = _val_split(
            X, y, sample_weight, **split_kwargs
        )
    else:
        X_t, X_v, y_t, y_v = _val_split(X, y, **split_kwargs)
        sw_t, sw_v = None, None

    optimizer = torch.optim.Adam(model.parameters())
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer,
        mode='min',
        factor=0.5,
        patience=DEFAULT_LR_PATIENCE,
        min_lr=1e-6,
    )

    X_t_tensor = torch.from_numpy(np.asarray(X_t, dtype=np.float32)).to(device)
    y_t_tensor = torch.from_numpy(np.asarray(y_t, dtype=np.float32)).view(-1, 1).to(device)
    X_v_tensor = torch.from_numpy(np.asarray(X_v, dtype=np.float32)).to(device)
    y_v_tensor = torch.from_numpy(np.asarray(y_v, dtype=np.float32)).view(-1, 1).to(device)
    sw_t_tensor = (
        None
        if sw_t is None
        else torch.from_numpy(np.asarray(sw_t, dtype=np.float32)).view(-1, 1).to(device)
    )
    sw_v_tensor = (
        None
        if sw_v is None
        else torch.from_numpy(np.asarray(sw_v, dtype=np.float32)).view(-1, 1).to(device)
    )

    best_state = copy.deepcopy(model.state_dict())
    best_val_loss = float('inf')
    epochs_without_improvement = 0
    batch_size = max(1, int(batch_size))

    for epoch in range(int(max_epochs)):
        model.train()
        for start in range(0, len(X_t_tensor), batch_size):
            stop = start + batch_size
            if feature_augmentation:
                augmented = augment_normalized_features(
                    X_t[start:stop],
                    feature_augmentation,
                    derive_seed(seed, epoch, start),
                    bounds=feature_bounds,
                )
                batch_x = torch.from_numpy(augmented).to(device)
            else:
                batch_x = X_t_tensor[start:stop]
            batch_y = y_t_tensor[start:stop]
            batch_weights = None
            if sw_t_tensor is not None:
                batch_weights = sw_t_tensor[start:stop].clone()
            if class_weight is not None:
                class_multiplier = torch.where(
                    batch_y > 0.5,
                    float(class_weight[1]),
                    float(class_weight[0]),
                )
                batch_weights = class_multiplier if batch_weights is None else batch_weights * class_multiplier

            optimizer.zero_grad()
            logits = model.forward_logits(batch_x)
            loss = _compute_weighted_bce(logits, batch_y, sample_weights=batch_weights)
            loss.backward()
            optimizer.step()

        model.eval()
        with torch.no_grad():
            val_logits = model.forward_logits(X_v_tensor)
            val_loss = _compute_weighted_bce(
                val_logits,
                y_v_tensor,
                sample_weights=sw_v_tensor,
            ).item()
        scheduler.step(val_loss)

        if verbose:
            print(
                f"    epoch {epoch + 1:03d}/{max_epochs}: "
                f"val_loss={val_loss:.6f} lr={optimizer.param_groups[0]['lr']:.2e}"
            )

        if val_loss < (best_val_loss - 1e-4):
            best_val_loss = val_loss
            best_state = copy.deepcopy(model.state_dict())
            epochs_without_improvement = 0
        else:
            epochs_without_improvement += 1
            if epochs_without_improvement >= DEFAULT_EARLY_STOP_PATIENCE:
                break

    model.load_state_dict(best_state)
    model.eval()

    return model


def cross_validate(X, y, hidden_layers=None, n_folds=DEFAULT_CV_FOLDS, max_epochs=DEFAULT_MAX_EPOCHS,
                   fp_weight=DEFAULT_FP_WEIGHT, sample_weight=None, groups=None,
                   sample_context=None, scaler_mode=DEFAULT_SCALER_MODE,
                   batch_size=DEFAULT_BATCH_SIZE, block_stride=1,
                   block_group_key=DEFAULT_BLOCK_GROUP_KEY,
                   report_group_keys=DEFAULT_REPORT_GROUP_KEYS, seed=None,
                   shap_samples=0, shap_feature_names=None, shap_seed=None,
                   feature_augmentation=None, X_aug=None, y_aug=None, groups_aug=None):
    """
    Perform grouped cross-validation with de-overlapped scoring.

    Args:
        X: Feature matrix (NOT normalized - scaler fit per fold)
        y: Labels
        hidden_layers: List of hidden layer sizes
        n_folds: Number of CV folds
        max_epochs: Maximum training epochs per fold
        fp_weight: Multiplier for class 0 weight (>1.0 penalizes FP more)
        sample_weight: Optional per-sample weights aligned with X/y
        groups: Optional split-group labels per sample
        sample_context: Optional aligned metadata for reporting/blocking
        scaler_mode: Feature normalization mode
        batch_size: Mini-batch size used for fold training
        block_stride: Subsampling stride applied at scoring time
        block_group_key: Group key used for block subsampling
        report_group_keys: Extra group reports to compute from OOF predictions
        seed: Optional base seed for deterministic per-fold training
        shap_samples: Total held-out samples to explain across all folds
        shap_feature_names: Feature names aligned with the columns in X
        shap_seed: Optional deterministic seed for SHAP sampling
        feature_augmentation: Optional train-time normalized feature perturbation
        X_aug: Optional packet-augmented feature matrix (train-only)
        y_aug: Labels aligned with X_aug
        groups_aug: Split-group labels aligned with X_aug

    Returns:
        dict: Mean and std of each metric across folds
    """
    if hidden_layers is None:
        hidden_layers = list(DEFAULT_HIDDEN_LAYERS)
    feature_augmentation = dict(feature_augmentation or {})
    feature_names = list(shap_feature_names) if shap_feature_names else None

    if groups is not None:
        from sklearn.model_selection import StratifiedGroupKFold
        unique_groups = len(set(groups))
        effective_folds = min(n_folds, unique_groups)
        splitter = StratifiedGroupKFold(n_splits=effective_folds, shuffle=True, random_state=42)
        split_iter = splitter.split(X, y, groups)
    else:
        from sklearn.model_selection import StratifiedKFold
        effective_folds = n_folds
        splitter = StratifiedKFold(n_splits=effective_folds, shuffle=True, random_state=42)
        split_iter = splitter.split(X, y)

    fold_metrics = []
    oof_prob = np.full(len(y), np.nan, dtype=np.float32)
    scored_mask = np.zeros(len(y), dtype=bool)
    fold_timings = []
    cv_start = perf_counter()
    shap_abs_sum = np.zeros(X.shape[1], dtype=np.float64)
    shap_count = 0
    shap_module = None
    fold_shap_counts = distribute_samples(shap_samples, effective_folds)
    if shap_samples > 0:
        try:
            import shap as shap_module
        except ImportError:
            print("Error: SHAP not installed. Run: pip install shap")

    for fold, (train_idx, val_idx) in enumerate(split_iter):
        fold_start = perf_counter()
        X_train_fold, X_val_fold = X[train_idx], X[val_idx]
        y_train_fold, y_val_fold = y[train_idx], y[val_idx]
        sw_train_fold = sample_weight[train_idx] if sample_weight is not None else None

        # Fit normalization only on the training fold
        preprocess_start = perf_counter()
        scaler = build_preprocessor(scaler_mode)
        train_context = slice_sample_context(sample_context, train_idx)
        fit_preprocessor(
            scaler, X_train_fold, y=y_train_fold, sample_context=train_context)
        X_train_scaled = scaler.transform(X_train_fold)
        X_val_scaled = scaler.transform(X_val_fold)
        # SHAP must describe clean, held-out deployment windows while the model
        # retains the promoted train-time augmentation recipe. Keep an explicit
        # view of the clean fold before packet-augmented rows are appended.
        X_train_clean_scaled = X_train_scaled
        y_train_clean = y_train_fold
        if groups is not None:
            train_groups = np.asarray(groups)[train_idx]
        else:
            train_groups = np.arange(len(train_idx))
        X_train_scaled, y_train_fold, sw_train_fold = _append_augmented_training_rows(
            X_train_scaled,
            y_train_fold,
            scaler,
            X_aug,
            y_aug,
            groups_aug,
            train_groups,
            sample_weight=sw_train_fold,
        )
        feature_bounds = None
        if feature_augmentation and feature_names is not None:
            feature_bounds = normalized_feature_bounds(scaler, feature_names)
        preprocess_elapsed = perf_counter() - preprocess_start

        train_predict_start = perf_counter()
        fold_seed = derive_seed(seed, fold)
        with suppress_stderr():
            model = train_model(X_train_scaled, y_train_fold,
                                hidden_layers=hidden_layers, max_epochs=max_epochs,
                                fp_weight=fp_weight, sample_weight=sw_train_fold,
                                batch_size=batch_size, seed=fold_seed,
                                feature_augmentation=feature_augmentation or None,
                                feature_bounds=feature_bounds)
            val_prob = predict_probabilities(model, X_val_scaled)
        train_predict_elapsed = perf_counter() - train_predict_start

        oof_prob[val_idx] = val_prob
        scoring_start = perf_counter()
        val_context = slice_sample_context(sample_context, val_idx)
        local_mask = build_block_mask(
            val_context,
            stride=block_stride,
            group_key=block_group_key,
        )
        if local_mask is None:
            local_mask = np.ones(len(val_idx), dtype=bool)

        scored_idx = val_idx[local_mask]
        scored_mask[scored_idx] = True
        metrics = evaluate_probabilities(y_val_fold[local_mask], val_prob[local_mask])
        fold_metrics.append(metrics)

        requested_fold_samples = fold_shap_counts[fold]
        if shap_module is not None and requested_fold_samples > 0:
            train_context = slice_sample_context(sample_context, train_idx)
            background_idx = select_balanced_shap_indices(
                y_train_clean,
                train_context,
                DEFAULT_SHAP_BACKGROUND_SAMPLES,
                derive_seed(shap_seed, fold, 1),
            )
            scored_local_idx = np.flatnonzero(local_mask)
            scored_context = slice_sample_context(val_context, scored_local_idx)
            explain_scored_idx = select_balanced_shap_indices(
                y_val_fold[scored_local_idx],
                scored_context,
                requested_fold_samples,
                derive_seed(shap_seed, fold, 2),
            )
            explain_local_idx = scored_local_idx[explain_scored_idx]
            shap_values = calculate_shap_values(
                model,
                X_train_clean_scaled[background_idx],
                X_val_scaled[explain_local_idx],
                shap_module=shap_module,
                seed=derive_seed(shap_seed, fold, 3),
            )
            if shap_values is not None:
                shap_abs_sum += np.sum(np.abs(shap_values), axis=0)
                shap_count += len(shap_values)
                print(
                    f"  Fold {fold + 1}/{effective_folds} SHAP: "
                    f"explained {len(shap_values)} held-out samples"
                )
        scoring_elapsed = perf_counter() - scoring_start
        fold_elapsed = perf_counter() - fold_start
        fold_timings.append(fold_elapsed)
        print(
            f"  Fold {fold + 1}/{effective_folds} timing: "
            f"preprocess={format_duration(preprocess_elapsed)}, "
            f"train+predict={format_duration(train_predict_elapsed)}, "
            f"score={format_duration(scoring_elapsed)}, "
            f"total={format_duration(fold_elapsed)}"
        )

    # Aggregate
    result = {}
    for key in fold_metrics[0]:
        values = [m[key] for m in fold_metrics]
        result[f'{key}_mean'] = np.mean(values)
        result[f'{key}_std'] = np.std(values)

    scored_idx = np.flatnonzero(scored_mask)
    oof_metrics = evaluate_probabilities(y[scored_idx], oof_prob[scored_idx])
    for key, value in oof_metrics.items():
        result[f'oof_{key}'] = value

    result['n_folds'] = len(fold_metrics)
    result['scored_samples'] = int(len(scored_idx))
    result['dense_samples'] = int(np.sum(~np.isnan(oof_prob)))
    result['scaler_mode'] = scaler_mode
    result['timings'] = {
        'fold_seconds': fold_timings,
        'total_seconds': perf_counter() - cv_start,
    }
    if shap_count > 0:
        feature_names = shap_feature_names or [f'feature_{idx}' for idx in range(X.shape[1])]
        mean_abs_shap = shap_abs_sum / shap_count
        result['shap_importance'] = dict(sorted(
            ((name, float(value)) for name, value in zip(feature_names, mean_abs_shap, strict=True)),
            key=lambda item: item[1],
            reverse=True,
        ))
        result['shap_samples'] = shap_count

    if sample_context is not None and report_group_keys:
        scored_context = slice_sample_context(sample_context, scored_idx)
        group_reports = {}
        for group_key in report_group_keys:
            report = build_group_report(
                y[scored_idx],
                oof_prob[scored_idx],
                scored_context.get(group_key),
            )
            if report is not None:
                group_reports[group_key] = report
        # Provenance-split session reports: synthetic derivatives may stress the
        # model, but they must not mask (or fake) movement in the real-session
        # worst/tail metrics that lead promotion.
        synthetic_flags = np.asarray(
            scored_context.get('synthetic', ()), dtype=bool)
        session_values = scored_context.get('session_group')
        if session_values is not None and synthetic_flags.size and synthetic_flags.any():
            for provenance_key, provenance_mask in (
                ('real_session_group', ~synthetic_flags),
                ('synthetic_session_group', synthetic_flags),
            ):
                provenance_idx = np.flatnonzero(provenance_mask)
                if provenance_idx.size == 0:
                    continue
                report = build_group_report(
                    y[scored_idx][provenance_idx],
                    oof_prob[scored_idx][provenance_idx],
                    np.asarray(session_values)[provenance_idx],
                )
                if report is not None:
                    group_reports[provenance_key] = report
        result['group_reports'] = group_reports

    return result


def leave_one_group_out_validation(group_key, unit, detail_group_key,
                                   skip_values=(), fp_weight=DEFAULT_FP_WEIGHT,
                                   seed=None, feature_names=None, hidden_layers=None,
                                   scaler_mode=DEFAULT_SCALER_MODE,
                                   batch_size=DEFAULT_BATCH_SIZE,
                                   excluded_chips=None,
                                   block_stride=DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
                                   use_cache=True, augment=False):
    """Leave-one-group-out generalization check over a sample-context grouping.

    For each value of ``group_key`` (a room, a chip, ...), train on all other
    values and evaluate on the held-out one. This measures how well the detector
    transfers to a group it never saw during training. Grouped CV can still mix
    groups across folds, so it tends to be optimistic about cross-group
    generalization; this routine removes that leakage by making the group the
    split boundary.

    Args:
        group_key: Sample-context key defining the held-out unit (e.g.
            ``environment_group`` or ``chip``).
        unit: Human-readable singular noun for the group (e.g. ``environment``).
        detail_group_key: Secondary sample-context key reported as the worst
            sub-group inside each held-out fold (e.g. ``chip`` for rooms).
        skip_values: Group values treated as missing metadata and ignored.
        augment: Optional augmentation component set for the held-out runs.

    This is a diagnostic only: it never trains a promotable model or exports
    runtime artifacts. Held-out scoring reuses the same block subsampling as
    grouped CV so the numbers stay comparable to the trainer's own report.
    """
    if hidden_layers is None:
        hidden_layers = list(DEFAULT_HIDDEN_LAYERS)
    if feature_names is None:
        feature_names = DEFAULT_FEATURES.copy()
    feature_names = list(feature_names)
    excluded_chips = parse_chip_filter(excluded_chips)
    skip_values = {str(v) for v in skip_values}
    augment_components, feature_augmentation, packet_augmentation = resolve_training_augmentation(augment)

    try:
        ensure_torch_available()
        torch_device_label = describe_torch_device()
        seed = resolve_training_seed(seed, trailing_newline=True)
        set_global_determinism(seed, torch_module=torch)
    except ImportError as exc:
        print(f"Error: Missing dependency - {exc}")
        print("Install with: pip install torch scikit-learn")
        return 1
    except (RuntimeError, ValueError) as exc:
        print(f"Error: {exc}")
        return 1

    print("\n" + "=" * 70)
    print(f"  LEAVE-ONE-{unit.upper()}-OUT GENERALIZATION CHECK")
    print("=" * 70)
    print(f"FP weight: {fp_weight}")
    print(f"Scaler: {scaler_mode}")
    print(f"Batch size: {batch_size}")
    print(f"Architecture: {' -> '.join(map(str, [len(feature_names)] + hidden_layers + [1]))}")
    print(
        "Augmentation: "
        f"{format_augmentation_config(feature_augmentation, packet_augmentation, components=augment_components)}"
    )
    if packet_augmentation:
        print(
            "Packet augmentation seeds: "
            + ", ".join(str(seed) for seed in FIXED_PACKET_AUGMENTATION_SEEDS)
        )
    print(f"Torch device: {torch_device_label}")
    if excluded_chips is not None:
        print(f"Excluded chips: {', '.join(sorted(excluded_chips))}")

    print("\nLoading training matrix...")
    matrix, _all_packets = load_training_matrix(
        environment_filter=None,
        excluded_chips=excluded_chips,
        feature_names=feature_names,
        use_cache=use_cache,
    )
    X = matrix['X']
    y = matrix['y']
    sample_context = matrix['sample_context']
    X_aug = y_aug = groups_aug = None
    if packet_augmentation:
        print("Loading packet-augmented training matrix...")
        aug_matrix, _ = load_training_matrix(
            environment_filter=None,
            excluded_chips=excluded_chips,
            feature_names=feature_names,
            use_cache=use_cache,
            packet_augmentation=packet_augmentation,
            augmentation_seeds=training_packet_augmentation_seeds(
                packet_augmentation
            ),
        )
        X_aug = aug_matrix['X']
        y_aug = aug_matrix['y']
        groups_aug = aug_matrix['sample_context'].get(group_key)

    group_values = sample_context.get(group_key)
    if group_values is None or len(group_values) == 0:
        print(f"Error: no {unit} metadata available for cross-{unit} CV")
        return 1
    group_values = np.asarray([str(v) for v in group_values])

    groups = sorted(
        name for name in set(group_values.tolist())
        if name and name not in skip_values
    )
    if len(groups) < 2:
        print(
            f"Error: need at least 2 named {unit} groups, found {len(groups)}: "
            f"{', '.join(groups) or 'none'}"
        )
        return 1

    skipped_count = int(np.sum(np.isin(group_values, list(skip_values)))) if skip_values else 0
    print(f"  Named {unit} groups: {', '.join(groups)}")
    if skipped_count:
        print(f"  Windows without {unit} metadata (ignored): {skipped_count}")

    label_values = np.asarray(sample_context.get(
        'label_name', np.full(len(y), 'unknown')))

    fold_rows = []
    for held_out in groups:
        test_mask = group_values == held_out
        train_mask = np.isin(group_values, groups) & ~test_mask

        n_train = int(np.sum(train_mask))
        n_test = int(np.sum(test_mask))
        test_pos = int(np.sum(y[test_mask] == 1))
        test_neg = n_test - test_pos
        train_groups = [name for name in groups if name != held_out]

        print("\n" + "-" * 70)
        print(f"Held-out {unit}: {held_out}")
        print(f"  Train on: {', '.join(train_groups)} ({n_train} windows)")
        print(f"  Test on:  {held_out} ({n_test} windows: {test_pos} motion, {test_neg} idle)")

        if test_pos == 0 or test_neg == 0:
            print(f"  Skipped: held-out {unit} lacks both motion and idle windows")
            continue
        if n_train == 0:
            print(f"  Skipped: no training windows for the remaining {unit} groups")
            continue

        scaler = build_preprocessor(scaler_mode)
        train_context = slice_sample_context(sample_context, np.flatnonzero(train_mask))
        fit_preprocessor(
            scaler, X[train_mask], y=y[train_mask], sample_context=train_context)
        X_train_scaled = scaler.transform(X[train_mask])
        X_test_scaled = scaler.transform(X[test_mask])
        X_train_scaled, y_train_fold, _ = _append_augmented_training_rows(
            X_train_scaled,
            y[train_mask],
            scaler,
            X_aug,
            y_aug,
            groups_aug,
            train_groups,
        )
        feature_bounds = None
        if feature_augmentation:
            feature_bounds = normalized_feature_bounds(scaler, feature_names)

        fold_seed = derive_seed(seed, _stable_text_seed(held_out))
        with suppress_stderr():
            model = train_model(
                X_train_scaled, y_train_fold,
                hidden_layers=hidden_layers,
                fp_weight=fp_weight,
                batch_size=batch_size,
                seed=fold_seed,
                feature_augmentation=feature_augmentation or None,
                feature_bounds=feature_bounds,
            )
            test_prob = predict_probabilities(model, X_test_scaled)

        test_context = slice_sample_context(sample_context, np.flatnonzero(test_mask))
        block_mask = build_block_mask(
            test_context, stride=block_stride, group_key=DEFAULT_BLOCK_GROUP_KEY)
        if block_mask is None:
            block_mask = np.ones(n_test, dtype=bool)

        y_test = y[test_mask]
        metrics = evaluate_probabilities(y_test[block_mask], test_prob[block_mask])
        detail_values = test_context.get(detail_group_key)
        detail_report = build_group_report(
            y_test[block_mask], test_prob[block_mask],
            np.asarray(detail_values)[block_mask] if detail_values is not None else None,
        )

        # False-positive breakdown by idle sub-type on the held-out group.
        test_labels = label_values[test_mask][block_mask]
        idle_breakdown = {}
        for idle_label in ('empty', 'static_presence'):
            idle_mask = test_labels == idle_label
            n_idle = int(np.sum(idle_mask))
            if n_idle == 0:
                continue
            idle_metrics = evaluate_probabilities(
                y_test[block_mask][idle_mask], test_prob[block_mask][idle_mask])
            idle_breakdown[idle_label] = (n_idle, idle_metrics['fp_rate'])

        worst_detail = detail_report.get('worst_recall') if detail_report else None
        detail_noun = detail_group_key.replace('_group', '')
        print(
            f"  Recall={metrics['recall']:.1f}%  FP={metrics['fp_rate']:.1f}%  "
            f"Precision={metrics['precision']:.1f}%  F1={metrics['f1']:.1f}%  "
            f"(scored {int(np.sum(block_mask))} windows)"
        )
        for idle_label, (n_idle, fp_rate) in idle_breakdown.items():
            print(f"    {idle_label} FP: {fp_rate:.1f}% ({n_idle} windows)")
        if worst_detail:
            print(
                f"    worst {detail_noun} recall: {worst_detail['group']} "
                f"{worst_detail['recall']:.1f}% (FP {worst_detail['fp_rate']:.1f}%)"
            )

        fold_rows.append({
            'group': held_out,
            'train_windows': n_train,
            'test_windows': int(np.sum(block_mask)),
            'test_motion': test_pos,
            'test_idle': test_neg,
            **metrics,
            'idle_breakdown': idle_breakdown,
            'worst_detail': worst_detail,
        })

    if not fold_rows:
        print(f"\nNo {unit} could be evaluated as a held-out fold.")
        return 1

    print("\n" + "=" * 70)
    print(f"  SUMMARY (each row: model never saw that {unit} during training)")
    print("=" * 70)
    header = f"{unit:<16}{'recall':>9}{'fp':>8}{'prec':>8}{'f1':>8}{'test_win':>10}"
    print(header)
    print("-" * len(header))
    for row in fold_rows:
        print(
            f"{row['group']:<16}{row['recall']:>8.1f}%{row['fp_rate']:>7.1f}%"
            f"{row['precision']:>7.1f}%{row['f1']:>7.1f}%{row['test_windows']:>10}"
        )
    macro_recall = float(np.mean([r['recall'] for r in fold_rows]))
    macro_fp = float(np.mean([r['fp_rate'] for r in fold_rows]))
    macro_f1 = float(np.mean([r['f1'] for r in fold_rows]))
    worst_recall = min(fold_rows, key=lambda r: r['recall'])
    worst_fp = max(fold_rows, key=lambda r: r['fp_rate'])
    print("-" * len(header))
    print(
        f"{'macro-average':<16}{macro_recall:>8.1f}%{macro_fp:>7.1f}%"
        f"{'':>7}{macro_f1:>7.1f}%"
    )
    print(f"\nWorst held-out recall: {worst_recall['group']} {worst_recall['recall']:.1f}%")
    print(f"Worst held-out FP rate: {worst_fp['group']} {worst_fp['fp_rate']:.1f}%")
    print("\nRuntime artifacts unchanged (diagnostic run).")
    return 0


def cross_environment_validation(**kwargs):
    """Leave-one-environment-out generalization check (train on other rooms)."""
    return leave_one_group_out_validation(
        group_key='environment_group',
        unit='environment',
        detail_group_key='chip',
        skip_values=('unknown-environment',),
        **kwargs,
    )


def cross_chip_validation(**kwargs):
    """Leave-one-chip-out generalization check (train on other chips)."""
    return leave_one_group_out_validation(
        group_key='chip',
        unit='chip',
        detail_group_key='environment_group',
        skip_values=('', 'unknown', 'UNKNOWN', 'unknown-chip'),
        **kwargs,
    )


def calculate_correlation_importance(feature_names=None, use_cache=True):
    """
    Calculate correlation of selected training features with motion label.

    This is a fast alternative to SHAP for initial feature screening.
    Reuses the canonical time-aware training matrix.

    Args:
        feature_names: Optional list of features to analyze (default: TRAINING_FEATURES)

    Returns:
        dict: {feature_name: correlation} sorted by absolute correlation
    """
    if feature_names is None:
        feature_names = list(TRAINING_FEATURES)

    print("\nCalculating feature correlations...")
    print(f"  Analyzing {len(feature_names)} features")

    matrix, _ = load_training_matrix(
        feature_names=feature_names,
        use_cache=use_cache,
    )
    stats = matrix['stats']
    print(f"  Loaded {stats['total']} packets")

    X = matrix['X']
    y = matrix['y']
    actual_features = matrix['feature_names']
    print(f"  Extracted features for {len(X)} samples")

    # Calculate correlations for each feature column
    correlations = {}
    for i, fname in enumerate(actual_features):
        corr = np.corrcoef(X[:, i], y)[0, 1]
        if not np.isnan(corr):
            correlations[fname] = corr

    # Sort by absolute correlation
    sorted_corr = dict(sorted(correlations.items(), key=lambda x: abs(x[1]), reverse=True))

    print_candidate_redundancy(X, actual_features)

    return sorted_corr


def print_candidate_redundancy(X, feature_names, baseline_features=None):
    """Report how much of each candidate the production set already explains.

    A candidate earns its place by what it adds, not by how well it separates on
    its own, so the screening question is redundancy: its strongest pairwise
    correlation against the production members, and the share of its variance a
    least-squares fit on all of them removes.
    """
    if baseline_features is None:
        baseline_features = DEFAULT_FEATURES
    names = list(feature_names)
    baseline_index = [i for i, name in enumerate(names) if name in baseline_features]
    candidate_index = [i for i, name in enumerate(names) if name not in baseline_features]
    if not candidate_index or not baseline_index:
        return

    design = np.column_stack(
        [X[:, baseline_index], np.ones(len(X), dtype=X.dtype)]
    )
    print("\n" + "=" * 74)
    print("  Candidate Redundancy Against The Production Set")
    print("=" * 74)
    print(f"{'Candidate':<22} {'max |r| vs production':>22} {'closest':>16} {'R2':>8}")
    print("-" * 74)
    for i in candidate_index:
        values = X[:, i]
        strongest, closest = 0.0, "-"
        for j in baseline_index:
            if values.std() < 1e-12 or X[:, j].std() < 1e-12:
                continue
            r = abs(float(np.corrcoef(values, X[:, j])[0, 1]))
            if r > strongest:
                strongest, closest = r, names[j]
        coefficients, *_ = np.linalg.lstsq(design, values, rcond=None)
        residual = values - design @ coefficients
        variance = float(values.var())
        r_squared = 1.0 - float(residual.var()) / variance if variance > 0 else 0.0
        print(f"{names[i]:<22} {strongest:>22.4f} {closest:>16} {r_squared:>8.4f}")
    print("-" * 74)
    print("  Lower is better: a candidate the production set can reconstruct "
          "adds nothing.")


def print_correlation_table(correlations, current_features=None):
    """Print correlation results in a nice table."""
    from tools.lib.csi_features import DEFAULT_FEATURES

    if current_features is None:
        current_features = DEFAULT_FEATURES

    print("\n" + "=" * 74)
    print("  Feature Correlation with Motion Label")
    print("=" * 74)
    print(f"{'Rank':<5} {'Feature':<22} {'Corr':>8} {'|Corr|':>8} {'Status':<12}")
    print("-" * 74)

    for rank, (fname, corr) in enumerate(correlations.items(), 1):
        status = "USED" if fname in current_features else ""
        bar = '█' * int(abs(corr) * 20)
        print(f"{rank:<5} {fname:<22} {corr:>+8.4f} {abs(corr):>8.4f} {status:<12} {bar}")

    print("-" * 74)

    # Recommendations
    print("\nRecommendations:")
    sorted_items = list(correlations.items())
    top_unused = [(f, c) for f, c in sorted_items if f not in current_features][:3]
    if top_unused:
        print(f"  Top unused features: {', '.join(f[0] for f in top_unused)}")

    low_used = [(f, c) for f, c in sorted_items if f in current_features and abs(c) < 0.2]
    if low_used:
        print(f"  Low correlation but used: {', '.join(f[0] for f in low_used)}")


def calculate_shap_values(model, background, X_explain, shap_module=None, seed=None):
    """Calculate SHAP values for explicitly separated background and explain sets."""
    if shap_module is None:
        try:
            import shap as shap_module
        except ImportError:
            print("Error: SHAP not installed. Run: pip install shap")
            return None

    if len(background) == 0 or len(X_explain) == 0:
        return None

    explainer = shap_module.Explainer(
        lambda values: predict_probabilities(model, values).reshape(-1, 1),
        background,
        algorithm='permutation',
        seed=seed,
    )

    with suppress_stderr():
        shap_values = explainer(X_explain).values

    if isinstance(shap_values, list):
        shap_values = shap_values[0]
    shap_values = np.asarray(shap_values)
    while shap_values.ndim > 2 and shap_values.shape[-1] == 1:
        shap_values = np.squeeze(shap_values, axis=-1)
    if shap_values.ndim == 1:
        shap_values = shap_values.reshape(len(X_explain), -1)
    return shap_values


def print_feature_importance(importance, title="Feature Importance (SHAP)",
                             current_features=None):
    """
    Print feature importance table with visual bars.

    Args:
        importance: Dict of {feature_name: importance_value}
        title: Title for the table
        current_features: Optional list of features currently in use (to mark USED)
    """
    print(f"\n{'='*78}")
    print(f"  {title}")
    print(f"{'='*78}\n")

    total = sum(importance.values())
    if total < 1e-10:
        print("  No importance values calculated.\n")
        return

    if current_features:
        print(f"{'Rank':<5} {'Feature':<22} {'SHAP':>8} {'Contrib':>8} {'Status':<8}")
        print("-" * 78)
    else:
        print(f"{'Rank':<6} {'Feature':<22} {'SHAP Value':>12} {'Contribution':>14}")
        print("-" * 70)

    for rank, (name, value) in enumerate(importance.items(), 1):
        pct = (value / total * 100)
        bar_len = int(pct / 2.5)  # Scale to ~40 chars max
        bar = '█' * bar_len
        if current_features:
            status = "USED" if name in current_features else ""
            print(f"{rank:<5} {name:<22} {value:>8.4f} {pct:>7.1f}% {status:<8} {bar}")
        else:
            print(f"{rank:<6} {name:<22} {value:>12.6f} {pct:>8.1f}% {bar}")

    if current_features:
        print("-" * 78)
    else:
        print("-" * 70)
        print(f"{'':6} {'TOTAL':<22} {total:>12.6f} {'100.0%':>14}")
    print()

    # Recommendations
    sorted_features = list(importance.keys())
    low_importance = [f for f in sorted_features if importance[f] / total < 0.03]
    high_importance = [f for f in sorted_features[:3]]

    print("Recommendations:")
    print(f"  Most important: {', '.join(high_importance)}")
    if low_importance:
        print(f"  Low importance (<3%): {', '.join(low_importance)}")

    if current_features:
        # Show top unused and low-importance used features
        top_unused = [f for f in sorted_features[:10] if f not in current_features]
        low_used = [f for f in sorted_features if f in current_features
                    and importance[f] / total < 0.05]
        if top_unused:
            print(f"  Top unused features: {', '.join(top_unused[:5])}")
        if low_used:
            print(f"  Low importance but USED: {', '.join(low_used)}")
    print()


def run_ablation_study(X, y, feature_names, sample_context=None, sample_weight=None,
                       hidden_layers=None, fp_weight=DEFAULT_FP_WEIGHT,
                       scaler_mode=DEFAULT_SCALER_MODE,
                       batch_size=DEFAULT_BATCH_SIZE):
    """
    Run ablation study: train model removing one feature at a time.

    This helps identify which features are truly important by measuring
    the impact of removing each one. Features whose removal improves or
    doesn't affect F1 are candidates for elimination.

    Args:
        X: Feature matrix (NOT normalized - scaler fit per fold)
        y: Labels
        feature_names: List of feature names
        sample_context: Optional aligned metadata for grouped CV
        sample_weight: Optional per-sample weights
        hidden_layers: Model architecture
        fp_weight: FP penalty weight
        scaler_mode: Feature normalization mode
        batch_size: Mini-batch size used during fold training

    Returns:
        list: Results for each ablation experiment
    """
    print("\n" + "="*80)
    print("                         ABLATION STUDY")
    print("="*80 + "\n")
    print("Training models with one feature removed at a time to measure impact...\n")

    if hidden_layers is None:
        hidden_layers = list(DEFAULT_HIDDEN_LAYERS)

    groups = None
    if sample_context is not None:
        groups = sample_context.get(DEFAULT_PRIMARY_GROUP_KEY)

    results = []

    # Baseline (all features)
    print(f"[1/{len(feature_names)+1}] Baseline (all {len(feature_names)} features)...")
    with suppress_stderr():
        static_presence_cv = cross_validate(
            X, y,
            hidden_layers=hidden_layers,
            n_folds=DEFAULT_CV_FOLDS,
            max_epochs=DEFAULT_MAX_EPOCHS,
            fp_weight=fp_weight,
            sample_weight=sample_weight,
            groups=groups,
            sample_context=sample_context,
            scaler_mode=scaler_mode,
            batch_size=batch_size,
            block_stride=DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
        )
    static_presence_f1 = static_presence_cv['f1_mean']
    results.append({
        'removed': 'None (baseline)',
        'n_features': len(feature_names),
        'f1_mean': static_presence_f1,
        'f1_std': static_presence_cv['f1_std'],
        'oof_f1': static_presence_cv['oof_f1'],
        'recall_mean': static_presence_cv['recall_mean'],
        'fp_rate_mean': static_presence_cv['fp_rate_mean'],
        'delta_f1': 0.0,
    })
    print(
        f"    F1: {static_presence_f1:.2f}% (+/- {static_presence_cv['f1_std']:.2f}%), "
        f"blocked OOF={static_presence_cv['oof_f1']:.2f}%\n"
    )

    # Remove each feature one at a time
    for i, feature_name in enumerate(feature_names):
        print(f"[{i+2}/{len(feature_names)+1}] Removing '{feature_name}'...")

        # Create X without this feature
        X_ablated = np.delete(X, i, axis=1)

        with suppress_stderr():
            cv = cross_validate(
                X_ablated, y,
                hidden_layers=hidden_layers,
                n_folds=DEFAULT_CV_FOLDS,
                max_epochs=DEFAULT_MAX_EPOCHS,
                fp_weight=fp_weight,
                sample_weight=sample_weight,
                groups=groups,
                sample_context=sample_context,
                scaler_mode=scaler_mode,
                batch_size=batch_size,
                block_stride=DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
            )

        f1 = cv['f1_mean']
        delta = f1 - static_presence_f1

        results.append({
            'removed': feature_name,
            'n_features': len(feature_names) - 1,
            'f1_mean': f1,
            'f1_std': cv['f1_std'],
            'oof_f1': cv['oof_f1'],
            'recall_mean': cv['recall_mean'],
            'fp_rate_mean': cv['fp_rate_mean'],
            'delta_f1': delta,
        })

        direction = "↑" if delta > 0.1 else "↓" if delta < -0.1 else "≈"
        print(
            f"    F1: {f1:.2f}% ({direction} {delta:+.2f}%), "
            f"blocked OOF={cv['oof_f1']:.2f}%\n"
        )

    # Print summary table
    print("\n" + "="*85)
    print("                           ABLATION SUMMARY")
    print("="*85 + "\n")

    # Sort by delta (worst impact first = most important features)
    sorted_results = sorted(results[1:], key=lambda r: r['delta_f1'])

    print(f"{'Removed Feature':<24} {'F1 (CV)':>14} {'OOF F1':>10} {'Delta':>10} {'Recall':>10} {'FP Rate':>10} {'Note':<12}")
    print("-"*85)

    # Print baseline first
    bl = results[0]
    print(f"{'None (baseline)':<24} {bl['f1_mean']:>8.2f}% +/-{bl['f1_std']:.1f} "
          f"{bl['oof_f1']:>9.2f}% {'---':>10} {bl['recall_mean']:>9.1f}% {bl['fp_rate_mean']:>9.1f}%")
    print("-"*85)

    important_features = []
    removable_features = []

    for r in sorted_results:
        delta_str = f"{r['delta_f1']:+.2f}%"

        note = ""
        if r['delta_f1'] < -0.5:
            note = "IMPORTANT"
            important_features.append(r['removed'])
        elif r['delta_f1'] > 0.1:
            note = "removable"
            removable_features.append(r['removed'])
        elif abs(r['delta_f1']) <= 0.1:
            note = "neutral"

        print(f"{r['removed']:<24} {r['f1_mean']:>8.2f}% +/-{r['f1_std']:.1f} "
              f"{r['oof_f1']:>9.2f}% {delta_str:>10} {r['recall_mean']:>9.1f}% {r['fp_rate_mean']:>9.1f}% {note:<12}")

    print("-"*85)

    # Recommendations
    print("\nInterpretation:")
    print("  - Delta < 0: Removing hurts performance (feature is important)")
    print("  - Delta > 0: Removing helps performance (feature adds noise)")
    print("  - Delta ≈ 0: Feature has minimal impact (candidate for removal)")

    print("\nRecommendations:")
    if important_features:
        print(f"  KEEP (removing hurts F1 by >0.5%): {', '.join(important_features)}")
    if removable_features:
        print(f"  REMOVE (removing helps F1 by >0.1%): {', '.join(removable_features)}")

    neutral = [r['removed'] for r in sorted_results if abs(r['delta_f1']) <= 0.1]
    if neutral:
        print(f"  NEUTRAL (minimal impact): {', '.join(neutral)}")

    print()
    return results


def print_cv_summary(cv_results, title="Primary grouped CV"):
    """Print the robust evaluation summary used for model selection."""
    print(f"\n{title}:")
    print(f"  Fold recall:    {cv_results['recall_mean']:.1f}% (+/- {cv_results['recall_std']:.1f}%)")
    print(f"  Fold precision: {cv_results['precision_mean']:.1f}% (+/- {cv_results['precision_std']:.1f}%)")
    print(f"  Fold FP rate:   {cv_results['fp_rate_mean']:.1f}% (+/- {cv_results['fp_rate_std']:.1f}%)")
    print(f"  Fold F1:        {cv_results['f1_mean']:.1f}% (+/- {cv_results['f1_std']:.1f}%)")
    print(f"  Blocked OOF F1: {cv_results['oof_f1']:.1f}%")
    print(f"  Scored windows: {cv_results['scored_samples']} / {cv_results['dense_samples']}")

    group_reports = cv_results.get('group_reports', {})
    provenance_keys = tuple(
        key for key in ('real_session_group', 'synthetic_session_group')
        if key in group_reports
    )
    for group_key in DEFAULT_REPORT_GROUP_KEYS + provenance_keys:
        report = group_reports.get(group_key)
        if not report:
            continue
        worst_recall = report['worst_recall']
        worst_fp = report['worst_fp_rate']
        print(
            f"  Worst {group_key} recall: "
            f"{worst_recall['group']} -> R={worst_recall['recall']:.1f}% "
            f"FP={worst_recall['fp_rate']:.1f}% (n={worst_recall['samples']})"
        )
        if worst_fp['group'] != worst_recall['group']:
            print(
                f"  Worst {group_key} FP:     "
                f"{worst_fp['group']} -> FP={worst_fp['fp_rate']:.1f}% "
                f"R={worst_fp['recall']:.1f}% (n={worst_fp['samples']})"
            )
        if group_key in ('lineage_group', 'session_group') + provenance_keys:
            tail_recall = report.get('tail_recall', {})
            tail_fp = report.get('tail_fp_rate', {})
            print(
                f"  Worst-{len(tail_recall.get('groups', []))} {group_key} mean: "
                f"R={tail_recall.get('value', 0.0):.1f}% "
                f"FP={tail_fp.get('value', 0.0):.1f}%"
            )


def select_regression_subset_indices(
    sample_context,
    max_samples=2048,
    block_stride=DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
):
    """Pick a deterministic subset for inference-regression artifacts."""
    if sample_context is None:
        return np.arange(0, max_samples)

    mask = build_block_mask(
        sample_context,
        stride=block_stride,
        group_key=DEFAULT_BLOCK_GROUP_KEY,
    )
    indices = np.flatnonzero(mask) if mask is not None else np.arange(len(next(iter(sample_context.values()))))
    if len(indices) == 0:
        return indices
    if len(indices) > max_samples:
        sampled = np.linspace(0, len(indices) - 1, num=max_samples, dtype=int)
        indices = indices[sampled]
    return indices


def read_exported_seed():
    """Read the seed embedded in generated weight files."""
    for path in (REFERENCE_SRC_DIR / 'ml_weights.py', CPP_DIR / 'ml_weights.h'):
        if not path.exists():
            continue
        try:
            with open(path, 'r', encoding='utf-8') as f:
                contents = f.read()
        except OSError:
            continue
        match = re.search(r'Seed:\s*(\d+)', contents)
        if match:
            return int(match.group(1))
    return None


def show_info():
    """Show dataset information."""
    print("\n" + "="*60)
    print("              DATASET INFORMATION")
    print("="*60 + "\n")

    # Load dataset info
    dataset_info = load_dataset_info()

    print("Labels defined in dataset_info.json:")
    for label, info in dataset_info.get('labels', {}).items():
        label_type = "MOTION" if label == 'motion' else "IDLE"
        print(f"  {label} -> {label_type}")
        if info.get('description'):
            print(f"    {info['description']}")
    print()

    # Load and analyze data
    _, stats = load_all_data()

    print(f"Chips available: {', '.join(stats['chips']) if stats['chips'] else 'None'}")
    print(f"Total packets: {stats['total']}")
    print(f"Session groups: {len(stats.get('session_groups', []))}")
    print(f"Named environments: {len(stats.get('environment_groups', []))}")
    print()

    print("Packets by label:")
    idle_total = 0
    motion_total = 0
    for label, count in sorted(stats['labels'].items()):
        is_motion = is_motion_label(label, dataset_info)
        label_type = "MOTION" if is_motion else "IDLE"
        print(f"  {label}: {count} packets ({label_type})")
        if is_motion:
            motion_total += count
        else:
            idle_total += count

    print("\nSummary:")
    print(f"  IDLE packets:   {idle_total}")
    print(f"  MOTION packets: {motion_total}")
    print()

    # Show data directory contents
    print("Data directory contents:")
    for subdir in sorted(DATA_DIR.iterdir()):
        if subdir.is_dir() and not subdir.name.startswith('.'):
            files = list(subdir.glob('*.npz'))
            if files:
                print(f"  {subdir.name}/: {len(files)} files")
                for f in sorted(files)[:3]:
                    print(f"    - {f.name}")
                if len(files) > 3:
                    print(f"    ... and {len(files) - 3} more")
    print()


def train_all(fp_weight=DEFAULT_FP_WEIGHT, seed=None, feature_names=None,
              feature_importance=False, ablation=False, shap_samples=200,
              hidden_layers=None, scaler_mode=DEFAULT_SCALER_MODE,
              batch_size=DEFAULT_BATCH_SIZE, export_artifacts=True,
              evaluate_deployment=False,
              deployment_roles=('selection', 'holdout'),
              allow_legacy_gate_fallback=True,
              force_export=False,
              environment_filter=None, excluded_chips=None,
              positive_chip_boost=None,
              use_cache=True, augment=False,
              timing_quality_policy=DEFAULT_TIMING_QUALITY_POLICY,
              timing_warn_weight=DEFAULT_TIMING_WARN_WEIGHT):
    """
    Train models with all available data.

    Args:
        fp_weight: Multiplier for class 0 (IDLE) weight. Values >1.0 penalize
                   false positives more, producing a more conservative model.
        seed: Optional random seed for reproducible training. If None, a random
              seed is generated and saved for reproducibility.
        feature_names: List of feature names to use. If None, uses DEFAULT_FEATURES.
        feature_importance: If True, calculate grouped out-of-fold SHAP importance.
        ablation: If True, run ablation study instead of training.
        hidden_layers: Hidden layer widths. None uses DEFAULT_HIDDEN_LAYERS.
        scaler_mode: Feature normalization mode.
        batch_size: Mini-batch size used for training and CV.
        export_artifacts: If False, leave runtime artifacts unchanged.
        evaluate_deployment: Train the final in-memory model and run the paired
                             gate even when artifacts are not exported.
        deployment_roles: Dataset roles allowed in the deployment replay.
        allow_legacy_gate_fallback: Use the latest real train pair when no
                                    role-isolated replay is configured.
        force_export: Export runtime artifacts even when the deployment
                      safety gates fail or regress. Gates still run and their
                      results are printed; the bypass is reported loudly.
        environment_filter: Optional environment name(s) to keep.
        excluded_chips: Optional chip name(s) to exclude.
        positive_chip_boost: Optional {CHIP: factor} boost applied to motion
                             samples after feature extraction.
        use_cache: If True, reuse the cached feature matrix.
        augment: Optional augmentation component set. ``base`` keeps the
                 current validated recipe, while ``drift`` and ``burst-loss``
                 add their named components.
    Returns:
        tuple[int, int | None, dict | None]:
            (exit_code, used_seed, evaluation_summary)
            - exit_code: 0 on success, non-zero on failure
            - used_seed: seed used for training (None only on early dependency errors)
            - evaluation_summary: CV report used for model selection
    """
    total_start = perf_counter()
    if scaler_mode == 'clipped_standard' and (export_artifacts or evaluate_deployment):
        print(
            "Error: clipped_standard cannot be used for runtime evaluation or "
            "export. Use --no-export for host-side CV, or choose an affine scaler."
        )
        return 1, seed, None
    environment_filter = parse_environment_filter(environment_filter)
    excluded_chips = parse_chip_filter(excluded_chips)
    positive_chip_boost = parse_positive_chip_boost(positive_chip_boost)
    augment_components, feature_augmentation, packet_augmentation = resolve_training_augmentation(augment)
    if hidden_layers is None:
        hidden_layers = list(DEFAULT_HIDDEN_LAYERS)
    if feature_names is None:
        feature_names = DEFAULT_FEATURES.copy()
    feature_names = list(feature_names)
    if export_artifacts:
        unsupported = [name for name in feature_names if name not in CPP_FEATURE_IDS]
        if unsupported:
            print(
                "Error: runtime export requires a C++ extractor id for every "
                f"feature; missing: {', '.join(unsupported)}"
            )
            return 1, seed, None

    print("\n" + "="*60)
    print("           ML MOTION DETECTOR TRAINING")
    print("="*60 + "\n")

    # Check dependencies and initialize deterministic training state.
    try:
        ensure_torch_available()
        torch_device_label = describe_torch_device()
        seed = resolve_training_seed(seed, trailing_newline=True)
        set_global_determinism(seed, torch_module=torch)
    except ImportError as e:
        print(f"Error: Missing dependency - {e}")
        print("Install with: pip install torch scikit-learn")
        return 1, None, None
    except (RuntimeError, ValueError) as e:
        print(f"Error: {e}")
        return 1, seed, None

    # Load or build the feature matrix used by training and CV.
    print("Loading training matrix...")
    matrix, _all_packets = load_training_matrix(
        environment_filter=environment_filter,
        excluded_chips=excluded_chips,
        feature_names=feature_names if feature_names is not None else DEFAULT_FEATURES.copy(),
        use_cache=use_cache,
        timing_quality_policy=timing_quality_policy,
        timing_warn_weight=timing_warn_weight,
    )
    X = matrix['X']
    y = matrix['y']
    actual_feature_names = matrix['feature_names']
    sample_context = matrix['sample_context']
    sample_weights = matrix['sample_weights']
    stats = matrix['stats']
    X_aug = y_aug = groups_aug = None
    if packet_augmentation:
        print("Loading packet-augmented training matrix...")
        aug_matrix, _ = load_training_matrix(
            environment_filter=environment_filter,
            excluded_chips=excluded_chips,
            feature_names=feature_names,
            use_cache=use_cache,
            packet_augmentation=packet_augmentation,
            augmentation_seeds=training_packet_augmentation_seeds(
                packet_augmentation
            ),
            timing_quality_policy=timing_quality_policy,
            timing_warn_weight=timing_warn_weight,
        )
        X_aug = aug_matrix['X']
        y_aug = aug_matrix['y']
        groups_aug = aug_matrix['sample_context'].get(DEFAULT_PRIMARY_GROUP_KEY)

    if not stats['chips']:
        print("Error: No datasets found in data/")
        print("Collect data using: ./espectre collect --label static_presence --duration 60")
        return 1, seed, None

    print(f"  Chips: {', '.join(stats['chips'])}")
    if environment_filter is not None:
        print(f"  Environment filter: {', '.join(sorted(environment_filter))}")
    if stats.get('excluded_chips'):
        print(f"  Excluded chips: {', '.join(stats['excluded_chips'])}")
    if stats.get('excluded_environments'):
        print(f"  Excluded environments: {', '.join(stats['excluded_environments'])}")
    print(f"  Session groups: {len(stats.get('session_groups', []))}")
    print(f"  Lineage groups: {len(stats.get('lineage_groups', []))}")
    if stats.get('excluded_dataset_roles'):
        print(
            "  Reserved roles excluded from training: "
            + ', '.join(stats['excluded_dataset_roles'])
        )
    if stats.get('excluded_long_recordings'):
        print(
            "  Long-recording empty replays excluded from training: "
            + str(len(stats['excluded_long_recordings']))
        )
    if stats.get('environment_groups'):
        print(f"  Named environments: {len(stats['environment_groups'])}")
    timing_counts = stats.get('timing_quality_counts', {})
    print(
        "  Timing provenance: "
        f"clean={timing_counts.get('clean', 0)}, "
        f"degraded={timing_counts.get('degraded', 0)}, "
        f"poor={timing_counts.get('poor', 0)}, "
        f"unknown={timing_counts.get('unknown', 0)}"
    )
    if stats.get('excluded_timing_quality'):
        print(
            "  Poor-timing files excluded: "
            + str(len(stats['excluded_timing_quality']))
        )
    for label, count in sorted(stats['labels'].items()):
        print(f"  {label}: {count} packets")
    print(f"  Total: {stats['total']} packets")

    print(f"Architecture: {' -> '.join(map(str, [len(feature_names)] + hidden_layers + [1]))}")
    print(f"Scaler: {scaler_mode}")
    print(f"Batch size: {batch_size}")
    print(f"Training sample contract: {TRAINING_SAMPLE_CONTRACT} (only supported contract)")
    print(
        "Augmentation: "
        f"{format_augmentation_config(feature_augmentation, packet_augmentation, components=augment_components)}"
    )
    if packet_augmentation:
        print(
            "Packet augmentation seeds: "
            + ", ".join(str(seed) for seed in FIXED_PACKET_AUGMENTATION_SEEDS)
        )
    print(f"Torch device: {torch_device_label}\n")

    print(f"  Samples: {len(X)}")
    print(f"  Features: {len(actual_feature_names)}")
    print(f"  Feature set: {', '.join(actual_feature_names)}")
    n_idle = np.sum(y == 0)
    n_motion = np.sum(y == 1)
    print(f"  Class balance: IDLE={n_idle}, MOTION={n_motion}")
    if n_idle > 0 and n_motion > 0:
        ratio = max(n_idle, n_motion) / min(n_idle, n_motion)
        print(f"  Imbalance ratio: {ratio:.1f}:1")
    if X_aug is not None:
        print(f"  Packet-augmented samples: {len(X_aug)} (train-only)")

    eval_groups = sample_context[DEFAULT_PRIMARY_GROUP_KEY]
    unique_eval_groups = len(set(eval_groups))
    print(f"  Primary eval groups ({DEFAULT_PRIMARY_GROUP_KEY}): {unique_eval_groups}")
    print(
        "  Evaluation block stride: "
        f"{DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE} windows per source file"
    )

    boosted_weights, boost_summary = apply_positive_chip_boost(
        sample_weights,
        sample_context,
        y,
        positive_chip_boost,
    )
    sample_weights = boosted_weights
    if len(sample_weights) != len(X):
        print(
            f"Error: sample weights mismatch (weights={len(sample_weights)}, samples={len(X)})."
        )
        return 1, seed, None
    print(
        f"  Weight stats: min={float(np.min(sample_weights)):.3f}, "
        f"max={float(np.max(sample_weights)):.3f}, "
        f"mean={float(np.mean(sample_weights)):.3f}"
    )
    if timing_quality_policy != DEFAULT_TIMING_QUALITY_POLICY:
        print(
            "  Timing quality policy: "
            f"{timing_quality_policy}"
            + (
                f" (warn weight={float(timing_warn_weight):.2f})"
                if "downweight-warn" in str(timing_quality_policy)
                else ""
            )
        )
    if positive_chip_boost is not None:
        applied = [
            f"{chip}x{info['factor']:.2f} ({info['affected']} motion windows)"
            for chip, info in boost_summary.items()
        ]
        print(f"  Positive chip boost: {', '.join(applied) if applied else 'none'}")

    # Run ablation study if requested
    if ablation:
        print(
            "\nCV-only ablation is a screening diagnostic. "
            "Validate any finalist with --ablation-feature before making a feature decision."
        )
        run_ablation_study(
            X, y, actual_feature_names,
            sample_context=sample_context,
            sample_weight=sample_weights,
            hidden_layers=hidden_layers,
            fp_weight=fp_weight,
            scaler_mode=scaler_mode,
            batch_size=batch_size,
        )
        return 0, seed, None

    if fp_weight != 1.0:
        print(f"\nFP weight: {fp_weight}x (penalizing false positives)")
    print(
        f"\n{min(DEFAULT_CV_FOLDS, unique_eval_groups)}-fold grouped CV by "
        f"{DEFAULT_PRIMARY_GROUP_KEY}..."
    )
    cv_start = perf_counter()
    with suppress_stderr():
        cv_results = cross_validate(
            X, y,
            hidden_layers=hidden_layers,
            n_folds=DEFAULT_CV_FOLDS,
            max_epochs=DEFAULT_MAX_EPOCHS,
            fp_weight=fp_weight,
            sample_weight=sample_weights,
            groups=eval_groups,
            sample_context=sample_context,
            scaler_mode=scaler_mode,
            batch_size=batch_size,
            block_stride=DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
            block_group_key=DEFAULT_BLOCK_GROUP_KEY,
            report_group_keys=DEFAULT_REPORT_GROUP_KEYS,
            seed=seed,
            shap_samples=shap_samples if feature_importance else 0,
            shap_feature_names=actual_feature_names,
            shap_seed=derive_seed(seed, 20_000),
            feature_augmentation=feature_augmentation or None,
            X_aug=X_aug,
            y_aug=y_aug,
            groups_aug=groups_aug,
        )
    cv_elapsed = perf_counter() - cv_start
    print(f"\nCV total time: {format_duration(cv_elapsed)}")

    print_cv_summary(cv_results)
    if feature_importance and cv_results.get('shap_importance'):
        print(
            f"\nGrouped out-of-fold SHAP used "
            f"{cv_results['shap_samples']} balanced held-out samples."
        )
        print_feature_importance(
            cv_results['shap_importance'],
            title="Feature Importance (grouped out-of-fold SHAP)",
        )

    if not export_artifacts and not evaluate_deployment:
        return 0, seed, cv_results

    # Train final model on full dataset for production export
    print("\nTraining final model on full dataset...")
    final_train_start = perf_counter()
    scaler = build_preprocessor(scaler_mode)
    fit_preprocessor(scaler, X, y=y, sample_context=sample_context)
    X_scaled = scaler.transform(X)
    y_final = y
    sw_final = sample_weights
    X_scaled, y_final, sw_final = _append_augmented_training_rows(
        X_scaled,
        y_final,
        scaler,
        X_aug,
        y_aug,
        groups_aug,
        eval_groups,
        sample_weight=sw_final,
    )
    feature_bounds = None
    if feature_augmentation:
        feature_bounds = normalized_feature_bounds(scaler, actual_feature_names)

    with suppress_stderr():
        model = train_model(
            X_scaled, y_final,
            hidden_layers=hidden_layers,
            max_epochs=DEFAULT_MAX_EPOCHS,
            fp_weight=fp_weight,
            sample_weight=sw_final,
            batch_size=batch_size,
            seed=derive_seed(seed, 10_000),
            feature_augmentation=feature_augmentation or None,
            feature_bounds=feature_bounds,
        )
    print(f"  Final training time: {format_duration(perf_counter() - final_train_start)}")

    if evaluate_deployment:
        print("\nEvaluating in-memory candidate on deployment safety recordings...")
        def gate_progress(message):
            print(f"  {message}")
        paired_gate = evaluate_paired_gate(
            model,
            scaler,
            actual_feature_names,
            roles=deployment_roles,
            allow_legacy_fallback=allow_legacy_gate_fallback,
            progress=gate_progress,
        )
        quiet_gate = evaluate_quiet_gate(
            model,
            scaler,
            actual_feature_names,
            roles=deployment_roles,
            progress=gate_progress,
        )
        occupancy_paired_gate = evaluate_occupancy_paired_gate(
            model,
            scaler,
            actual_feature_names,
            roles=deployment_roles,
            allow_legacy_fallback=allow_legacy_gate_fallback,
            progress=gate_progress,
        )
        occupancy_quiet_gate = evaluate_occupancy_quiet_gate(
            model,
            scaler,
            actual_feature_names,
            roles=deployment_roles,
            progress=gate_progress,
        )
        gain_stress = evaluate_candidate_gain_stress(
            model,
            scaler,
            actual_feature_names,
            environment_filter=environment_filter,
            excluded_chips=excluded_chips,
            dataset_roles=deployment_roles,
        )
        cv_results['paired'] = paired_gate
        cv_results['quiet'] = quiet_gate
        cv_results['occupancy_paired'] = occupancy_paired_gate
        cv_results['occupancy_quiet'] = occupancy_quiet_gate
        cv_results['gain_stress'] = gain_stress
        print(
            f"  Paired: pass={paired_gate['pass_count']} "
            f"maxFP={paired_gate['max_fp_rate']:.2f}% "
            f"worstRecall={paired_gate['worst_chip_recall']:.2f}% "
            f"alarms={paired_gate.get('total_effective_alarms', 0)}"
        )
        if quiet_gate is None:
            print("  Quiet holdout: not configured")
        else:
            print(
                f"  Quiet holdout: {'pass' if quiet_gate['passed'] else 'fail'} "
                f"maxFP={quiet_gate['max_fp_rate']:.2f}% "
                f"alarms={quiet_gate['total_effective_alarms']}"
            )
        occupancy_paired_total = len((occupancy_paired_gate or {}).get('by_chip', {}))
        if occupancy_paired_gate is None:
            print("  Occupancy 70% paired: not configured")
        else:
            print(
                f"  Occupancy 70% paired: pass={occupancy_paired_gate['pass_count']} "
                f"maxFP={occupancy_paired_gate['max_fp_rate']:.2f}% "
                f"worstRecall={occupancy_paired_gate['worst_chip_recall']:.2f}% "
                f"alarms={occupancy_paired_gate.get('total_effective_alarms', 0)}"
            )
        if occupancy_quiet_gate is None:
            print("  Occupancy 70% quiet: not configured")
        else:
            print(
                f"  Occupancy 70% quiet: "
                f"{'pass' if occupancy_quiet_gate['passed'] else 'fail'} "
                f"maxFP={occupancy_quiet_gate['max_fp_rate']:.2f}% "
                f"alarms={occupancy_quiet_gate['total_effective_alarms']}"
            )
        print_gain_stress_summary(gain_stress, title="IN-MEMORY ML GAIN-STRESS GATE")
        paired_total = len(paired_gate.get('by_chip', {}))
        occupancy_failed = (
            occupancy_paired_gate is None
            or occupancy_paired_total == 0
            or occupancy_paired_gate['pass_count'] < occupancy_paired_total
            or occupancy_quiet_gate is None
            or not occupancy_quiet_gate['passed']
        )
        if export_artifacts and (
            paired_total == 0
            or paired_gate['pass_count'] < paired_total
            or (quiet_gate is not None and not quiet_gate['passed'])
            or occupancy_failed
        ):
            if force_export:
                print(
                    "WARNING: deployment safety gate FAILED; exporting anyway "
                    "because --force-promote bypasses the promotion rules"
                )
            else:
                print("Error: deployment safety gate failed; runtime artifacts were not exported")
                return 1, seed, cv_results
        try:
            baseline_paired = evaluate_exported_paired_gate(
                roles=deployment_roles,
                allow_legacy_fallback=allow_legacy_gate_fallback,
            )
        except (FileNotFoundError, ImportError, AttributeError) as exc:
            baseline_paired = None
            print(f"  Exported baseline unavailable ({exc}); using absolute paired gate")
        if baseline_paired is not None:
            cv_results['baseline_paired'] = baseline_paired
            print(
                f"  Baseline paired: pass={baseline_paired['pass_count']} "
                f"maxFP={baseline_paired['max_fp_rate']:.2f}% "
                f"worstRecall={baseline_paired['worst_chip_recall']:.2f}%"
            )
            paired_failures = paired_non_regression_failures(
                paired_gate, baseline_paired)
            cv_results['paired_non_regression_failures'] = paired_failures
            if paired_failures:
                label = (
                    "Blocked by per-recording non-regression on:"
                    if export_artifacts
                    else "Per-recording non-regression failures:"
                )
                print(f"  {label}")
                print(format_non_regression_failures(paired_failures, indent='    '))
                if export_artifacts and force_export:
                    print(
                        "WARNING: candidate regresses the paired deployment "
                        "gate; exporting anyway because --force-promote "
                        "bypasses the promotion rules"
                    )
                elif export_artifacts:
                    print(
                        "Error: candidate regresses the paired deployment gate; "
                        "runtime artifacts were not exported"
                    )
                    return 1, seed, cv_results

        try:
            baseline_occupancy_paired = evaluate_exported_occupancy_paired_gate(
                roles=deployment_roles,
                allow_legacy_fallback=allow_legacy_gate_fallback,
            )
        except (FileNotFoundError, ImportError, AttributeError) as exc:
            baseline_occupancy_paired = None
            print(
                f"  Exported occupancy baseline unavailable ({exc}); "
                "using absolute occupancy gate"
            )
        if baseline_occupancy_paired is not None and occupancy_paired_gate is not None:
            cv_results['baseline_occupancy_paired'] = baseline_occupancy_paired
            print(
                f"  Baseline occupancy 70% paired: "
                f"pass={baseline_occupancy_paired['pass_count']} "
                f"maxFP={baseline_occupancy_paired['max_fp_rate']:.2f}% "
                f"worstRecall={baseline_occupancy_paired['worst_chip_recall']:.2f}%"
            )
            occupancy_failures = paired_non_regression_failures(
                occupancy_paired_gate, baseline_occupancy_paired)
            cv_results['occupancy_non_regression_failures'] = occupancy_failures
            if occupancy_failures:
                label = (
                    "Blocked by occupancy-70% per-recording non-regression on:"
                    if export_artifacts
                    else "Occupancy-70% per-recording non-regression failures:"
                )
                print(f"  {label}")
                print(format_non_regression_failures(occupancy_failures, indent='    '))
                if export_artifacts and force_export:
                    print(
                        "WARNING: candidate regresses the occupancy-70% "
                        "deployment gate; exporting anyway because "
                        "--force-promote bypasses the promotion rules"
                    )
                elif export_artifacts:
                    print(
                        "Error: candidate regresses the occupancy-70% "
                        "deployment gate; runtime artifacts were not exported"
                    )
                    return 1, seed, cv_results

    if not export_artifacts:
        print("\nArtifacts unchanged.")
        return 0, seed, cv_results

    regression_indices = select_regression_subset_indices(
        sample_context,
        max_samples=2048,
        block_stride=DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
    )

    # Export models
    print("\nExporting model artifacts...")
    export_start = perf_counter()

    mp_path = REFERENCE_SRC_DIR / 'ml_weights.py'
    cpp_path = CPP_DIR / 'ml_weights.h'
    test_data_path = GENERATED_DATA_DIR / 'ml_test_data.npz'
    with tempfile.TemporaryDirectory(prefix='espectre_model_export_') as staging:
        staging_dir = Path(staging)
        staged_mp_path = staging_dir / mp_path.name
        staged_cpp_path = staging_dir / cpp_path.name
        staged_test_data_path = staging_dir / test_data_path.name
        mp_size = export_micropython(
            model, scaler, staged_mp_path,
            seed=seed,
            feature_names=actual_feature_names,
            scaler_mode=scaler_mode,
        )
        cpp_size = export_cpp_weights(
            model, scaler, staged_cpp_path,
            seed=seed,
            feature_names=actual_feature_names,
            scaler_mode=scaler_mode,
        )
        with suppress_stderr():
            n_test = export_test_data(
                model,
                scaler,
                X[regression_indices],
                y[regression_indices],
                staged_test_data_path,
            )
        atomic_write_set({
            mp_path: staged_mp_path.read_bytes(),
            cpp_path: staged_cpp_path.read_bytes(),
            test_data_path: staged_test_data_path.read_bytes(),
        })

    print(f"  MicroPython weights: {mp_path.name} ({mp_size/1024:.1f} KB)")
    print(f"  C++ weights: {cpp_path.name} ({cpp_size/1024:.1f} KB)")
    print(f"  Test data: {test_data_path.name} ({n_test} blocked samples)")
    print(f"  Export time: {format_duration(perf_counter() - export_start)}")

    print("\n" + "="*60)
    print("                    DONE!")
    print("="*60)
    print(
        f"\nModel trained with blocked grouped CV F1={cv_results['oof_f1']:.1f}% "
        f"(fold mean {cv_results['f1_mean']:.1f}% +/- {cv_results['f1_std']:.1f}%)"
    )
    print("\nGenerated files:")
    print(f"  - {mp_path} (MicroPython)")
    print(f"  - {cpp_path} (C++ ESPHome)")
    print(f"  - {test_data_path} (test data for validation)")
    print(f"\nTotal runtime: {format_duration(perf_counter() - total_start)}")
    print()

    return 0, seed, cv_results


def _paired_gate_key(paired_metrics):
    """Ranking key for paired real-data gate results."""
    if paired_metrics is None:
        return None
    return (
        paired_metrics.get('pass_count', 0),
        paired_metrics.get('worst_chip_recall', -float('inf')),
        paired_metrics.get('worst_chip_f1', -float('inf')),
        -paired_metrics.get('max_fp_rate', float('inf')),
        -paired_metrics.get('total_effective_alarms', float('inf')),
        paired_metrics.get('mean_f1', -float('inf')),
        paired_metrics.get('mean_recall', -float('inf')),
    )


def _combined_candidate_key(cv_metrics, paired_metrics=None):
    """
    Final selection key.

    After deployment safety passes, paired replay recall leads ranking and
    grouped OOF robustness breaks ties between similarly safe candidates.
    """
    cv_key = build_candidate_key(cv_metrics)
    paired_key = _paired_gate_key(paired_metrics)
    if paired_key is None:
        return cv_key
    return paired_key + cv_key


def _format_exported_gate_summary(gate):
    """Build a short one-line summary for exported-artifact verification."""
    if gate is None:
        return "exported_gates=not_run"
    metrics = gate.paired_metrics
    if metrics is None:
        summary = "paired=not_configured"
    else:
        paired = (
            "paired=pass"
            if gate.paired_returncode == 0
            else f"paired=fail({gate.paired_returncode})"
        )
        summary = (
            f"{paired} maxFP={metrics.get('max_fp_rate', 0.0):.2f}% "
            f"worstRecall={metrics.get('worst_chip_recall', 0.0):.2f}% "
            f"worstF1={metrics.get('worst_chip_f1', 0.0):.2f}% "
            f"alarms={metrics.get('total_effective_alarms', 0)}"
        )
    if gate.quiet_metrics is None:
        summary += " quiet=not_configured"
    else:
        summary += (
            f" quietMaxFP={gate.quiet_metrics.get('max_fp_rate', 0.0):.2f}%"
            f" quietAlarms={gate.quiet_metrics.get('total_effective_alarms', 0)}"
        )
    occupancy_paired = gate.occupancy_paired_metrics
    occupancy_quiet = gate.occupancy_quiet_metrics
    if occupancy_paired is None:
        return summary + " occupancy70=not_configured"
    occupancy_total = len(occupancy_paired.get('by_chip', {}))
    occupancy_pass = (
        "occupancy70=pass"
        if gate.occupancy_passed
        else "occupancy70=fail"
    )
    quiet_fp = (
        occupancy_quiet.get('max_fp_rate', 0.0) if occupancy_quiet else 0.0
    )
    quiet_alarms = (
        occupancy_quiet.get('total_effective_alarms', 0) if occupancy_quiet else 0
    )
    return (
        summary
        + f" {occupancy_pass}({occupancy_paired.get('pass_count', 0)}/{occupancy_total})"
        + f" occMaxFP={occupancy_paired.get('max_fp_rate', 0.0):.2f}%"
        + f" occWorstRecall={occupancy_paired.get('worst_chip_recall', 0.0):.2f}%"
        + f" occQuietMaxFP={quiet_fp:.2f}% occQuietAlarms={quiet_alarms}"
    )


def _candidate_beats_baseline(candidate_cv, candidate_gate, static_presence_cv, static_presence_gate):
    """Require deployment safety plus robust grouped-CV improvement."""
    if candidate_gate is None or static_presence_gate is None:
        return False
    if not candidate_gate.passed or not static_presence_gate.passed:
        return False
    if (
        candidate_gate.paired_metrics is not None
        and static_presence_gate.paired_metrics is not None
        and not paired_result_non_regression(
            candidate_gate.paired_metrics,
            static_presence_gate.paired_metrics,
        )
    ):
        return False
    if (
        candidate_gate.occupancy_paired_metrics is not None
        and static_presence_gate.occupancy_paired_metrics is not None
        and not paired_result_non_regression(
            candidate_gate.occupancy_paired_metrics,
            static_presence_gate.occupancy_paired_metrics,
        )
    ):
        return False
    return compare_robust_cv(candidate_cv, static_presence_cv)['passed']


def _format_candidate_comparison(candidate_cv, baseline_cv):
    """Format material CV deltas and equivalence decisions for seed search."""
    comparison = compare_robust_cv(candidate_cv, baseline_cv)
    parts = []
    for check in comparison['checks']:
        state = 'regression' if check['regressed'] else 'improvement' if check['improved'] else 'tie'
        parts.append(
            f"{check['label']} {check['delta']:+.2f}pp "
            f"(margin {check['margin']:.2f}, {state})"
        )
    return comparison, '; '.join(parts)


def _search_candidate_key(cv_metrics, gate=None):
    """Ranking key for broken-baseline seed search fallback."""
    gate_passed = 1 if gate is not None and gate.passed else 0
    paired_passed = 1 if gate is not None and gate.paired_returncode == 0 else 0
    paired_key = _paired_gate_key(gate.paired_metrics if gate is not None else None)
    if paired_key is None:
        paired_key = (-float('inf'),) * 7
    return (
        gate_passed,
        paired_passed,
    ) + tuple(paired_key) + build_candidate_key(cv_metrics)


def train_until_improvement(max_trials, fp_weight=DEFAULT_FP_WEIGHT, feature_names=None,
                            hidden_layers=None, scaler_mode=DEFAULT_SCALER_MODE,
                            batch_size=DEFAULT_BATCH_SIZE, environment_filter=None,
                            excluded_chips=None, positive_chip_boost=None,
                            use_cache=True, augment=False,
                            timing_quality_policy=DEFAULT_TIMING_QUALITY_POLICY,
                            timing_warn_weight=DEFAULT_TIMING_WARN_WEIGHT,
                            search_output_path=DEFAULT_SEED_SEARCH_OUTPUT,
                            export_artifacts=True):
    """
    Train all requested seeds and keep the strongest robust improvement.

    Baseline is recomputed using the seed embedded in the current exported model.
    Deployment replays are safety gates. Grouped OOF worst/tail metrics lead
    ranking with one-event equivalence margins.

    When the current exported baseline fails the paired gate, the command still
    evaluates all MAX_TRIALS candidates, but only a candidate that restores the
    deployment safety gate can replace it.
    """
    if max_trials < 1:
        print("Error: --seed-search-until-improvement must be >= 1")
        return 1

    if feature_names is None:
        feature_names = DEFAULT_FEATURES
    if hidden_layers is None:
        hidden_layers = list(DEFAULT_HIDDEN_LAYERS)
    host_only_search = any(name not in CPP_FEATURE_IDS for name in feature_names)
    in_memory_search = host_only_search or not export_artifacts
    excluded_chips = parse_chip_filter(excluded_chips)
    positive_chip_boost = parse_positive_chip_boost(positive_chip_boost)
    augment_components, feature_augmentation, packet_augmentation = resolve_training_augmentation(augment)
    try:
        ensure_torch_available()
        torch_device_label = describe_torch_device()
    except ImportError as exc:
        print(f"Error: Missing dependency - {exc}")
        return 1
    except (RuntimeError, ValueError) as exc:
        print(f"Error: {exc}")
        return 1

    print("\n" + "=" * 70)
    print("  SEED SEARCH (evaluate all candidates)")
    print("=" * 70)
    print(f"Max trials: {max_trials}")
    print(f"FP weight: {fp_weight}")
    print(f"Scaler: {scaler_mode}")
    print(f"Batch size: {batch_size}")
    if in_memory_search:
        reason = "host-side candidate features" if host_only_search else "--no-export"
        print(f"Artifact mode: in-memory only ({reason})")
    print(
        "Augmentation: "
        f"{format_augmentation_config(feature_augmentation, packet_augmentation, components=augment_components)}"
    )
    if packet_augmentation:
        print(
            "Packet augmentation seeds: "
            + ", ".join(str(seed) for seed in FIXED_PACKET_AUGMENTATION_SEEDS)
        )
    print(f"Torch device: {torch_device_label}")
    if environment_filter is not None:
        print(f"Environment filter: {', '.join(sorted(parse_environment_filter(environment_filter)))}")
    if excluded_chips is not None:
        print(f"Excluded chips: {', '.join(sorted(excluded_chips))}")
    if positive_chip_boost is not None:
        print(
            "Positive chip boost: "
            + ', '.join(f"{chip}={factor:.2f}" for chip, factor in sorted(positive_chip_boost.items()))
        )

    static_presence_seed = read_exported_seed()
    if static_presence_seed is None:
        static_presence_seed = 42
        print("\nWarning: current exported seed not found, using 42 as baseline seed")

    train_kwargs = {
        'fp_weight': fp_weight,
        'feature_names': feature_names,
        'feature_importance': False,
        'ablation': False,
        'hidden_layers': hidden_layers,
        'scaler_mode': scaler_mode,
        'batch_size': batch_size,
        'environment_filter': environment_filter,
        'excluded_chips': excluded_chips,
        'positive_chip_boost': positive_chip_boost,
        'use_cache': use_cache,
        'augment': augment_components,
        'timing_quality_policy': timing_quality_policy,
        'timing_warn_weight': timing_warn_weight,
        # Candidate selection may reuse selection recordings, but the holdout
        # stays sealed until exactly one winner has been chosen.
        'deployment_roles': ('selection',),
        'allow_legacy_gate_fallback': True,
    }

    baseline_train_kwargs = dict(train_kwargs)
    baseline_train_kwargs['feature_names'] = list(DEFAULT_FEATURES)
    print(f"\nEvaluating current model baseline with seed {static_presence_seed}...")
    static_presence_rc, _, static_presence_metrics = train_all(
        seed=static_presence_seed,
        export_artifacts=False,
        **baseline_train_kwargs,
    )
    if static_presence_rc != 0 or static_presence_metrics is None:
        print("Error: unable to evaluate current model baseline")
        return 1

    static_presence_session = static_presence_metrics.get('group_reports', {}).get('session_group', {}).get('worst_recall', {})
    static_presence_chip = static_presence_metrics.get('group_reports', {}).get('chip', {}).get('worst_recall', {})
    print(
        f"Baseline: session_min_recall={static_presence_session.get('recall', 0.0):.1f}% "
        f"chip_min_recall={static_presence_chip.get('recall', 0.0):.1f}% "
        f"blocked_oof_f1={static_presence_metrics['oof_f1']:.1f}%"
    )
    static_presence_gate = run_exported_ml_gates(roles=('selection',))
    print(f"Baseline exported ML gates: {_format_exported_gate_summary(static_presence_gate)}")
    baseline_holdout_gate = run_exported_ml_gates(
        roles=('holdout',),
        allow_legacy_fallback=False,
    )
    if baseline_holdout_gate.available:
        print(
            "Reserved baseline holdout captured for final non-regression: "
            f"{_format_exported_gate_summary(baseline_holdout_gate)}"
        )
    else:
        print("Reserved holdout: not configured")
    broken_baseline_mode = not static_presence_gate.passed
    if broken_baseline_mode:
        print(
            "Warning: baseline paired gate failed; "
            "running all trials and ranking candidates against the broken baseline"
        )

    search_results = {
        'config': {
            'max_trials': max_trials,
            'fp_weight': fp_weight,
            'feature_names': list(feature_names),
            'hidden_layers': list(hidden_layers),
            'scaler_mode': scaler_mode,
            'batch_size': batch_size,
            'augment': bool(parse_augmentation_components(augment)),
            'augmentation': format_augmentation_config(
                feature_augmentation,
                packet_augmentation,
                components=augment_components,
            ),
            'timing_quality_policy': timing_quality_policy,
            'timing_warn_weight': float(timing_warn_weight),
            'training_sample_contract': TRAINING_SAMPLE_CONTRACT,
            'export_artifacts': not in_memory_search,
            'environment_filter': environment_filter,
            'excluded_chips': sorted(excluded_chips) if excluded_chips else None,
            'started_at': datetime.now().isoformat(timespec='seconds'),
        },
        'baseline': {
            'seed': static_presence_seed,
            'feature_names': list(DEFAULT_FEATURES),
            'oof_f1': static_presence_metrics.get('oof_f1'),
            'session_min_recall': static_presence_session.get('recall'),
            'selection_paired_metrics': static_presence_gate.paired_metrics,
            'selection_quiet_metrics': static_presence_gate.quiet_metrics,
            'selection_occupancy_paired_metrics': (
                static_presence_gate.occupancy_paired_metrics
            ),
            'selection_occupancy_quiet_metrics': (
                static_presence_gate.occupancy_quiet_metrics
            ),
            'holdout_paired_metrics': baseline_holdout_gate.paired_metrics,
            'holdout_occupancy_paired_metrics': (
                baseline_holdout_gate.occupancy_paired_metrics
            ),
        },
        'trials': [],
        'final_holdout': None,
    }

    def _write_search_results():
        """Persist after every trial: a search that crashes at trial 9 of 10
        still leaves the per-replay rows behind."""
        if search_output_path is None:
            return
        write_json_results(Path(search_output_path), search_results)

    def _record_trial(trial_seed_value, status, cv_metrics, gate):
        """Record one evaluated trial in both the summary and the results file.

        Every branch that evaluates a candidate goes through here, including the
        broken-baseline ranking path: a search that promotes nothing still has to
        leave its per-replay rows behind, and that is the run whose rows matter
        most.
        """
        trial_summaries.append((trial_seed_value, cv_metrics, gate, status))
        session_summary = cv_metrics.get('group_reports', {}).get('session_group', {}).get('worst_recall', {})
        fp_summary = cv_metrics.get('group_reports', {}).get('session_group', {}).get('worst_fp_rate', {})
        search_results['trials'].append({
            'seed': trial_seed_value,
            'status': status,
            'oof_f1': cv_metrics.get('oof_f1'),
            'session_min_recall': session_summary.get('recall'),
            'session_max_fp_rate': fp_summary.get('fp_rate'),
            'cv': cv_metrics.get('cv'),
            'paired_passed': bool(gate.passed) if gate else None,
            'paired_metrics': gate.paired_metrics if gate else None,
            'quiet_metrics': gate.quiet_metrics if gate else None,
            'occupancy_paired_metrics': (
                gate.occupancy_paired_metrics if gate else None
            ),
            'occupancy_quiet_metrics': (
                gate.occupancy_quiet_metrics if gate else None
            ),
            'non_regression_failures': (
                paired_non_regression_failures(
                    gate.paired_metrics,
                    static_presence_gate.paired_metrics)
                if gate is not None
                and gate.paired_metrics is not None
                and static_presence_gate.paired_metrics is not None
                else []
            ),
            'occupancy_non_regression_failures': (
                paired_non_regression_failures(
                    gate.occupancy_paired_metrics,
                    static_presence_gate.occupancy_paired_metrics)
                if gate is not None
                and gate.occupancy_paired_metrics is not None
                and static_presence_gate.occupancy_paired_metrics is not None
                else []
            ),
        })
        _write_search_results()

    _write_search_results()

    if in_memory_search:
        backup_dir, saved_files = None, []
        print("Artifacts: unchanged throughout in-memory seed search")
    else:
        backup_dir, saved_files = _backup_artifacts()
        print(f"Artifacts backup: {backup_dir}")

    trial_summaries = []
    improved = False
    improved_seed = None
    improved_metrics = None
    improved_gate = None
    best_candidate_backup_dir = None
    best_candidate_saved_files = None
    best_search_key = _search_candidate_key(static_presence_metrics, static_presence_gate)

    for idx in range(1, max_trials + 1):
        trial_seed = generate_random_training_seed()
        print(f"\n[{idx}/{max_trials}] Training with auto-generated seed {trial_seed}")
        # One train_all per trial: CV once, then final fit for the paired gate.
        # Pass an explicit random seed so resolve_training_seed does not reuse the
        # currently exported model seed on every trial.
        export_rc, used_seed, final_metrics = train_all(
            seed=trial_seed,
            export_artifacts=not in_memory_search,
            evaluate_deployment=in_memory_search,
            **train_kwargs,
        )
        if export_rc != 0 or final_metrics is None:
            print(f"  Candidate training failed (exit={export_rc})")
            _restore_artifacts(saved_files)
            failure_status = 'training_failed' if in_memory_search else 'export_failed'
            trial_summaries.append((used_seed, final_metrics or {}, None, failure_status))
            search_results['trials'].append({
                'seed': used_seed,
                'status': failure_status,
                'returncode': export_rc,
            })
            _write_search_results()
            continue

        session_summary = final_metrics.get('group_reports', {}).get('session_group', {}).get('worst_recall', {})
        fp_summary = final_metrics.get('group_reports', {}).get('session_group', {}).get('worst_fp_rate', {})
        print(
            f"  Result: session_min_recall={session_summary.get('recall', 0.0):.1f}% "
            f"session_max_fp={fp_summary.get('fp_rate', 0.0):.1f}% "
            f"blocked_oof_f1={final_metrics['oof_f1']:.1f}%"
        )

        candidate_gate = (
            in_memory_gate_result(final_metrics)
            if in_memory_search
            else run_exported_ml_gates(roles=('selection',))
        )
        gate_kind = "In-memory" if in_memory_search else "Exported"
        print(f"  {gate_kind} ML gates: {_format_exported_gate_summary(candidate_gate)}")
        if not candidate_gate.passed and candidate_gate.paired_output.strip():
            print(candidate_gate.paired_output.strip())

        if broken_baseline_mode:
            status = 'ranked_rejected'
            candidate_search_key = _search_candidate_key(final_metrics, candidate_gate)
            if candidate_gate.passed and candidate_search_key > best_search_key:
                improved = True
                improved_seed = used_seed
                improved_metrics = final_metrics
                improved_gate = candidate_gate
                best_search_key = candidate_search_key
                if best_candidate_backup_dir is not None:
                    shutil.rmtree(best_candidate_backup_dir, ignore_errors=True)
                if not in_memory_search:
                    best_candidate_backup_dir, best_candidate_saved_files = _backup_artifacts()
                status = 'ranked_best'
                print("  Broken baseline mode: current best candidate updated")
            elif not candidate_gate.passed:
                print("  Broken baseline mode: candidate still fails deployment safety")
            else:
                print("  Broken baseline mode: candidate did not beat current best")
            _record_trial(used_seed, status, final_metrics, candidate_gate)
            _restore_artifacts(saved_files)
            continue

        comparison, comparison_text = _format_candidate_comparison(
            final_metrics,
            static_presence_metrics,
        )
        print(f"  Robust CV comparison: {comparison_text}")
        status = 'robust_rejected'
        if _candidate_beats_baseline(
            final_metrics,
            candidate_gate,
            static_presence_metrics,
            static_presence_gate,
        ):
            candidate_key = _combined_candidate_key(
                final_metrics,
                candidate_gate.paired_metrics,
            )
            if not improved or candidate_key > best_search_key:
                improved = True
                improved_seed = used_seed
                improved_metrics = final_metrics
                improved_gate = candidate_gate
                best_search_key = candidate_key
                if best_candidate_backup_dir is not None:
                    shutil.rmtree(best_candidate_backup_dir, ignore_errors=True)
                if not in_memory_search:
                    best_candidate_backup_dir, best_candidate_saved_files = _backup_artifacts()
                status = 'robust_best'
                print("  Robust improvement: current best candidate updated")
            else:
                status = 'robust_eligible'
                print("  Robust improvement: eligible, but not the current best")
        elif not candidate_gate.passed:
            print("  Deployment safety gate rejected candidate")
        elif not paired_result_non_regression(
            candidate_gate.paired_metrics,
            static_presence_gate.paired_metrics,
        ):
            print("  Per-recording paired non-regression rejected candidate")
            print(format_non_regression_failures(paired_non_regression_failures(
                candidate_gate.paired_metrics, static_presence_gate.paired_metrics)))
        elif comparison['regressions']:
            print("  Robust CV rejected candidate due to material regression")
        else:
            print("  Robust CV found no material improvement")
        _record_trial(used_seed, status, final_metrics, candidate_gate)
        _restore_artifacts(saved_files)

    print("\n" + "=" * 70)
    print("  UNTIL-IMPROVEMENT SUMMARY")
    print("=" * 70)
    for seed, metrics, gate, status in trial_summaries:
        session_summary = metrics.get('group_reports', {}).get('session_group', {}).get('worst_recall', {})
        fp_summary = metrics.get('group_reports', {}).get('session_group', {}).get('worst_fp_rate', {})
        print(
            f"  seed={seed} | sessionMinR={session_summary.get('recall', 0.0):.1f}% "
            f"sessionMaxFP={fp_summary.get('fp_rate', 0.0):.1f}% "
            f"blockedOOF={metrics.get('oof_f1', 0.0):.1f}% | {status} | "
            f"{_format_exported_gate_summary(gate)}"
        )

    if improved and in_memory_search:
        holdout_kwargs = dict(train_kwargs)
        holdout_kwargs['deployment_roles'] = ('holdout',)
        holdout_kwargs['allow_legacy_gate_fallback'] = False
        holdout_rc, _, holdout_metrics = train_all(
            seed=improved_seed,
            export_artifacts=False,
            evaluate_deployment=True,
            **holdout_kwargs,
        )
        if holdout_rc != 0 or holdout_metrics is None:
            print("Final reserved holdout evaluation failed")
            return 1
        final_holdout_gate = in_memory_gate_result(holdout_metrics)
        if final_holdout_gate.available:
            print(
                "Final reserved holdout: "
                f"{_format_exported_gate_summary(final_holdout_gate)}"
            )
            holdout_failures = (
                []
                if (baseline_holdout_gate.paired_metrics is None
                    or final_holdout_gate.paired_metrics is None)
                else paired_non_regression_failures(
                    final_holdout_gate.paired_metrics,
                    baseline_holdout_gate.paired_metrics,
                )
            )
            search_results['final_holdout'] = {
                'seed': improved_seed,
                'passed': bool(final_holdout_gate.passed),
                'paired_metrics': final_holdout_gate.paired_metrics,
                'quiet_metrics': final_holdout_gate.quiet_metrics,
                'non_regression_failures': holdout_failures,
            }
            _write_search_results()
            if not final_holdout_gate.passed or holdout_failures:
                if holdout_failures:
                    print("Blocked by per-recording non-regression on:")
                    print(format_non_regression_failures(holdout_failures))
                print("Final reserved holdout rejected the selected candidate")
                return 1
        search_results['selected_seed'] = improved_seed
        _write_search_results()
        print(
            f"\nSelected research seed after full robust ranking: {improved_seed} "
            f"(blocked_oof_f1={improved_metrics['oof_f1']:.1f}%, "
            f"{_format_exported_gate_summary(improved_gate)})"
        )
        print("Runtime artifacts unchanged; rerun the selected seed explicitly to export it")
        if search_output_path is not None:
            print(f"Seed search results: {search_output_path}")
        return 0

    if improved:
        if best_candidate_saved_files is not None:
            _restore_artifacts(best_candidate_saved_files)
            final_holdout_gate = run_exported_ml_gates(
                roles=('holdout',),
                allow_legacy_fallback=False,
            )
            if final_holdout_gate.available:
                print(
                    "Final reserved holdout: "
                    f"{_format_exported_gate_summary(final_holdout_gate)}"
                )
                holdout_failures = (
                    []
                    if (baseline_holdout_gate.paired_metrics is None
                        or final_holdout_gate.paired_metrics is None)
                    else paired_non_regression_failures(
                        final_holdout_gate.paired_metrics,
                        baseline_holdout_gate.paired_metrics,
                    )
                )
                search_results['final_holdout'] = {
                    'seed': improved_seed,
                    'passed': bool(final_holdout_gate.passed),
                    'paired_metrics': final_holdout_gate.paired_metrics,
                    'quiet_metrics': final_holdout_gate.quiet_metrics,
                    'non_regression_failures': holdout_failures,
                }
                _write_search_results()
                if not final_holdout_gate.passed or holdout_failures:
                    _restore_artifacts(saved_files)
                    if holdout_failures:
                        print("Blocked by per-recording non-regression on:")
                        print(format_non_regression_failures(holdout_failures))
                    print(
                        "Final reserved holdout rejected the selected candidate; "
                        "current artifacts were restored"
                    )
                    return 1
            search_results['selected_seed'] = improved_seed
            _write_search_results()
            print(
                f"\nSelected seed after full robust ranking: {improved_seed} "
                f"(blocked_oof_f1={improved_metrics['oof_f1']:.1f}%, "
                f"{_format_exported_gate_summary(improved_gate)})"
            )
            if search_output_path is not None:
                print(f"Seed search results: {search_output_path}")
            return 0

    search_results['selected_seed'] = None
    _write_search_results()
    if search_output_path is not None:
        print(f"\nSeed search results: {search_output_path}")
    if broken_baseline_mode:
        print("No candidate beat the current broken baseline; current artifacts remain unchanged")
        return 1

    print("No improvement found within max trials; current artifacts remain unchanged")
    return 1


def write_json_results(path, payload):
    """Write a JSON experiment payload."""
    atomic_write_text(
        path,
        json.dumps(payload, indent=2, default=_json_value) + '\n',
    )


def _json_value(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


def slim_cv_result(cv_results):
    """Keep only the CV fields needed by experiment payloads."""
    session_report = cv_results.get('group_reports', {}).get('session_group', {})
    chip_report = cv_results.get('group_reports', {}).get('chip', {})
    return {
        'f1_mean': float(cv_results['f1_mean']),
        'f1_std': float(cv_results['f1_std']),
        'oof_f1': float(cv_results['oof_f1']),
        'recall_mean': float(cv_results['recall_mean']),
        'fp_rate_mean': float(cv_results['fp_rate_mean']),
        'worst_session_recall': float(session_report.get('worst_recall', {}).get('recall', 0.0)),
        'worst_session_fp_rate': float(session_report.get('worst_fp_rate', {}).get('fp_rate', 0.0)),
        'worst_chip_recall': float(chip_report.get('worst_recall', {}).get('recall', 0.0)),
        'candidate_key': list(build_candidate_key(cv_results)),
    }


def architecture_stats(input_dim, hidden_layers):
    """Return parameter, size, and FLOP estimates for an MLP."""
    layer_sizes = [input_dim] + list(hidden_layers) + [1]
    n_params = 0
    flops = 0
    for idx in range(len(layer_sizes) - 1):
        n_params += layer_sizes[idx] * layer_sizes[idx + 1]
        n_params += layer_sizes[idx + 1]
        flops += layer_sizes[idx] * layer_sizes[idx + 1]
    return {
        'layer_sizes': layer_sizes,
        'params': int(n_params),
        'weight_kb': float(n_params * 4 / 1024),
        'flops': int(flops),
    }


def architecture_campaign_rank_key(result):
    """Sort safely passing runs by robust grouped-CV performance."""
    robust_key = tuple(-value for value in build_candidate_key(result['cv']))
    return (
        -result['paired']['pass_count'],
        result['paired'].get('total_effective_alarms', 0),
        *robust_key,
        result['paired']['max_fp_rate'],
        -result['paired']['worst_chip_recall'],
        -result['paired']['worst_chip_f1'],
        result['params'],
    )


def aggregate_architecture_runs(name, runs):
    """Aggregate multi-seed runs for one architecture."""
    template = runs[0]
    candidate_keys = np.asarray(
        [build_candidate_key(run['cv']) for run in runs],
        dtype=np.float64,
    )
    return {
        'name': name,
        'layers': list(template['layers']),
        'architecture': template['architecture'],
        'params': int(template['params']),
        'weight_kb': float(template['weight_kb']),
        'flops': int(template['flops']),
        'seeds': [int(run['seed']) for run in runs],
        'median_paired_pass_count': float(np.median([run['paired']['pass_count'] for run in runs])),
        'median_paired_effective_alarms': float(np.median([
            run['paired'].get('total_effective_alarms', 0) for run in runs
        ])),
        'median_paired_max_fp_rate': float(np.median([run['paired']['max_fp_rate'] for run in runs])),
        'median_paired_worst_chip_recall': float(np.median([
            run['paired']['worst_chip_recall'] for run in runs
        ])),
        'median_paired_worst_chip_f1': float(np.median([run['paired']['worst_chip_f1'] for run in runs])),
        'median_oof_f1': float(np.median([run['cv']['oof_f1'] for run in runs])),
        'median_cv_candidate_key': [
            float(value) for value in np.median(candidate_keys, axis=0)
        ],
        'best_single_run': min(runs, key=architecture_campaign_rank_key),
        'runs': runs,
    }


def aggregate_architecture_rank_key(summary):
    """Sort key for aggregated architecture summaries (lower is better)."""
    robust_key = tuple(-value for value in summary.get(
        'median_cv_candidate_key',
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, summary['median_oof_f1'], 0.0),
    ))
    return (
        -summary['median_paired_pass_count'],
        summary.get('median_paired_effective_alarms', 0.0),
        *robust_key,
        summary['median_paired_max_fp_rate'],
        -summary['median_paired_worst_chip_recall'],
        -summary['median_paired_worst_chip_f1'],
        summary['params'],
    )


def paired_non_regression(candidate, baseline):
    """Treat paired validation as a non-regression constraint."""
    return (
        candidate['median_paired_pass_count'] >= baseline['median_paired_pass_count']
        and candidate['median_paired_max_fp_rate'] <= baseline['median_paired_max_fp_rate'] + 1e-6
        and candidate['median_paired_worst_chip_recall']
        >= baseline['median_paired_worst_chip_recall'] - 0.25
        and candidate['median_paired_worst_chip_f1']
        >= baseline['median_paired_worst_chip_f1'] - 0.25
    )


def architecture_candidate_beats_baseline(candidate, baseline):
    """Promote only stable paired improvements that do not regress validation."""
    if candidate['name'] == baseline['name']:
        return True
    if not paired_non_regression(candidate, baseline):
        return False
    return aggregate_architecture_rank_key(candidate) < aggregate_architecture_rank_key(baseline)


def evaluate_architecture_candidate(
    name,
    hidden_layers,
    seed,
    dataset,
    scaler_mode,
    batch_size,
    fp_weight,
    feature_augmentation=None,
):
    """Train and evaluate one architecture on CV and the paired gate."""
    stats = architecture_stats(dataset['X'].shape[1], hidden_layers)
    print(f"\n== {name} | seed {seed} ==")
    print(
        f"Architecture: {' -> '.join(map(str, stats['layer_sizes']))} | "
        f"params={stats['params']} | weights={stats['weight_kb']:.1f} KB | flops={stats['flops']}"
    )

    with suppress_stderr():
        cv = cross_validate(
            dataset['X'],
            dataset['y'],
            hidden_layers=list(hidden_layers),
            n_folds=DEFAULT_CV_FOLDS,
            max_epochs=DEFAULT_MAX_EPOCHS,
            fp_weight=fp_weight,
            sample_weight=dataset['sample_weights'],
            groups=dataset['groups'],
            sample_context=dataset['sample_context'],
            scaler_mode=scaler_mode,
            batch_size=batch_size,
            block_stride=DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
            block_group_key=DEFAULT_BLOCK_GROUP_KEY,
            report_group_keys=DEFAULT_REPORT_GROUP_KEYS,
            seed=seed,
            shap_feature_names=dataset['feature_names'],
            feature_augmentation=feature_augmentation,
            X_aug=dataset.get('X_aug'),
            y_aug=dataset.get('y_aug'),
            groups_aug=dataset.get('groups_aug'),
        )

    scaler = build_preprocessor(scaler_mode)
    fit_preprocessor(
        scaler,
        dataset['X'],
        y=dataset['y'],
        sample_context=dataset['sample_context'],
    )
    X_scaled = scaler.transform(dataset['X'])
    y_train = dataset['y']
    sample_weight = dataset['sample_weights']
    X_scaled, y_train, sample_weight = _append_augmented_training_rows(
        X_scaled,
        y_train,
        scaler,
        dataset.get('X_aug'),
        dataset.get('y_aug'),
        dataset.get('groups_aug'),
        dataset['groups'],
        sample_weight=sample_weight,
    )
    feature_bounds = (
        normalized_feature_bounds(scaler, dataset['feature_names'])
        if feature_augmentation else None
    )
    with suppress_stderr():
        model = train_model(
            X_scaled,
            y_train,
            hidden_layers=list(hidden_layers),
            max_epochs=DEFAULT_MAX_EPOCHS,
            fp_weight=fp_weight,
            sample_weight=sample_weight,
            batch_size=batch_size,
            seed=derive_seed(seed, 10_000),
            feature_augmentation=feature_augmentation,
            feature_bounds=feature_bounds,
        )

    sample = X_scaled[:1].astype(np.float32)
    for _ in range(10):
        predict_probabilities(model, sample)
    n_bench = 1000
    bench_start = perf_counter()
    for _ in range(n_bench):
        predict_probabilities(model, sample)
    inference_us = (perf_counter() - bench_start) / n_bench * 1e6

    paired = evaluate_paired_gate(model, scaler, dataset['feature_names'])
    result = {
        'name': name,
        'seed': int(seed),
        'fp_weight': float(fp_weight),
        'layers': list(hidden_layers),
        'architecture': ' -> '.join(map(str, stats['layer_sizes'])),
        'params': int(stats['params']),
        'weight_kb': float(stats['weight_kb']),
        'flops': int(stats['flops']),
        'inference_us': float(inference_us),
        'cv': slim_cv_result(cv),
        'paired': paired,
    }
    print(
        f"{name} | OOF={result['cv']['oof_f1']:.1f}% | "
        f"paired pass={paired['pass_count']} maxFP={paired['max_fp_rate']:.1f}% "
        f"worstRecall={paired['worst_chip_recall']:.1f}% "
        f"worstF1={paired['worst_chip_f1']:.1f}% | "
        f"inf={inference_us:.1f} us"
    )
    return result


def build_feature_ablation_dataset(dataset, feature_name):
    """Return a dataset view with one or more ``+``-joined features removed."""
    feature_names = list(dataset['feature_names'])
    removed_features = [
        name.strip() for name in str(feature_name).split('+') if name.strip()
    ]
    unknown = [name for name in removed_features if name not in feature_names]
    if not removed_features or unknown:
        raise ValueError(
            f"Unknown ablation feature(s) '{', '.join(unknown or removed_features)}'. "
            f"Available features: {', '.join(feature_names)}"
        )
    removed_indices = [feature_names.index(name) for name in removed_features]
    candidate = dict(dataset)
    candidate['X'] = np.delete(dataset['X'], removed_indices, axis=1)
    if dataset.get('X_aug') is not None:
        candidate['X_aug'] = np.delete(
            dataset['X_aug'], removed_indices, axis=1
        )
    candidate['feature_names'] = [
        name for idx, name in enumerate(feature_names)
        if idx not in removed_indices
    ]
    return candidate


def _print_feature_ablation_comparison(baseline, candidate):
    """Print the CV and paired real-data deltas for a targeted feature ablation."""
    rows = (
        ('Blocked OOF F1', baseline['cv']['oof_f1'], candidate['cv']['oof_f1'], '%'),
        ('Fold F1', baseline['cv']['f1_mean'], candidate['cv']['f1_mean'], '%'),
        ('Fold recall', baseline['cv']['recall_mean'], candidate['cv']['recall_mean'], '%'),
        ('Fold FP rate', baseline['cv']['fp_rate_mean'], candidate['cv']['fp_rate_mean'], '%'),
        (
            'Worst-session recall',
            baseline['cv']['worst_session_recall'],
            candidate['cv']['worst_session_recall'],
            '%',
        ),
        (
            'Worst-session FP rate',
            baseline['cv']['worst_session_fp_rate'],
            candidate['cv']['worst_session_fp_rate'],
            '%',
        ),
        ('Paired mean F1', baseline['paired']['mean_f1'], candidate['paired']['mean_f1'], '%'),
        ('Paired worst-chip F1', baseline['paired']['worst_chip_f1'], candidate['paired']['worst_chip_f1'], '%'),
        ('Paired max FP rate', baseline['paired']['max_fp_rate'], candidate['paired']['max_fp_rate'], '%'),
        (
            'Paired worst-chip recall',
            baseline['paired']['worst_chip_recall'],
            candidate['paired']['worst_chip_recall'],
            '%',
        ),
    )
    print("\n" + "=" * 82)
    print("  TARGETED FEATURE ABLATION COMPARISON")
    print("=" * 82)
    print(f"{'Metric':<29} {'Baseline':>14} {'Candidate':>14} {'Delta':>14}")
    print("-" * 82)
    for label, baseline_value, candidate_value, suffix in rows:
        delta = candidate_value - baseline_value
        if suffix:
            print(
                f"{label:<29} {baseline_value:>13.2f}{suffix} "
                f"{candidate_value:>13.2f}{suffix} {delta:>+13.2f}{suffix}"
            )
        else:
            print(
                f"{label:<29} {int(baseline_value):>14d} "
                f"{int(candidate_value):>14d} {int(delta):>+14d}"
            )
    print("-" * 82)


def _feature_ablation_rank_key(result):
    """Rank one targeted ablation result with paired metrics first."""
    cv = result['cv']
    return _paired_gate_key(result['paired']) + (
        cv['worst_session_recall'],
        cv['worst_chip_recall'],
        -cv['worst_session_fp_rate'],
        cv['oof_f1'],
        cv['f1_mean'],
    )


def _non_regression_failure(replay, metric, candidate_value, baseline_value,
                            margin=0.0, eval_count=None):
    """Describe one blocking comparison, in the units the gate reasons about."""
    entry = {
        'replay': replay,
        'metric': metric,
        'candidate': candidate_value,
        'baseline': baseline_value,
        'margin': margin,
    }
    if eval_count:
        # Percentages hide how small these differences are; the gate margin is
        # one evaluation, so report the evaluation count that produced them.
        entry['eval_count'] = int(eval_count)
        entry['candidate_evaluations'] = round(candidate_value * eval_count / 100.0)
        entry['baseline_evaluations'] = round(baseline_value * eval_count / 100.0)
    return entry


FP_SEED_NOISE_EVALUATIONS = 5


RECALL_SEED_NOISE_EVALUATIONS = 1


def paired_non_regression_failures(candidate, baseline, tolerance=0.25):
    """List every comparison blocking a candidate; empty when nothing does.

    Reported rather than merely counted, because a rejection that does not say
    which replay moved cannot be argued with.
    """
    if candidate['pass_count'] != baseline['pass_count']:
        if candidate['pass_count'] > baseline['pass_count']:
            return []
        return [_non_regression_failure(
            '<paired>', 'pass_count', candidate['pass_count'], baseline['pass_count'])]
    candidate_rows = candidate.get('by_chip') or {}
    baseline_rows = baseline.get('by_chip') or {}
    shared_keys = sorted(set(candidate_rows).intersection(baseline_rows))
    failures = []
    if shared_keys and len(shared_keys) == len(candidate_rows) == len(baseline_rows):
        for key in shared_keys:
            candidate_row = candidate_rows[key]
            baseline_row = baseline_rows[key]
            if candidate_row.get('effective_alarms', 0) > baseline_row.get('effective_alarms', 0):
                failures.append(_non_regression_failure(
                    key, 'effective_alarms',
                    candidate_row.get('effective_alarms', 0),
                    baseline_row.get('effective_alarms', 0)))
                continue
            if candidate_row.get('low_rssi') or baseline_row.get('low_rssi'):
                # Weak-link replays are stress diagnostics: at -75/-77 dBm
                # recall and FP jitter by whole events between equally healthy
                # models, so within the absolute stress targets only the alarm
                # count ratchets against the baseline.
                continue
            fp_evals = max(
                int(candidate_row.get('static_presence_eval_count', 0)),
                int(baseline_row.get('static_presence_eval_count', 0)),
            )
            recall_evals = max(
                int(candidate_row.get('motion_eval_count', 0)),
                int(baseline_row.get('motion_eval_count', 0)),
            )
            fp_margin = FP_SEED_NOISE_EVALUATIONS * 100.0 / max(fp_evals, 1)
            recall_margin = RECALL_SEED_NOISE_EVALUATIONS * 100.0 / max(recall_evals, 1)
            if candidate_row.get('fp_rate', 100.0) > baseline_row.get('fp_rate', 100.0) + fp_margin + 1e-9:
                failures.append(_non_regression_failure(
                    key, 'fp_rate', candidate_row.get('fp_rate', 100.0),
                    baseline_row.get('fp_rate', 100.0), fp_margin, fp_evals))
            if candidate_row.get('recall', 0.0) < baseline_row.get('recall', 0.0) - recall_margin - 1e-9:
                failures.append(_non_regression_failure(
                    key, 'recall', candidate_row.get('recall', 0.0),
                    baseline_row.get('recall', 0.0), recall_margin, recall_evals))
        return failures
    aggregate_checks = (
        ('max_fp_rate', candidate['max_fp_rate'], baseline['max_fp_rate'] + tolerance,
         candidate['max_fp_rate'] <= baseline['max_fp_rate'] + tolerance),
        ('worst_chip_recall', candidate['worst_chip_recall'], baseline['worst_chip_recall'] - tolerance,
         candidate['worst_chip_recall'] >= baseline['worst_chip_recall'] - tolerance),
        ('worst_chip_f1', candidate['worst_chip_f1'], baseline['worst_chip_f1'] - tolerance,
         candidate['worst_chip_f1'] >= baseline['worst_chip_f1'] - tolerance),
        ('total_effective_alarms', candidate.get('total_effective_alarms', 0),
         baseline.get('total_effective_alarms', 0),
         candidate.get('total_effective_alarms', 0) <= baseline.get('total_effective_alarms', 0)),
    )
    for metric, candidate_value, limit, ok in aggregate_checks:
        if not ok:
            failures.append(_non_regression_failure(
                '<aggregate>', metric, candidate_value, limit, tolerance))
    return failures


def paired_result_non_regression(candidate, baseline, tolerance=0.25):
    """Preserve each paired replay within its measured seed-noise margin."""
    return not paired_non_regression_failures(candidate, baseline, tolerance)


def format_non_regression_failures(failures, indent='    '):
    """Render blocking comparisons as one readable line each."""
    lines = []
    for item in failures:
        detail = (
            f"{item['candidate']:.4g} vs {item['baseline']:.4g} "
            f"(margin {item['margin']:.4g})"
        )
        if 'candidate_evaluations' in item:
            detail += (
                f" = {item['candidate_evaluations']} vs "
                f"{item['baseline_evaluations']} of {item['eval_count']} evaluations"
            )
        lines.append(f"{indent}{item['replay']} | {item['metric']}: {detail}")
    return '\n'.join(lines)


def deployment_candidate_beats_baseline(candidate, baseline):
    """Compare single-run candidates with safety first and robust CV leading."""
    if not paired_result_non_regression(candidate['paired'], baseline['paired']):
        return False
    return compare_robust_cv(candidate['cv'], baseline['cv'])['passed']


def experiment_feature_ablation(feature_name, seed=None,
                                scaler_mode=DEFAULT_SCALER_MODE,
                                batch_size=DEFAULT_BATCH_SIZE,
                                fp_weight=DEFAULT_FP_WEIGHT,
                                environment_filter=None,
                                excluded_chips=None,
                                positive_chip_boost=None,
                                use_cache=True,
                                augment=False,
                                timing_quality_policy=DEFAULT_TIMING_QUALITY_POLICY,
                                timing_warn_weight=DEFAULT_TIMING_WARN_WEIGHT):
    """Compare the production baseline against feature removals without exporting."""
    environment_filter = parse_environment_filter(environment_filter)
    excluded_chips = parse_chip_filter(excluded_chips)
    positive_chip_boost = parse_positive_chip_boost(positive_chip_boost)
    removed_features = [
        name.strip() for name in str(feature_name).split(',') if name.strip()
    ]
    if not removed_features:
        print("Error: --ablation-feature requires at least one feature name")
        return 1
    if len(set(removed_features)) != len(removed_features):
        print("Error: --ablation-feature contains duplicate names")
        return 1
    augment_components, feature_augmentation, packet_augmentation = (
        resolve_training_augmentation(augment)
    )
    try:
        ensure_torch_available()
        seed = resolve_training_seed(seed, trailing_newline=True)
        set_global_determinism(seed, torch_module=torch)
    except (ImportError, RuntimeError, ValueError) as exc:
        print(f"Error: {exc}")
        return 1

    print("\n" + "=" * 70)
    print("  TARGETED FEATURE ABLATION")
    print("=" * 70)
    print(f"Removed features: {', '.join(removed_features)}")
    print(f"Seed: {seed}")
    print(
        "Augmentation: "
        + format_augmentation_config(
            feature_augmentation,
            packet_augmentation,
            components=augment_components,
        )
    )
    print("Artifacts: unchanged")

    matrix, _ = load_training_matrix(
        environment_filter=environment_filter,
        excluded_chips=excluded_chips,
        feature_names=TRAINING_FEATURES,
        use_cache=use_cache,
        timing_quality_policy=timing_quality_policy,
        timing_warn_weight=timing_warn_weight,
    )
    if not matrix['stats']['chips']:
        print("Error: No datasets found in data/")
        return 1

    augmented_matrix = None
    if packet_augmentation:
        print("Loading packet-augmented training matrix...")
        augmented_matrix, _ = load_training_matrix(
            environment_filter=environment_filter,
            excluded_chips=excluded_chips,
            feature_names=TRAINING_FEATURES,
            use_cache=use_cache,
            packet_augmentation=packet_augmentation,
            augmentation_seeds=training_packet_augmentation_seeds(
                packet_augmentation
            ),
            timing_quality_policy=timing_quality_policy,
            timing_warn_weight=timing_warn_weight,
        )

    sample_weights, _ = apply_positive_chip_boost(
        matrix['sample_weights'],
        matrix['sample_context'],
        matrix['y'],
        positive_chip_boost,
    )
    baseline_dataset = {
        'X': np.asarray(matrix['X'], dtype=np.float32),
        'y': np.asarray(matrix['y'], dtype=np.int8),
        'feature_names': list(matrix['feature_names']),
        'sample_context': matrix['sample_context'],
        'sample_weights': np.asarray(sample_weights, dtype=np.float32),
        'groups': matrix['sample_context'][DEFAULT_PRIMARY_GROUP_KEY],
    }
    if augmented_matrix is not None:
        baseline_dataset.update({
            'X_aug': np.asarray(augmented_matrix['X'], dtype=np.float32),
            'y_aug': np.asarray(augmented_matrix['y'], dtype=np.int8),
            'groups_aug': augmented_matrix['sample_context'][
                DEFAULT_PRIMARY_GROUP_KEY
            ],
        })

    for removed_feature in removed_features:
        requested = [
            name.strip() for name in removed_feature.split('+') if name.strip()
        ]
        unknown = [
            name for name in requested
            if name not in baseline_dataset['feature_names']
        ]
        if not requested or unknown:
            print(
                "Error: unknown ablation feature(s) '"
                + ', '.join(unknown or requested)
                + "'. "
                "Available features: "
                + ', '.join(baseline_dataset['feature_names'])
            )
            return 1

    baseline = evaluate_architecture_candidate(
        'production baseline',
        DEFAULT_HIDDEN_LAYERS,
        seed,
        baseline_dataset,
        scaler_mode,
        batch_size,
        fp_weight,
        feature_augmentation=feature_augmentation or None,
    )
    for removed_feature in removed_features:
        candidate_dataset = build_feature_ablation_dataset(
            baseline_dataset,
            removed_feature,
        )
        candidate = evaluate_architecture_candidate(
            f"Drop {removed_feature}",
            DEFAULT_HIDDEN_LAYERS,
            seed,
            candidate_dataset,
            scaler_mode,
            batch_size,
            fp_weight,
            feature_augmentation=feature_augmentation or None,
        )
        _print_feature_ablation_comparison(baseline, candidate)
        if deployment_candidate_beats_baseline(candidate, baseline):
            print(
                "Paired-first result: candidate ranks above the production "
                "baseline for this seed."
            )
        else:
            print(
                "Paired-first result: candidate does not beat the production "
                "baseline for this seed."
            )
    return 0


def experiment_architectures(scaler_mode=DEFAULT_SCALER_MODE,
                             batch_size=DEFAULT_BATCH_SIZE,
                             fp_weight=DEFAULT_FP_WEIGHT,
                             feature_names=None,
                             environment_filter=None,
                             excluded_chips=None,
                             architectures=None,
                             positive_chip_boost=None,
                             output_path=DEFAULT_EXPERIMENT_OUTPUT,
                             use_cache=True,
                             timing_quality_policy=DEFAULT_TIMING_QUALITY_POLICY,
                             timing_warn_weight=DEFAULT_TIMING_WARN_WEIGHT):
    """Run the FP-first architecture campaign without changing artifacts."""
    environment_filter = parse_environment_filter(environment_filter)
    excluded_chips = parse_chip_filter(excluded_chips)
    positive_chip_boost = parse_positive_chip_boost(positive_chip_boost)
    feature_names = list(feature_names or TRAINING_FEATURES)
    architectures = normalize_architecture_specs(architectures or DEFAULT_ARCHITECTURE_SWEEP)

    static_presence_layers = tuple(DEFAULT_HIDDEN_LAYERS)
    if static_presence_layers not in {tuple(spec['layers']) for spec in architectures}:
        architectures.insert(0, {
            'name': f"Current default ({format_hidden_layers(DEFAULT_HIDDEN_LAYERS)})",
            'layers': list(DEFAULT_HIDDEN_LAYERS),
        })
    else:
        architectures = sorted(
            architectures,
            key=lambda spec: tuple(spec['layers']) != static_presence_layers,
        )
    static_presence_name = next(
        spec['name'] for spec in architectures if tuple(spec['layers']) == static_presence_layers
    )
    screening_seed = read_exported_seed() or DEFAULT_EXPERIMENT_SCREENING_SEED

    try:
        ensure_torch_available()
        torch_device_label = describe_torch_device()
    except ImportError as exc:
        print(f"Error: Missing dependency - {exc}")
        return 1
    except (RuntimeError, ValueError) as exc:
        print(f"Error: {exc}")
        return 1

    print("\n" + "=" * 70)
    print("  FP-FIRST MLP ARCHITECTURE CAMPAIGN")
    print("=" * 70)
    print(f"Scaler: {scaler_mode}")
    print(f"Batch size: {batch_size}")
    print(f"Torch device: {torch_device_label}")
    print(f"FP weight: {fp_weight}")
    print(f"Feature set: {', '.join(feature_names)}")
    print(f"Screening seed: {screening_seed}")
    print(
        "Architectures: "
        + ', '.join(f"{spec['name']} [{format_hidden_layers(spec['layers'])}]" for spec in architectures)
    )
    if environment_filter is not None:
        print(f"Environment filter: {', '.join(sorted(environment_filter))}")
    if excluded_chips is not None:
        print(f"Excluded chips: {', '.join(sorted(excluded_chips))}")
    if positive_chip_boost is not None:
        print(
            "Positive chip boost: "
            + ', '.join(f"{chip}={factor:.2f}" for chip, factor in sorted(positive_chip_boost.items()))
        )

    print("\nLoading training matrix...")
    matrix, _ = load_training_matrix(
        environment_filter=environment_filter,
        excluded_chips=excluded_chips,
        feature_names=feature_names,
        use_cache=use_cache,
        timing_quality_policy=timing_quality_policy,
        timing_warn_weight=timing_warn_weight,
    )
    stats = matrix['stats']
    if not stats['chips']:
        print("Error: No datasets found in data/")
        return 1

    print(f"  Chips: {', '.join(stats['chips'])}")
    print(f"  Session groups: {len(stats.get('session_groups', []))}")
    print(f"  Total: {stats['total']} packets")

    X = matrix['X']
    y = matrix['y']
    feature_names = matrix['feature_names']
    sample_context = matrix['sample_context']
    sample_weights = matrix['sample_weights']
    sample_weights, boost_summary = apply_positive_chip_boost(
        sample_weights,
        sample_context,
        y,
        positive_chip_boost,
    )
    dataset = {
        'X': np.asarray(X, dtype=np.float32),
        'y': np.asarray(y, dtype=np.int8),
        'feature_names': list(feature_names),
        'sample_context': sample_context,
        'sample_weights': np.asarray(sample_weights, dtype=np.float32),
        'groups': sample_context[DEFAULT_PRIMARY_GROUP_KEY],
        'boost_summary': boost_summary,
    }
    print(f"  Samples: {len(dataset['X'])}")
    print(f"  Features: {len(dataset['feature_names'])}")
    print(f"  Class balance: IDLE={np.sum(dataset['y']==0)}, MOTION={np.sum(dataset['y']==1)}")

    results = {
        'config': {
            'scaler': scaler_mode,
            'batch_size': batch_size,
            'fp_weight': fp_weight,
            'environment': sorted(environment_filter) if environment_filter else None,
            'exclude_chip': sorted(excluded_chips) if excluded_chips else [],
            'positive_chip_boost': positive_chip_boost,
            'timing_quality_policy': timing_quality_policy,
            'timing_warn_weight': float(timing_warn_weight),
            'training_sample_contract': TRAINING_SAMPLE_CONTRACT,
            'screening_seed': screening_seed,
            'initial_seeds': list(DEFAULT_EXPERIMENT_INITIAL_SEEDS),
            'final_seeds': list(DEFAULT_EXPERIMENT_FINAL_SEEDS),
            'architectures': architectures,
            'feature_names': list(feature_names),
        },
        'screening': [],
        'seed_filter': [],
        'seed_finalists': [],
        'promotion': None,
    }

    print("\n== Single-seed screening ==")
    screening_results = []
    for spec in architectures:
        run = evaluate_architecture_candidate(
            spec['name'],
            spec['layers'],
            screening_seed,
            dataset,
            scaler_mode,
            batch_size,
            fp_weight,
        )
        screening_results.append(run)
        results['screening'] = screening_results
        write_json_results(output_path, results)

    challengers = [
        item for item in sorted(screening_results, key=architecture_campaign_rank_key)
        if item['name'] != static_presence_name
    ][:2]
    finalists = [static_presence_name] + [item['name'] for item in challengers]
    print(f"\nFinalists for 3-seed filter: {', '.join(finalists)}")

    specs_by_name = {spec['name']: spec for spec in architectures}

    print("\n== 3-seed robustness filter ==")
    seed_filter = []
    for name in finalists:
        spec = specs_by_name[name]
        runs = [
            evaluate_architecture_candidate(
                name,
                spec['layers'],
                seed,
                dataset,
                scaler_mode,
                batch_size,
                fp_weight,
            )
            for seed in DEFAULT_EXPERIMENT_INITIAL_SEEDS
        ]
        summary = aggregate_architecture_runs(name, runs)
        seed_filter.append(summary)
        results['seed_filter'] = seed_filter
        write_json_results(output_path, results)
        print(
            f"{name} | median paired pass={summary['median_paired_pass_count']:.1f} | "
            f"median maxFP={summary['median_paired_max_fp_rate']:.1f}% | "
            f"median worstRecall={summary['median_paired_worst_chip_recall']:.1f}% | "
            f"median worstF1={summary['median_paired_worst_chip_f1']:.1f}% | "
            f"median OOF={summary['median_oof_f1']:.1f}%"
        )

    challenger_summaries = [
        item for item in sorted(seed_filter, key=aggregate_architecture_rank_key)
        if item['name'] != static_presence_name
    ]
    head_to_head = [static_presence_name]
    if challenger_summaries:
        head_to_head.append(challenger_summaries[0]['name'])
    print(f"\n5-seed head-to-head: {', '.join(head_to_head)}")

    print("\n== 5-seed final comparison ==")
    seed_finalists = []
    for name in head_to_head:
        spec = specs_by_name[name]
        runs = [
            evaluate_architecture_candidate(
                name,
                spec['layers'],
                seed,
                dataset,
                scaler_mode,
                batch_size,
                fp_weight,
            )
            for seed in DEFAULT_EXPERIMENT_FINAL_SEEDS
        ]
        summary = aggregate_architecture_runs(name, runs)
        seed_finalists.append(summary)
        results['seed_finalists'] = seed_finalists
        write_json_results(output_path, results)
        print(
            f"{name} | median paired pass={summary['median_paired_pass_count']:.1f} | "
            f"median maxFP={summary['median_paired_max_fp_rate']:.1f}% | "
            f"median worstRecall={summary['median_paired_worst_chip_recall']:.1f}% | "
            f"median worstF1={summary['median_paired_worst_chip_f1']:.1f}% | "
            f"median OOF={summary['median_oof_f1']:.1f}%"
        )

    seed_finalists = sorted(seed_finalists, key=aggregate_architecture_rank_key)
    static_presence_final = next(item for item in seed_finalists if item['name'] == static_presence_name)
    winner = seed_finalists[0]
    promote_candidate = (
        winner['name'] != static_presence_name
        and architecture_candidate_beats_baseline(winner, static_presence_final)
    )
    results['promotion'] = {
        'winner': winner['name'],
        'static_presence': static_presence_final['name'],
        'decision': f"promote {winner['name']}" if promote_candidate else f"keep {static_presence_name}",
        'clear_winner': bool(promote_candidate),
        'summary': winner,
        'static_presence_summary': static_presence_final,
        'output_path': str(output_path),
    }
    write_json_results(output_path, results)

    if not promote_candidate:
        print(f"\nDecision: keep {static_presence_name}")
        return 0

    print(f"\nDecision: {winner['name']} beats {static_presence_name} on paired ranking")
    return 0


def experiment_fp_weights(fp_weights=None, scaler_mode=DEFAULT_SCALER_MODE,
                          batch_size=DEFAULT_BATCH_SIZE, hidden_layers=None,
                          feature_names=None,
                          environment_filter=None, excluded_chips=None,
                          positive_chip_boost=None,
                          output_path=DEFAULT_FP_WEIGHT_EXPERIMENT_OUTPUT,
                          use_cache=True,
                          timing_quality_policy=DEFAULT_TIMING_QUALITY_POLICY,
                          timing_warn_weight=DEFAULT_TIMING_WARN_WEIGHT):
    """Run a gated, multi-seed FP-weight campaign."""
    weights = parse_fp_weight_sweep(fp_weights or DEFAULT_FP_WEIGHT_SWEEP)
    if DEFAULT_FP_WEIGHT not in weights:
        weights.insert(0, DEFAULT_FP_WEIGHT)
    else:
        weights = [DEFAULT_FP_WEIGHT] + [value for value in weights if value != DEFAULT_FP_WEIGHT]
    hidden_layers = list(hidden_layers or DEFAULT_HIDDEN_LAYERS)
    feature_names = list(feature_names or TRAINING_FEATURES)
    environment_filter = parse_environment_filter(environment_filter)
    excluded_chips = parse_chip_filter(excluded_chips)
    positive_chip_boost = parse_positive_chip_boost(positive_chip_boost)
    screening_seed = read_exported_seed() or DEFAULT_EXPERIMENT_SCREENING_SEED

    try:
        ensure_torch_available()
        torch_device_label = describe_torch_device()
    except (ImportError, RuntimeError, ValueError) as exc:
        print(f"Error: {exc}")
        return 1

    print("\n" + "=" * 70)
    print("  FP-WEIGHT CAMPAIGN")
    print("=" * 70)
    print(f"Architecture: {format_hidden_layers(hidden_layers)}")
    print(f"Scaler: {scaler_mode}")
    print(f"Batch size: {batch_size}")
    print(f"Torch device: {torch_device_label}")
    print(f"Screening seed: {screening_seed}")
    print(f"FP weights: {', '.join(map(str, weights))}")
    print(f"Feature set: {', '.join(feature_names)}")
    print("Artifacts: unchanged during evaluation")

    matrix, _ = load_training_matrix(
        environment_filter=environment_filter,
        excluded_chips=excluded_chips,
        feature_names=feature_names,
        use_cache=use_cache,
        timing_quality_policy=timing_quality_policy,
        timing_warn_weight=timing_warn_weight,
    )
    if not matrix['stats']['chips']:
        print("Error: No datasets found in data/")
        return 1
    sample_weights, _ = apply_positive_chip_boost(
        matrix['sample_weights'], matrix['sample_context'], matrix['y'], positive_chip_boost,
    )
    dataset = {
        'X': np.asarray(matrix['X'], dtype=np.float32),
        'y': np.asarray(matrix['y'], dtype=np.int8),
        'feature_names': list(matrix['feature_names']),
        'sample_context': matrix['sample_context'],
        'sample_weights': np.asarray(sample_weights, dtype=np.float32),
        'groups': matrix['sample_context'][DEFAULT_PRIMARY_GROUP_KEY],
    }

    def evaluate(weight, seed):
        return evaluate_architecture_candidate(
            f"fp_weight={weight:g}", hidden_layers, seed, dataset,
            scaler_mode, batch_size, weight,
        )

    results = {
        'config': {
            'weights': weights,
            'baseline_weight': DEFAULT_FP_WEIGHT,
            'hidden_layers': hidden_layers,
            'scaler': scaler_mode,
            'batch_size': batch_size,
            'feature_names': list(feature_names),
            'timing_quality_policy': timing_quality_policy,
            'timing_warn_weight': float(timing_warn_weight),
            'training_sample_contract': TRAINING_SAMPLE_CONTRACT,
            'screening_seed': screening_seed,
            'initial_seeds': list(DEFAULT_EXPERIMENT_INITIAL_SEEDS),
            'final_seeds': list(DEFAULT_EXPERIMENT_FINAL_SEEDS),
        },
        'screening': [],
        'seed_filter': [],
        'seed_finalists': [],
        'promotion': None,
    }

    print("\n== Single-seed screening ==")
    for weight in weights:
        results['screening'].append(evaluate(weight, screening_seed))
        write_json_results(output_path, results)
    baseline_name = f"fp_weight={DEFAULT_FP_WEIGHT:g}"
    challengers = [
        run for run in sorted(results['screening'], key=architecture_campaign_rank_key)
        if run['name'] != baseline_name
    ][:2]
    finalist_weights = [DEFAULT_FP_WEIGHT] + [run['fp_weight'] for run in challengers]

    print("\n== 3-seed robustness filter ==")
    for weight in finalist_weights:
        runs = [evaluate(weight, seed) for seed in DEFAULT_EXPERIMENT_INITIAL_SEEDS]
        summary = aggregate_architecture_runs(f"fp_weight={weight:g}", runs)
        summary['fp_weight'] = float(weight)
        results['seed_filter'].append(summary)
        write_json_results(output_path, results)

    challengers = [
        item for item in sorted(results['seed_filter'], key=aggregate_architecture_rank_key)
        if item['name'] != baseline_name
    ]
    head_to_head = [DEFAULT_FP_WEIGHT]
    if challengers:
        head_to_head.append(challengers[0]['fp_weight'])

    print("\n== 5-seed final comparison ==")
    for weight in head_to_head:
        runs = [evaluate(weight, seed) for seed in DEFAULT_EXPERIMENT_FINAL_SEEDS]
        summary = aggregate_architecture_runs(f"fp_weight={weight:g}", runs)
        summary['fp_weight'] = float(weight)
        results['seed_finalists'].append(summary)
        write_json_results(output_path, results)

    finalists = sorted(results['seed_finalists'], key=aggregate_architecture_rank_key)
    baseline = next(item for item in finalists if item['name'] == baseline_name)
    winner = finalists[0]
    promote_candidate = (
        winner['name'] != baseline_name
        and architecture_candidate_beats_baseline(winner, baseline)
    )
    results['promotion'] = {
        'winner': winner['name'],
        'baseline': baseline_name,
        'decision': f"promote {winner['name']}" if promote_candidate else f"keep {baseline_name}",
        'clear_winner': bool(promote_candidate),
        'summary': winner,
        'baseline_summary': baseline,
    }
    write_json_results(output_path, results)
    print(f"\nDecision: {results['promotion']['decision']}")
    return 0
