# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Training preprocessing, scalers, and normalized augmentation."""

from __future__ import annotations

from tools.lib.bootstrap import setup_paths

setup_paths()

import numpy as np

DEFAULT_SCALER_MODE = 'standard'


DEFAULT_CLIP_PERCENTILES = (1.0, 99.0)


DEFAULT_BLOCK_GROUP_KEY = 'source_file'


class ClippedStandardScaler:
    """Clip heavy tails before applying standard z-score normalization."""

    def __init__(self, lower_percentile=1.0, upper_percentile=99.0):
        self.lower_percentile = float(lower_percentile)
        self.upper_percentile = float(upper_percentile)
        self.lower_bounds_ = None
        self.upper_bounds_ = None
        self.mean_ = None
        self.scale_ = None

    def fit(self, X):
        X = np.asarray(X, dtype=np.float32)
        self.lower_bounds_ = np.percentile(X, self.lower_percentile, axis=0)
        self.upper_bounds_ = np.percentile(X, self.upper_percentile, axis=0)
        clipped = np.clip(X, self.lower_bounds_, self.upper_bounds_)
        self.mean_ = clipped.mean(axis=0)
        self.scale_ = clipped.std(axis=0)
        self.scale_[self.scale_ < 1e-6] = 1.0
        return self

    def transform(self, X):
        X = np.asarray(X, dtype=np.float32)
        clipped = np.clip(X, self.lower_bounds_, self.upper_bounds_)
        return (clipped - self.mean_) / self.scale_

    def fit_transform(self, X):
        return self.fit(X).transform(X)


class SessionBalancedRobustScaler:
    """Robust affine scaler fitted on equal-sized session/class strata."""

    def __init__(self, max_samples_per_stratum=2048):
        self.max_samples_per_stratum = int(max_samples_per_stratum)
        self.center_ = None
        self.scale_ = None
        self.selected_indices_ = None

    def fit(self, X, y=None, groups=None):
        X = np.asarray(X, dtype=np.float32)
        if y is None or groups is None:
            raise ValueError("session_balanced_robust requires labels and session groups")
        y = np.asarray(y)
        groups = np.asarray(groups).astype(str)
        if len(X) != len(y) or len(X) != len(groups):
            raise ValueError("scaler inputs must have matching rows")

        strata = {}
        for idx, key in enumerate(zip(groups.tolist(), y.tolist(), strict=True)):
            strata.setdefault((str(key[0]), int(key[1])), []).append(idx)
        non_empty = [indices for indices in strata.values() if indices]
        if not non_empty:
            raise ValueError("session_balanced_robust received no samples")
        per_stratum = min(self.max_samples_per_stratum, min(map(len, non_empty)))
        selected = []
        for key in sorted(strata):
            indices = np.asarray(strata[key], dtype=np.int64)
            if len(indices) <= per_stratum:
                selected.extend(indices.tolist())
            else:
                positions = np.linspace(0, len(indices) - 1, per_stratum, dtype=np.int64)
                selected.extend(indices[positions].tolist())
        self.selected_indices_ = np.asarray(selected, dtype=np.int64)
        balanced = X[self.selected_indices_]
        self.center_ = np.median(balanced, axis=0).astype(np.float32)
        q25, q75 = np.percentile(balanced, (25.0, 75.0), axis=0)
        self.scale_ = np.asarray(q75 - q25, dtype=np.float32)
        self.scale_[self.scale_ < 1e-6] = 1.0
        return self

    def transform(self, X):
        if self.center_ is None or self.scale_ is None:
            raise ValueError("session_balanced_robust scaler is not fitted")
        return (np.asarray(X, dtype=np.float32) - self.center_) / self.scale_

    def fit_transform(self, X, y=None, groups=None):
        return self.fit(X, y=y, groups=groups).transform(X)


def build_preprocessor(mode=DEFAULT_SCALER_MODE, clip_percentiles=DEFAULT_CLIP_PERCENTILES):
    """Build the feature normalization object used in CV and final training."""
    from sklearn.preprocessing import RobustScaler, StandardScaler

    if mode == 'standard':
        return StandardScaler()
    if mode == 'robust':
        return RobustScaler()
    if mode == 'session_balanced_robust':
        return SessionBalancedRobustScaler()
    if mode == 'clipped_standard':
        return ClippedStandardScaler(*clip_percentiles)
    raise ValueError(f"Unsupported scaler mode: {mode}")


def fit_preprocessor(preprocessor, X, y=None, sample_context=None):
    """Fit a scaler with fold-local metadata when the mode requires it."""
    if isinstance(preprocessor, SessionBalancedRobustScaler):
        groups = None if sample_context is None else sample_context.get('session_group')
        preprocessor.fit(X, y=y, groups=groups)
    elif not hasattr(preprocessor, 'fit') and hasattr(preprocessor, 'fit_transform'):
        # Preserve compatibility with lightweight test doubles and third-party
        # preprocessors that only expose fit_transform.
        preprocessor.fit_transform(X)
        if not hasattr(preprocessor, 'transform'):
            preprocessor.transform = lambda values: np.asarray(values)
    else:
        preprocessor.fit(X)
    return preprocessor


def normalized_feature_bounds(preprocessor, feature_names):
    """Return normalized bounds used to keep augmented feature rows valid."""
    center, scale = get_preprocessor_arrays(preprocessor, allow_clipping=True)
    lower = np.full(len(feature_names), -np.inf, dtype=np.float32)
    upper = np.full(len(feature_names), np.inf, dtype=np.float32)
    for name in (
        'turb_mad_over_mean',
        'turb_iqr_over_mean',
        'turb_p95_over_mean',
        'turb_mad_over_mean_aggr',
        'turb_iqr_over_mean_aggr',
        'turb_p95_over_mean_aggr',
    ):
        if name in feature_names:
            idx = feature_names.index(name)
            lower[idx] = (0.0 - center[idx]) / scale[idx]
    if 'turb_autocorr' in feature_names:
        idx = feature_names.index('turb_autocorr')
        lower[idx] = (-1.0 - center[idx]) / scale[idx]
        upper[idx] = (1.0 - center[idx]) / scale[idx]
    return lower, upper


def augment_normalized_features(X, config, seed, bounds=None, apply_fraction=0.5):
    """Deterministically augment normalized training rows only."""
    X = np.asarray(X, dtype=np.float32)
    if not config:
        return X.copy()
    rng = np.random.default_rng(seed)
    result = X.copy()
    selected = rng.random(len(result)) < float(apply_fraction)
    if not np.any(selected):
        return result
    row_count, feature_count = int(np.sum(selected)), result.shape[1]
    noise_sigma = float(config.get('noise_sigma', 0.0))
    if noise_sigma > 0.0:
        result[selected] += rng.normal(0.0, noise_sigma, size=(row_count, feature_count)).astype(np.float32)
    jitter_sigma = float(config.get('jitter_sigma', 0.0))
    if jitter_sigma > 0.0:
        jitter = np.empty((row_count, feature_count), dtype=np.float32)
        jitter[:, :min(3, feature_count)] = rng.normal(0.0, jitter_sigma, size=(row_count, 1))
        if feature_count > 3:
            jitter[:, 3:] = rng.normal(0.0, jitter_sigma, size=(row_count, 1))
        result[selected] += jitter
    dropout_probability = float(config.get('dropout_probability', 0.0))
    if dropout_probability > 0.0:
        dropout = rng.random((row_count, feature_count)) < dropout_probability
        selected_rows = result[selected]
        selected_rows[dropout] = 0.0
        result[selected] = selected_rows
    if bounds is not None:
        lower, upper = bounds
        result = np.maximum(result, np.asarray(lower, dtype=np.float32))
        result = np.minimum(result, np.asarray(upper, dtype=np.float32))
    return result


def get_preprocessor_arrays(preprocessor, *, allow_clipping=False):
    """Extract affine arrays, rejecting clipping that the runtime cannot carry."""
    if isinstance(preprocessor, ClippedStandardScaler) and not allow_clipping:
        raise ValueError(
            "clipped_standard is supported only for host-side CV; runtime "
            "evaluation and export require an affine scaler"
        )
    center = getattr(preprocessor, 'mean_', None)
    if center is None:
        center = getattr(preprocessor, 'center_', None)
    scale = getattr(preprocessor, 'scale_', None)
    if center is None or scale is None:
        raise AttributeError("Preprocessor must expose center/scale arrays for export")

    center = np.asarray(center, dtype=np.float32)
    scale = np.asarray(scale, dtype=np.float32)
    scale[scale < 1e-6] = 1.0
    return center, scale


def slice_sample_context(sample_context, indices):
    """Slice aligned metadata dicts with NumPy indices."""
    if sample_context is None:
        return None
    return {
        key: np.asarray(values)[indices]
        for key, values in sample_context.items()
    }


def select_balanced_shap_indices(y, sample_context, max_samples, seed):
    """Select deterministic SHAP samples balanced by class, chip, and session."""
    y = np.asarray(y)
    max_samples = min(max(int(max_samples), 0), len(y))
    if max_samples == 0:
        return np.asarray([], dtype=np.int64)

    rng = np.random.default_rng(seed)
    chip_values = np.asarray(
        sample_context.get('chip', np.full(len(y), 'unknown-chip'))
        if sample_context is not None else np.full(len(y), 'unknown-chip')
    )
    session_values = np.asarray(
        sample_context.get('session_group', np.full(len(y), 'unknown-session'))
        if sample_context is not None else np.full(len(y), 'unknown-session')
    )

    buckets_by_label = {}
    for idx, label in enumerate(y):
        stratum = (str(chip_values[idx]), str(session_values[idx]))
        buckets_by_label.setdefault(int(label), {}).setdefault(stratum, []).append(idx)

    label_states = {}
    for label, buckets in buckets_by_label.items():
        keys = sorted(buckets)
        rng.shuffle(keys)
        shuffled_buckets = {}
        for key in keys:
            values = np.asarray(buckets[key], dtype=np.int64)
            rng.shuffle(values)
            shuffled_buckets[key] = values.tolist()
        label_states[label] = {
            'keys': keys,
            'buckets': shuffled_buckets,
            'cursor': 0,
        }

    labels = sorted(label_states)
    rng.shuffle(labels)
    selected = []
    while len(selected) < max_samples:
        progressed = False
        for label in labels:
            state = label_states[label]
            keys = state['keys']
            for _ in range(len(keys)):
                key = keys[state['cursor'] % len(keys)]
                state['cursor'] += 1
                bucket = state['buckets'][key]
                if bucket:
                    selected.append(bucket.pop())
                    progressed = True
                    break
            if len(selected) >= max_samples:
                break
        if not progressed:
            break

    return np.asarray(selected, dtype=np.int64)


def distribute_samples(total_samples, n_folds):
    """Distribute a requested sample count as evenly as possible across folds."""
    total_samples = max(int(total_samples), 0)
    n_folds = max(int(n_folds), 1)
    base, remainder = divmod(total_samples, n_folds)
    return [base + (fold < remainder) for fold in range(n_folds)]


def build_block_mask(sample_context, stride=1, group_key=DEFAULT_BLOCK_GROUP_KEY):
    """Subsample validation windows to reduce overlap optimism during scoring."""
    if sample_context is None:
        return None

    first_key = next(iter(sample_context), None)
    n_samples = len(sample_context[first_key]) if first_key is not None else 0
    if stride <= 1 or n_samples == 0:
        return np.ones(n_samples, dtype=bool)

    mask = np.zeros(n_samples, dtype=bool)
    group_values = sample_context.get(group_key)
    if group_values is None:
        mask[::stride] = True
        return mask

    counters = {}
    for idx, raw_group in enumerate(group_values):
        group = str(raw_group)
        count = counters.get(group, 0)
        if count % stride == 0:
            mask[idx] = True
        counters[group] = count + 1
    return mask
