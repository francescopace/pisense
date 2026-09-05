# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Model runtime helpers and exported weight artifacts."""

from __future__ import annotations

from tools.lib.bootstrap import setup_paths

setup_paths()

import os
import importlib.util
import numpy as np
import shutil
import tempfile
from pathlib import Path
from tools.lib.atomic_io import atomic_savez, atomic_write_text
from datetime import datetime

try:
    import torch
    import torch.nn as nn
except ImportError:
    torch = None
    nn = None

from .augmentation import (
    derive_seed,
)

from .dataset import (
    CPP_DIR,
    GENERATED_DATA_DIR,
    REFERENCE_SRC_DIR,
)

from .feature_cache import (
    TRAINING_FEATURES,
)

from .preprocessing import (
    DEFAULT_SCALER_MODE,
    get_preprocessor_arrays,
)

TorchModuleBase = nn.Module if nn is not None else object


def ensure_torch_available():
    """Return the torch module or raise ImportError with a stable message."""
    if torch is None or nn is None:
        raise ImportError("No module named 'torch'")
    return torch


def set_active_torch_device(device):
    """Set the process-wide PyTorch training device preference."""
    global ACTIVE_TORCH_DEVICE
    ACTIVE_TORCH_DEVICE = str(device or DEFAULT_TORCH_DEVICE).strip().lower()


def resolve_torch_device(device=None, torch_module=None):
    """Resolve a PyTorch device name from cpu/cuda/mps."""
    torch_mod = torch_module if torch_module is not None else ensure_torch_available()
    requested = str(device or ACTIVE_TORCH_DEVICE or DEFAULT_TORCH_DEVICE).strip().lower()
    if requested == 'cuda':
        if not torch_mod.cuda.is_available():
            raise RuntimeError("CUDA device requested, but torch.cuda.is_available() is false")
        return torch_mod.device('cuda')
    if requested == 'mps':
        if not (hasattr(torch_mod.backends, 'mps') and torch_mod.backends.mps.is_available()):
            raise RuntimeError("MPS device requested, but torch.backends.mps.is_available() is false")
        return torch_mod.device('mps')
    if requested == 'cpu':
        return torch_mod.device('cpu')
    raise ValueError(f"Unsupported torch device: {device!r}")


def describe_torch_device(device=None):
    """Return a compact human-readable training device description."""
    torch_mod = ensure_torch_available()
    resolved = resolve_torch_device(device, torch_module=torch_mod)
    if resolved.type == 'cuda':
        name = torch_mod.cuda.get_device_name(resolved)
        return f"cuda ({name})"
    if resolved.type == 'mps':
        return "mps (Apple Metal)"
    return "cpu"


def model_torch_device(model):
    """Return the device where a TorchMLP stores its parameters."""
    try:
        return next(model.parameters()).device
    except StopIteration:
        return resolve_torch_device('cpu')


def _init_linear(layer, seed=None):
    """Initialize a Linear layer with Glorot uniform weights and zero bias."""
    if torch is None:
        raise ImportError("No module named 'torch'")
    if seed is None:
        nn.init.xavier_uniform_(layer.weight)
        nn.init.zeros_(layer.bias)
        return

    rng_state = torch.get_rng_state()
    try:
        torch.manual_seed(int(seed))
        nn.init.xavier_uniform_(layer.weight)
        nn.init.zeros_(layer.bias)
    finally:
        torch.set_rng_state(rng_state)


class TorchMLP(TorchModuleBase):
    """Dense binary classifier with export helpers for runtime artifacts."""

    def __init__(self, num_features, hidden_layers=None, use_dropout=True,
                 dropout_rate=0.2, seed=None):
        super().__init__()
        if hidden_layers is None:
            hidden_layers = list(DEFAULT_HIDDEN_LAYERS)
        self.num_features = int(num_features)
        self.hidden_layers = [int(units) for units in hidden_layers]
        self.dropout_rate = float(dropout_rate)
        self.use_dropout = bool(use_dropout and dropout_rate > 0.0)

        self.linears = nn.ModuleList()
        self.dropouts = nn.ModuleList()

        in_features = self.num_features
        for layer_idx, units in enumerate(self.hidden_layers):
            linear = nn.Linear(in_features, units)
            _init_linear(linear, derive_seed(seed, layer_idx, 0))
            self.linears.append(linear)
            if self.use_dropout:
                self.dropouts.append(nn.Dropout(self.dropout_rate))
            in_features = units

        self.output = nn.Linear(in_features, 1)
        _init_linear(self.output, derive_seed(seed, len(self.hidden_layers), 0))

    def forward_logits(self, x):
        if not isinstance(x, torch.Tensor):
            x = torch.as_tensor(x, dtype=torch.float32)
        activations = x
        for layer_idx, linear in enumerate(self.linears):
            activations = torch.relu(linear(activations))
            if self.use_dropout:
                activations = self.dropouts[layer_idx](activations)
        return self.output(activations)

    def forward(self, x):
        return torch.sigmoid(self.forward_logits(x))

    def predict(self, X, verbose=0):
        probs = predict_probabilities(self, X)
        return probs.reshape(-1, 1)

    def get_weights(self):
        return extract_model_weights(self)


def extract_model_weights(model):
    """Return dense-layer weights in the export layout expected by the runtimes."""
    if isinstance(model, TorchMLP):
        weights = []
        for linear in list(model.linears) + [model.output]:
            kernel = linear.weight.detach().cpu().numpy().T.copy()
            bias = linear.bias.detach().cpu().numpy().copy()
            weights.extend((kernel, bias))
        return weights
    if hasattr(model, 'get_weights'):
        return model.get_weights()
    raise TypeError(f"Unsupported model type for weight export: {type(model)!r}")


def predict_logits(model, X):
    """Return flat logits for a dense binary classifier."""
    ensure_torch_available()
    X = np.asarray(X, dtype=np.float32)
    if X.size == 0:
        return np.asarray([], dtype=np.float32)
    if not isinstance(model, TorchMLP):
        raise TypeError(f"Unsupported model type for logits: {type(model)!r}")
    model.eval()
    device = model_torch_device(model)
    with torch.no_grad():
        logits = model.forward_logits(torch.from_numpy(X).to(device))
    return logits.detach().cpu().numpy().reshape(-1)


DEFAULT_HIDDEN_LAYERS = [24, 12]


DEFAULT_TORCH_DEVICE = 'cpu'


ACTIVE_TORCH_DEVICE = DEFAULT_TORCH_DEVICE


def load_exported_ml_weights():
    """Load the currently exported Python reference ML weights module."""
    weights_path = REFERENCE_SRC_DIR / 'ml_weights.py'
    if not weights_path.exists():
        raise FileNotFoundError(f"Exported ML weights not found: {weights_path}")
    spec = importlib.util.spec_from_file_location("exported_ml_weights", weights_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def exported_weight_matrices(weights_module):
    """Return exported matrices in the host [input][output] convention."""
    if hasattr(weights_module, 'WEIGHTS_T'):
        return [
            np.asarray(layer, dtype=np.float32).T
            for layer in weights_module.WEIGHTS_T
        ]
    return [
        np.asarray(layer, dtype=np.float32)
        for layer in weights_module.WEIGHTS
    ]


def predict_exported_probabilities_from_weights(weights_module, X_raw):
    """Vectorized inference matching the Python reference High Accuracy detector."""
    center = np.asarray(weights_module.FEATURE_MEAN, dtype=np.float32)
    scale = np.asarray(weights_module.FEATURE_SCALE, dtype=np.float32)
    scale[scale < 1e-6] = 1.0
    matrices = exported_weight_matrices(weights_module)
    layers = [
        (
            layer_weights,
            np.asarray(layer_biases, dtype=np.float32),
            layer_index == len(matrices) - 1,
        )
        for layer_index, (layer_weights, layer_biases) in enumerate(
            zip(matrices, weights_module.BIASES, strict=True)
        )
    ]
    return predict_probabilities_from_arrays(X_raw, center, scale, layers)


def predict_probabilities_from_arrays(features, center, scale, layers):
    """Run the shared exported/runtime-array inference implementation."""
    features = np.asarray(features, dtype=np.float32)
    if features.size == 0:
        return np.zeros(0, dtype=np.float32)
    center = np.asarray(center, dtype=np.float32)
    scale = np.asarray(scale, dtype=np.float32).copy()
    scale[scale < 1e-6] = 1.0
    activations = (features - center) / scale
    for weights, biases, is_output in layers:
        activations = (
            activations @ np.asarray(weights, dtype=np.float32)
            + np.asarray(biases, dtype=np.float32)
        )
        if not is_output:
            activations = np.maximum(activations, 0.0)

    logits = activations.reshape(-1).astype(np.float32, copy=False)
    probabilities = np.empty(logits.shape, dtype=np.float32)
    probabilities[logits < -20.0] = 0.0
    probabilities[logits > 20.0] = 1.0
    mask = (logits >= -20.0) & (logits <= 20.0)
    probabilities[mask] = 1.0 / (1.0 + np.exp(-logits[mask]))
    return probabilities


def predict_probabilities(model, X):
    """
    Return probabilities through the deployment-equivalent logit mapping.
    """
    return predict_runtime_probabilities(model, X)


def predict_runtime_probabilities(model, X):
    """
    Return probabilities using the same post-logit mapping as Python/C++ runtime inference.
    """
    X = np.asarray(X, dtype=np.float32)
    logits = predict_logits(model, X)
    probabilities = np.empty_like(logits, dtype=np.float32)
    probabilities[logits < -20.0] = 0.0
    probabilities[logits > 20.0] = 1.0
    mask = (logits >= -20.0) & (logits <= 20.0)
    probabilities[mask] = 1.0 / (1.0 + np.exp(-logits[mask]))
    return probabilities


def get_model_architecture(model):
    """Return the layer sizes of a dense MLP as [input, ..., output]."""
    weights = extract_model_weights(model)
    if not weights:
        return []

    layer_sizes = [int(weights[0].shape[0])]
    for idx in range(0, len(weights), 2):
        layer_sizes.append(int(weights[idx].shape[1]))
    return layer_sizes


def render_micropython_weights(weights, center, scale, architecture, seed=None,
                               feature_names=None,
                               scaler_mode=DEFAULT_SCALER_MODE,
                               trained_at=None):
    """Render inference-ready MicroPython weights without a runtime transpose."""
    if feature_names is None:
        feature_names = list(TRAINING_FEATURES)
    seed_info = f"Seed: {seed}"
    timestamp = trained_at or datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    architecture_text = ' -> '.join(map(str, architecture))
    architecture_csv = ', '.join(str(x) for x in architecture)
    hidden_csv = ', '.join(str(x) for x in architecture[1:-1])
    feature_csv = ', '.join(repr(name) for name in feature_names)
    center_csv = ', '.join(f'{x:.9g}' for x in center)
    scale_csv = ', '.join(f'{x:.9g}' for x in scale)

    # Build code - weights only
    code = f'''# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Reference ML Model Weights

Auto-generated neural network weights for motion detection.
Architecture: {architecture_text}
Normalization: {scaler_mode}
Trained: {timestamp}
{seed_info}

This file is auto-generated by train_ml_model.py.
DO NOT EDIT - your changes will be overwritten!

Author: Francesco Pace <francesco.pace@gmail.com>
"""

# Model metadata
MODEL_LAYER_SIZES = [{architecture_csv}]
MODEL_HIDDEN_LAYERS = [{hidden_csv}]
ML_NUM_FEATURES = {architecture[0]}
ML_NUM_LAYERS = {len(architecture) - 1}
NORMALIZATION_MODE = "{scaler_mode}"
FEATURE_NAMES = [{feature_csv}]

# Feature normalization
FEATURE_MEAN = [{center_csv}]
FEATURE_SCALE = [{scale_csv}]

'''

    # Store each matrix as [output][input], matching the inference loop. This
    # avoids retaining a second forest of nested lists while transposing at
    # import time on memory-constrained MicroPython targets.
    weight_names = []
    bias_names = []
    for i in range(0, len(weights), 2):
        W = weights[i]
        b = weights[i + 1]
        layer_num = i // 2 + 1
        in_size, out_size = W.shape

        activation = 'Sigmoid' if i == len(weights) - 2 else 'ReLU'
        code += f'# Layer {layer_num}: {in_size} -> {out_size} ({activation})\n'
        code += f'WT{layer_num} = [\n'
        for row in W.T:
            code += '    [' + ', '.join(f'{x:.9g}' for x in row) + '],\n'
        code += ']\n'
        code += f'B{layer_num} = [' + ', '.join(f'{x:.9g}' for x in b) + ']\n\n'
        weight_names.append(f'WT{layer_num}')
        bias_names.append(f'B{layer_num}')

    code += f'WEIGHTS_T = [{", ".join(weight_names)}]\n'
    code += f'BIASES = [{", ".join(bias_names)}]\n'
    return code


def export_micropython(model, scaler, output_path, seed=None,
                       feature_names=None, scaler_mode=DEFAULT_SCALER_MODE,
                       trained_at=None):
    """
    Export model weights to MicroPython code.

    Generates ml_weights.py with inference-ready transposed network weights.
    The inference functions are in high_accuracy_detector.py (not auto-generated).
    """
    weights = extract_model_weights(model)
    center, scale = get_preprocessor_arrays(scaler)
    architecture = get_model_architecture(model)
    code = render_micropython_weights(
        weights,
        center,
        scale,
        architecture,
        seed=seed,
        feature_names=feature_names,
        scaler_mode=scaler_mode,
        trained_at=trained_at,
    )
    atomic_write_text(output_path, code)
    return len(code)


CPP_FEATURE_IDS = {
    'turb_autocorr': 6,
    'turb_zcr': 14,
    'l1_delta_lag_ratio': 25,
    'turb_iqr_over_mean_aggr': 45,
    'chan_shape_coherent_innovation_energy': 46,
    'chan_shape_excess_path': 47,
    'chan_shape_spread_subband': 48,
    'chan_shape_subband_kendall_lag_excess': 49,
}


def resolve_cpp_feature_ids(feature_names):
    """Map feature names to their published C++ extractor ids."""
    ids = []
    for name in feature_names:
        if name not in CPP_FEATURE_IDS:
            raise ValueError(
                f"feature {name!r} has no C++ extractor id; add it to "
                f"CPP_FEATURE_IDS and the MLFeatureId enum in csi_features.h "
                f"before exporting a model that uses it"
            )
        ids.append(CPP_FEATURE_IDS[name])
    return ids


def export_cpp_weights(model, scaler, output_path, seed=None,
                       feature_names=None, scaler_mode=DEFAULT_SCALER_MODE,
                       trained_at=None):
    """
    Export model weights to the shared C++ header.

    Generates ml_weights.h with constexpr weights.

    Args:
        model: Trained PyTorch model
        scaler: Fitted preprocessing object exposing center/scale arrays
        output_path: Output file path
        seed: Random seed used for training (or None if not set)
        feature_names: Ordered feature names expected by the model
        scaler_mode: Normalization mode used during training
        trained_at: Optional training timestamp preserved during metadata-only regeneration

    Returns:
        Size of generated code
    """

    def cpp_float(value):
        """Render a numeric literal with a valid C++ float suffix."""
        text = f'{float(value):.9g}'
        if 'e' not in text and 'E' not in text and '.' not in text:
            text += '.0'
        return text + 'f'

    weights = extract_model_weights(model)
    architecture = get_model_architecture(model)
    arch = ' -> '.join(map(str, architecture))
    center, scale = get_preprocessor_arrays(scaler)
    if feature_names is None:
        feature_names = list(TRAINING_FEATURES)

    feature_ids = resolve_cpp_feature_ids(feature_names)

    seed_info = f"Seed: {seed}"
    timestamp = trained_at or datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    architecture_csv = ', '.join(str(x) for x in architecture)
    center_csv = ', '.join(cpp_float(x) for x in center)
    scale_csv = ', '.join(cpp_float(x) for x in scale)
    feature_ids_csv = ', '.join(str(i) for i in feature_ids)
    feature_names_comment = ', '.join(feature_names)
    comment_gap = ' * '

    code = f'''/*
 * ESPectre - ML Model Weights
{comment_gap}
 * Auto-generated neural network weights for motion detection.
 * Architecture: {arch}
 * Normalization: {scaler_mode}
 * Trained: {timestamp}
 * {seed_info}
{comment_gap}
 * This file is auto-generated by train_ml_model.py.
 * DO NOT EDIT - your changes will be overwritten!
{comment_gap}
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

#pragma once

namespace espectre {{

// Model metadata
constexpr uint8_t ML_MODEL_NUM_LAYERS = {len(architecture) - 1};
constexpr uint8_t ML_MODEL_INPUT_SIZE = {architecture[0]};
constexpr uint8_t ML_MAX_LAYER_WIDTH = {max(architecture[1:])};
constexpr uint8_t ML_MODEL_LAYER_SIZES[{len(architecture)}] = {{{architecture_csv}}};
constexpr char ML_NORMALIZATION_MODE[] = "{scaler_mode}";

// Feature normalization
constexpr float ML_FEATURE_MEAN[{len(center)}] = {{{center_csv}}};
constexpr float ML_FEATURE_SCALE[{len(scale)}] = {{{scale_csv}}};

// Feature identity (MLFeatureId in csi_features.h), one per model input slot.
// Order: {feature_names_comment}
constexpr uint8_t ML_FEATURE_IDS[{len(feature_ids)}] = {{{feature_ids_csv}}};

'''

    # Add weights for each layer
    weight_names = []
    bias_names = []
    input_sizes = []
    output_sizes = []
    for i in range(0, len(weights), 2):
        W = weights[i]
        b = weights[i + 1]
        layer_num = i // 2 + 1
        in_size, out_size = W.shape

        activation = 'Sigmoid' if i == len(weights) - 2 else 'ReLU'
        code += f'// Layer {layer_num}: {in_size} -> {out_size} ({activation})\n'
        flat_weights = W.reshape(-1)
        code += f'constexpr float ML_W{layer_num}[{len(flat_weights)}] = {{' \
                + ', '.join(cpp_float(x) for x in flat_weights) + '};\n'
        code += f'constexpr float ML_B{layer_num}[{out_size}] = {{{", ".join(cpp_float(x) for x in b)}}};\n\n'
        weight_names.append(f'ML_W{layer_num}')
        bias_names.append(f'ML_B{layer_num}')
        input_sizes.append(str(in_size))
        output_sizes.append(str(out_size))

    code += (
        f'constexpr uint8_t ML_MODEL_LAYER_INPUT_SIZES[ML_MODEL_NUM_LAYERS] = '
        f'{{{", ".join(input_sizes)}}};\n'
    )
    code += (
        f'constexpr uint8_t ML_MODEL_LAYER_OUTPUT_SIZES[ML_MODEL_NUM_LAYERS] = '
        f'{{{", ".join(output_sizes)}}};\n'
    )
    code += (
        f'constexpr const float* ML_MODEL_WEIGHTS[ML_MODEL_NUM_LAYERS] = '
        f'{{{", ".join(weight_names)}}};\n'
    )
    code += (
        f'constexpr const float* ML_MODEL_BIASES[ML_MODEL_NUM_LAYERS] = '
        f'{{{", ".join(bias_names)}}};\n\n'
    )

    code += '''}  // namespace espectre
'''

    atomic_write_text(output_path, code)

    return len(code)


def export_test_data(model, scaler, X_test_raw, y_test, output_path):
    """
    Export test data for validation across Python and C++.

    Generates ml_test_data.npz with RAW features (not normalized) and expected outputs.
    This allows testing the full pipeline including normalization.

    The artifact is committed, so it carries only what the host and C++
    regression suites read: raw features, labels, and expected outputs. It
    holds no object arrays and stays loadable with ``allow_pickle=False``.

    Args:
        model: Trained PyTorch model
        scaler: Fitted preprocessing object used for normalization
        X_test_raw: Test features (NOT normalized, raw values)
        y_test: Test labels
        output_path: Output file path

    Returns:
        Number of test samples
    """
    # Normalize for prediction
    X_test_scaled = scaler.transform(X_test_raw)
    predictions = predict_runtime_probabilities(model, X_test_scaled)

    # Save RAW features (not normalized) so tests can verify full pipeline
    payload = {
        'features': X_test_raw.astype(np.float32),
        'labels': y_test.astype(np.int32),
        'expected_outputs': predictions.astype(np.float32),
    }
    atomic_savez(output_path, payload)

    return len(X_test_raw)


def _model_artifact_paths():
    """Return paths of generated model artifacts."""
    return [
        REFERENCE_SRC_DIR / 'ml_weights.py',
        CPP_DIR / 'ml_weights.h',
        GENERATED_DATA_DIR / 'ml_test_data.npz',
    ]


def _backup_artifacts():
    """Backup model artifacts to a temporary directory."""
    backup_dir = Path(tempfile.mkdtemp(prefix='ml_seed_search_backup_'))
    saved_files = []
    for path in _model_artifact_paths():
        if path.exists():
            rel_name = path.name
            shutil.copy2(path, backup_dir / rel_name)
            saved_files.append((path, backup_dir / rel_name, True))
        else:
            saved_files.append((path, None, False))
    return backup_dir, saved_files


def _restore_artifacts(saved_files):
    """Restore model artifacts from backup copies."""
    for original, backup, existed in saved_files:
        if existed and backup is not None and backup.exists():
            original.parent.mkdir(parents=True, exist_ok=True)
            temporary = original.parent / f".{original.name}.restore.{os.getpid()}"
            shutil.copy2(backup, temporary)
            os.replace(temporary, original)
        elif not existed:
            original.unlink(missing_ok=True)
