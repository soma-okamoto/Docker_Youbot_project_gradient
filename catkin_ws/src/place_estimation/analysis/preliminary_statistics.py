#!/usr/bin/env python3
"""Pure statistical helpers for preliminary-experiment analysis."""
from typing import Dict, Iterable, List, Optional, Sequence

import numpy as np


def descriptive_statistics(values: Iterable[float]) -> Dict[str, object]:
    """Return finite-value descriptive statistics as JSON-safe values."""
    array = np.asarray(list(values), dtype=float).reshape(-1)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return {"count": 0}
    sample_std = float(np.std(array, ddof=1)) if array.size >= 2 else 0.0
    return {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "sample_std": sample_std,
        "minimum": float(np.min(array)),
        "p05": float(np.percentile(array, 5.0)),
        "p50": float(np.percentile(array, 50.0)),
        "p90": float(np.percentile(array, 90.0)),
        "p95": float(np.percentile(array, 95.0)),
        "p99": float(np.percentile(array, 99.0)),
        "maximum": float(np.max(array)),
    }


def vector_statistics(vectors: Iterable[Sequence[float]]) -> Dict[str, object]:
    """Return the mean and sample covariance of finite 3-D vectors."""
    rows = []
    for vector in vectors:
        value = np.asarray(vector, dtype=float).reshape(-1)
        if value.shape == (3,) and np.all(np.isfinite(value)):
            rows.append(value)
    if not rows:
        return {"count": 0}
    matrix = np.vstack(rows)
    covariance = (
        np.cov(matrix, rowvar=False, ddof=1)
        if matrix.shape[0] >= 2
        else np.zeros((3, 3), dtype=float)
    )
    return {
        "count": int(matrix.shape[0]),
        "mean": np.mean(matrix, axis=0).tolist(),
        "sample_covariance": np.asarray(covariance).reshape(3, 3).tolist(),
        "sample_std": np.sqrt(np.maximum(np.diag(covariance), 0.0)).tolist(),
    }


def principal_standard_deviation(covariance: Sequence[float]) -> float:
    matrix = np.asarray(covariance, dtype=float).reshape(3, 3)
    matrix = 0.5 * (matrix + matrix.T)
    if not np.all(np.isfinite(matrix)):
        return float("nan")
    eigenvalues = np.linalg.eigvalsh(matrix)
    if eigenvalues[0] < -1.0e-12:
        return float("nan")
    return float(np.sqrt(max(eigenvalues[-1], 0.0)))


def learning_stability_candidate(
    bias_history: Iterable[Sequence[float]],
    tolerance_m: float = 0.002,
    stable_updates: int = 3,
) -> Optional[int]:
    """Find the first sample after which bias updates remain small.

    This is a diagnostic candidate, not an automatic parameter decision.
    """
    values = np.asarray(list(bias_history), dtype=float)
    if (
        values.ndim != 2
        or values.shape[1] != 3
        or values.shape[0] < stable_updates + 1
        or not np.all(np.isfinite(values))
    ):
        return None
    changes = np.linalg.norm(np.diff(values, axis=0), axis=1)
    for start in range(0, changes.size - stable_updates + 1):
        if np.all(changes[start:] <= tolerance_m):
            return int(start + 2)
    return None


def _values(trials: Iterable[Dict[str, object]], key: str) -> List[float]:
    output = []
    for trial in trials:
        value = trial.get(key)
        if value is None:
            continue
        try:
            value = float(value)
        except (TypeError, ValueError):
            continue
        if np.isfinite(value):
            output.append(value)
    return output


def _vectors(
    trials: Iterable[Dict[str, object]],
    source_prefix: str,
    used_prefix: str = "used",
) -> List[List[float]]:
    output = []
    for trial in trials:
        source = [trial.get(f"{source_prefix}_{axis}") for axis in "xyz"]
        used = [trial.get(f"{used_prefix}_{axis}") for axis in "xyz"]
        try:
            error = np.asarray(source, dtype=float) - np.asarray(used, dtype=float)
        except (TypeError, ValueError):
            continue
        if error.shape == (3,) and np.all(np.isfinite(error)):
            output.append(error.tolist())
    return output


def build_parameter_report(
    trials: List[Dict[str, object]],
    learning_rows: List[Dict[str, object]],
    current_min_variance: float = 1.0e-8,
    bias_stability_tolerance_m: float = 0.002,
    stable_updates: int = 3,
) -> Dict[str, object]:
    """Build evidence summaries and non-binding Config candidates."""
    trials = sorted(trials, key=lambda row: int(row["operation_id"]))
    learning_rows = sorted(
        learning_rows, key=lambda row: int(row.get("sample_count", 0))
    )

    distributions = {
        "selected_yolo_d2": descriptive_statistics(
            _values(trials, "selected_yolo_d2")
        ),
        "selected_meta_d2": descriptive_statistics(
            _values(trials, "selected_meta_d2")
        ),
        "selected_yolo_meta_d2": descriptive_statistics(
            _values(trials, "selected_yolo_meta_d2")
        ),
        "kl_prior_to_posterior": descriptive_statistics(
            _values(trials, "kl_prior_to_posterior")
        ),
        "mean_shift_m": descriptive_statistics(
            _values(trials, "mean_shift_m")
        ),
        "yolo_candidate_principal_std_m": descriptive_statistics(
            value
            for trial in trials
            for value in trial.get("_yolo_principal_stds", [])
        ),
        "used_observation_principal_std_m": descriptive_statistics(
            _values(trials, "used_principal_std_m")
        ),
    }

    current_errors = _vectors(trials, "current")
    tf_errors = _vectors(trials, "tf")
    operation_error = {
        "current_minus_used_observation": vector_statistics(current_errors),
        "tf_minus_used_observation": vector_statistics(tf_errors),
    }

    bias_current_history = [
        row["bias_current"]
        for row in learning_rows
        if row.get("bias_current") is not None
    ]
    bias_tf_history = [
        row["bias_tf"]
        for row in learning_rows
        if row.get("bias_tf") is not None
    ]
    current_candidate = learning_stability_candidate(
        bias_current_history, bias_stability_tolerance_m, stable_updates
    )
    tf_candidate = learning_stability_candidate(
        bias_tf_history, bias_stability_tolerance_m, stable_updates
    )
    available_candidates = [
        value for value in (current_candidate, tf_candidate) if value is not None
    ]
    learning_min_candidate = (
        max(available_candidates) if len(available_candidates) == 2 else None
    )

    learned_eigenvalues = []
    for row in learning_rows:
        for key in ("covariance_current", "covariance_tf"):
            value = row.get(key)
            if value is None:
                continue
            matrix = np.asarray(value, dtype=float).reshape(3, 3)
            if np.all(np.isfinite(matrix)):
                learned_eigenvalues.extend(
                    np.linalg.eigvalsh(0.5 * (matrix + matrix.T)).tolist()
                )
    floor_hits = sum(
        value <= current_min_variance * (1.0 + 1.0e-6)
        for value in learned_eigenvalues
    )

    completeness_keys = {
        "P_current": "current_x",
        "P_tf": "tf_x",
        "prior_distribution": "pred_x",
        "posterior_distribution": "place_x",
        "KL_evaluation": "kl_prior_to_posterior",
    }
    data_completeness = {
        name: sum(trial.get(key) is not None for trial in trials)
        for name, key in completeness_keys.items()
    }

    def percentile_candidate(name: str):
        statistics = distributions[name]
        return statistics.get("p95") if statistics.get("count", 0) else None

    candidates = {
        "gate_yolo_p95_diagnostic": percentile_candidate(
            "selected_yolo_d2"
        ),
        "gate_meta_p95_diagnostic": percentile_candidate(
            "selected_meta_d2"
        ),
        "gate_yolo_meta_p95_diagnostic": percentile_candidate(
            "selected_yolo_meta_d2"
        ),
        "max_std_yolo_p95_m_diagnostic": percentile_candidate(
            "yolo_candidate_principal_std_m"
        ),
        "learning_min_samples_stability_candidate": learning_min_candidate,
        "bias_stability_tolerance_m": bias_stability_tolerance_m,
        "stable_updates_required": stable_updates,
    }

    warnings = [
        "These values describe repeatability and internal consistency; they do not measure absolute accuracy without ground truth.",
        "P95 values are diagnostic candidates only. Freeze parameters using separate calibration data before the main experiment.",
        "Distance distributions are affected by the current covariance and candidate-selection settings.",
    ]
    if floor_hits:
        warnings.append(
            f"Learned covariance eigenvalues hit the current floor "
            f"{floor_hits} times; min_variance requires review."
        )
    if learning_min_candidate is None:
        warnings.append(
            "The recorded bias history did not establish a learning_min_samples candidate."
        )
    incomplete = [
        name
        for name, count in data_completeness.items()
        if count < len(trials)
    ]
    if incomplete:
        warnings.append(
            "Some trial records are incomplete for: "
            + ", ".join(incomplete)
            + ". Wait until rosbag reports 'Recording to' before the first trial."
        )

    return {
        "schema_version": 1,
        "trial_count": len(trials),
        "learning_update_count": len(learning_rows),
        "data_completeness": data_completeness,
        "distributions": distributions,
        "operation_error_repeatability": operation_error,
        "learned_covariance_eigenvalues_m2": descriptive_statistics(
            learned_eigenvalues
        ),
        "current_min_variance_m2": current_min_variance,
        "min_variance_floor_hit_count": floor_hits,
        "parameter_candidates": candidates,
        "warnings": warnings,
    }
