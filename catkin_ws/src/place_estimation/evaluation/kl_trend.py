#!/usr/bin/env python3
"""Trend summaries for sequential KL-divergence evaluation."""
from typing import Dict, Iterable, List

import numpy as np


def _validated_series(values: Iterable[float]) -> np.ndarray:
    series = np.asarray(list(values), dtype=float)
    if series.ndim != 1 or series.size == 0:
        raise ValueError("trend series must contain at least one value")
    if not np.all(np.isfinite(series)):
        raise ValueError("trend series contains NaN/Inf")
    if np.any(series < 0.0):
        raise ValueError("trend series must be non-negative")
    return series


def _linear_slope(series: np.ndarray) -> float:
    if series.size < 2:
        return 0.0
    x = np.arange(series.size, dtype=float)
    centered_x = x - np.mean(x)
    denominator = float(centered_x @ centered_x)
    if denominator == 0.0:
        return 0.0
    return float(centered_x @ (series - np.mean(series)) / denominator)


def summarize_decreasing_metric(
    values: Iterable[float],
    window_size: int,
    min_samples: int,
    relative_change_threshold: float,
    normalized_slope_threshold: float,
) -> Dict[str, object]:
    """Summarize a metric for which a decrease represents improvement."""
    series = _validated_series(values)
    if window_size <= 0:
        raise ValueError("window_size must be positive")
    if min_samples < 2:
        raise ValueError("min_samples must be at least 2")
    if not 0.0 <= relative_change_threshold < 1.0:
        raise ValueError("relative_change_threshold must be in [0, 1)")
    if normalized_slope_threshold < 0.0:
        raise ValueError("normalized_slope_threshold must be non-negative")

    # Keep the initial and recent comparison windows disjoint.
    effective_window = min(window_size, max(1, series.size // 2))
    initial = series[:effective_window]
    recent = series[-effective_window:]
    initial_mean = float(np.mean(initial))
    recent_mean = float(np.mean(recent))
    scale = max(abs(initial_mean), np.finfo(float).eps)
    relative_improvement = (initial_mean - recent_mean) / scale
    slope = _linear_slope(series)
    recent_slope = _linear_slope(recent)
    recent_scale = max(abs(recent_mean), np.finfo(float).eps)
    normalized_recent_slope = recent_slope / recent_scale

    if series.size < min_samples:
        overall_change = "insufficient_data"
        recent_direction = "insufficient_data"
    else:
        if relative_improvement >= relative_change_threshold:
            overall_change = "improved"
        elif relative_improvement <= -relative_change_threshold:
            overall_change = "worsened"
        else:
            overall_change = "unchanged"

        if normalized_recent_slope <= -normalized_slope_threshold:
            recent_direction = "decreasing"
        elif normalized_recent_slope >= normalized_slope_threshold:
            recent_direction = "increasing"
        else:
            recent_direction = "flat"

    return {
        "count": int(series.size),
        "first": float(series[0]),
        "latest": float(series[-1]),
        "minimum": float(np.min(series)),
        "maximum": float(np.max(series)),
        "initial_window_mean": initial_mean,
        "recent_window_mean": recent_mean,
        "relative_improvement": float(relative_improvement),
        "slope_per_trial": slope,
        "recent_slope_per_trial": recent_slope,
        "normalized_recent_slope": float(normalized_recent_slope),
        "overall_change": overall_change,
        "recent_direction": recent_direction,
    }


def evaluate_adaptation_history(
    records: List[Dict[str, float]],
    window_size: int = 3,
    min_samples: int = 6,
    relative_change_threshold: float = 0.10,
    normalized_slope_threshold: float = 0.02,
) -> Dict[str, object]:
    """Evaluate whether prior/posterior disagreement decreases over trials."""
    if not records:
        raise ValueError("records must not be empty")

    arguments = (
        window_size,
        min_samples,
        relative_change_threshold,
        normalized_slope_threshold,
    )
    summaries = {
        "kl_trend": summarize_decreasing_metric(
            (record["kl"] for record in records), *arguments
        ),
        "mean_shift_trend": summarize_decreasing_metric(
            (record["mean_shift"] for record in records), *arguments
        ),
        "mean_component_trend": summarize_decreasing_metric(
            (record["mean_component"] for record in records), *arguments
        ),
        "covariance_component_trend": summarize_decreasing_metric(
            (record["covariance_component"] for record in records), *arguments
        ),
    }

    if len(records) < min_samples:
        state = "insufficient_data"
    else:
        kl_change = summaries["kl_trend"]["overall_change"]
        shift_change = summaries["mean_shift_trend"]["overall_change"]
        directions = {
            summaries["kl_trend"]["recent_direction"],
            summaries["mean_shift_trend"]["recent_direction"],
        }
        if kl_change == "improved" and shift_change == "improved":
            state = (
                "adapted_stable"
                if directions == {"flat"}
                else "adapting"
            )
        elif kl_change == "worsened" and shift_change == "worsened":
            state = "worsening"
        else:
            state = "mixed"

    latest = records[-1]
    return {
        "schema_version": 1,
        "sample_count": len(records),
        "latest_operation_id": int(latest["operation_id"]),
        "adaptation_state": state,
        "latest": {
            "kl_posterior_to_prior": float(latest["kl"]),
            "mean_shift_m": float(latest["mean_shift"]),
            "mean_component": float(latest["mean_component"]),
            "covariance_component": float(latest["covariance_component"]),
        },
        **summaries,
    }
