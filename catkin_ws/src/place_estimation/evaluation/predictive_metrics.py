#!/usr/bin/env python3
"""ROS-free, paired predictive scores for frozen Place-operation priors.

Scores use the declared approximation Var(y - mu) = Sigma + R. They do not
establish error independence, target identity, or absolute physical accuracy.
"""
from collections import Counter, deque
import copy

import numpy as np


SENSORS = ("meta", "realsense")
CHI_SQUARED_3D_95 = 7.814727903251179


def _position(value):
    position = np.asarray(value, dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("position must contain three finite coordinates")
    return position


def _covariance(value, min_variance):
    covariance = np.asarray(value, dtype=float)
    if covariance.size != 9 or not np.all(np.isfinite(covariance)):
        raise ValueError("covariance must contain nine finite values")
    covariance = covariance.reshape(3, 3)
    covariance = 0.5 * (covariance + covariance.T)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if eigenvalues[0] < -min_variance:
        raise ValueError("covariance is not positive semidefinite")
    regularized = bool(np.any(eigenvalues < min_variance))
    covariance = (eigenvectors * np.maximum(eigenvalues, min_variance)) @ eigenvectors.T
    return covariance, regularized, int(np.count_nonzero(
        eigenvalues <= min_variance * (1.0 + 1.e-6)
    ))


def gaussian_observation_score(mean, covariance, observation,
                               observation_covariance, min_variance=1.e-8):
    """Score one 3-D observation, including its declared measurement noise.

    Natural logarithms, metres and m^2 are used. Negative NLL is valid for a
    continuous density. Invalid covariance is rejected; tiny eigenvalues are
    floored explicitly and reported rather than silently treated as evidence.
    """
    if not np.isfinite(min_variance) or min_variance <= 0.0:
        raise ValueError("min_variance must be finite and positive")
    residual = _position(observation) - _position(mean)
    prior_covariance, prior_regularized, floor_count = _covariance(
        covariance, min_variance
    )
    sensor_covariance, observation_regularized, _ = _covariance(
        observation_covariance, min_variance
    )
    predictive_covariance = prior_covariance + sensor_covariance
    factor = np.linalg.cholesky(predictive_covariance)
    whitened = np.linalg.solve(factor, residual)
    d2 = float(whitened @ whitened)
    logdet = 2.0 * float(np.log(np.diag(factor)).sum())
    eigenvalues = np.linalg.eigvalsh(predictive_covariance)
    result = {
        "nll": 0.5 * (3.0 * np.log(2.0 * np.pi) + logdet + d2),
        "mahalanobis_squared": d2,
        "inside_95_predictive_ellipsoid": d2 <= CHI_SQUARED_3D_95,
        "log_determinant": logdet,
        "position_error_m": float(np.linalg.norm(residual)),
        "squared_position_error_m2": float(residual @ residual),
        "predictive_covariance_trace_m2": float(np.trace(predictive_covariance)),
        "predictive_min_eigenvalue_m2": float(eigenvalues[0]),
        "predictive_condition_number": float(eigenvalues[-1] / eigenvalues[0]),
        "prior_floor_eigenvalue_count": floor_count,
        "covariance_regularized": prior_regularized or observation_regularized,
    }
    if not all(np.isfinite(value) for value in result.values()):
        raise ValueError("predictive score is not finite")
    return {key: float(value) if isinstance(value, np.floating) else value
            for key, value in result.items()}


def evaluate_trial(snapshot, min_variance=1.e-8):
    """Compare both priors on exactly the same sensor representatives.

    A trial score requires exactly one quality-valid candidate, independently
    of prior-distance gates. Multiple candidates remain ambiguous. Scores of
    algorithm-selected candidates are a separate selection-dependent diagnostic.
    Every quality candidate is retained for later independent target annotation.
    """
    if snapshot.get("schema_version") != 1:
        raise ValueError("unsupported predictive snapshot schema")
    operation_id = snapshot.get("operation_id")
    if isinstance(operation_id, bool) or not isinstance(operation_id, int) or operation_id <= 0:
        raise ValueError("snapshot requires an explicit positive operation_id")
    result = copy.deepcopy(snapshot)
    result["evaluation"] = {
        "score_unit": "nats",
        "position_unit": "m",
        "covariance_model": "prior_plus_observation_independent_errors",
        "error_independence_verified": False,
        "target_identity_verified": False,
        "physical_ground_truth": False,
        "trial_score_selection": "unique_quality_candidate_before_prior_gates",
        "delta_nll_definition": "fixed_minus_adaptive",
        "min_variance_m2": min_variance,
        "kl_role": "distribution_update_diagnostic",
    }
    priors = snapshot.get("priors", {})
    predictions_available = all(name in priors for name in ("fixed", "adaptive"))
    evaluations = {}
    for sensor in SENSORS:
        data = snapshot.get("sensors", {}).get(sensor, {})
        candidates = data.get("quality_candidates", [])
        evaluation = {
            "quality_candidate_count": len(candidates),
            "candidate_scores": [],
            "trial_score": None,
            "selected_diagnostic": None,
            "selected_diagnostic_selection_dependent": True,
        }
        if not data.get("enabled", False):
            status = "sensor_disabled"
        elif not data.get("received", False):
            status = "no_observation"
        elif not candidates:
            status = "no_quality_candidate"
        elif not predictions_available:
            status = "missing_prediction_snapshot"
        else:
            for candidate in candidates:
                entry = {"index": candidate["index"]}
                try:
                    for model in ("adaptive", "fixed"):
                        prior = priors[model]
                        entry[model] = gaussian_observation_score(
                            prior["mean"], prior["covariance"],
                            candidate["mean"], candidate["covariance"], min_variance,
                        )
                    entry["delta_nll"] = entry["fixed"]["nll"] - entry["adaptive"]["nll"]
                    entry["delta_squared_error_m2"] = (
                        entry["fixed"]["squared_position_error_m2"]
                        - entry["adaptive"]["squared_position_error_m2"]
                    )
                except (KeyError, TypeError, ValueError, np.linalg.LinAlgError) as error:
                    entry = {"index": candidate["index"], "error": str(error)}
                evaluation["candidate_scores"].append(entry)
                if (candidate["index"] == data.get("algorithm_selected_index")
                        and "error" not in entry):
                    evaluation["selected_diagnostic"] = entry
            if len(candidates) != 1:
                status = "ambiguous_candidates"
            elif "error" in evaluation["candidate_scores"][0]:
                status = "invalid_predictive_distribution"
            else:
                status = "scored_unverified_target"
                evaluation["trial_score"] = evaluation["candidate_scores"][0]
        evaluation["status"] = status
        evaluations[sensor] = evaluation
    result["scores"] = evaluations
    return result


class PredictiveHistory:
    """Bounded-window, per-sensor paired summaries; no adaptation verdict."""

    def __init__(self, window_size=10):
        if window_size <= 0:
            raise ValueError("window_size must be positive")
        self.window_size = window_size
        self.trial_count = 0
        self.latest_operation_id = None
        self.latest_learning_sample_count = None
        self.sensors = {
            sensor: {"statuses": Counter(), "enabled_trials": 0,
                     "received_trials": 0, "scored_trials": 0,
                     "positive_delta_trials": 0, "sum_adaptive_nll": 0.0,
                     "sum_fixed_nll": 0.0, "sum_delta_nll": 0.0,
                     "sum_adaptive_squared_error": 0.0,
                     "sum_fixed_squared_error": 0.0,
                     "sum_adaptive_d2": 0.0, "sum_fixed_d2": 0.0,
                     "adaptive_covered_95": 0, "fixed_covered_95": 0,
                     "adaptive_floor_hit_trials": 0, "fixed_floor_hit_trials": 0,
                     "recent_delta_nll": deque(maxlen=window_size)}
            for sensor in SENSORS
        }

    def add(self, result):
        self.trial_count += 1
        self.latest_operation_id = result["operation_id"]
        self.latest_learning_sample_count = result.get("learning_sample_count_before_trial")
        for sensor in SENSORS:
            state = self.sensors[sensor]
            data = result["sensors"][sensor]
            scores = result["scores"][sensor]
            state["statuses"][scores["status"]] += 1
            state["enabled_trials"] += int(data["enabled"])
            state["received_trials"] += int(data["enabled"] and data["received"])
            score = scores["trial_score"]
            if score is None:
                continue
            state["scored_trials"] += 1
            state["positive_delta_trials"] += int(score["delta_nll"] > 0.0)
            for model in ("adaptive", "fixed"):
                state["sum_" + model + "_nll"] += score[model]["nll"]
                state["sum_" + model + "_squared_error"] += score[model]["squared_position_error_m2"]
                state["sum_" + model + "_d2"] += score[model]["mahalanobis_squared"]
                state[model + "_covered_95"] += int(score[model]["inside_95_predictive_ellipsoid"])
                state[model + "_floor_hit_trials"] += int(score[model]["prior_floor_eigenvalue_count"] > 0)
            state["sum_delta_nll"] += score["delta_nll"]
            state["recent_delta_nll"].append(score["delta_nll"])
        return self.summary()

    def summary(self):
        sensors = {}
        for sensor, state in self.sensors.items():
            count = state["scored_trials"]
            enabled = state["enabled_trials"]
            sensors[sensor] = {
                "enabled_trials": enabled,
                "received_trials": state["received_trials"],
                "scored_trials": count,
                "status_counts": dict(state["statuses"]),
                "observation_received_rate": state["received_trials"] / enabled if enabled else None,
                "scored_trial_rate": count / enabled if enabled else None,
                "positive_delta_trials": state["positive_delta_trials"],
                "mean_adaptive_nll": state["sum_adaptive_nll"] / count if count else None,
                "mean_fixed_nll": state["sum_fixed_nll"] / count if count else None,
                "mean_delta_nll": state["sum_delta_nll"] / count if count else None,
                "cumulative_delta_nll": state["sum_delta_nll"] if count else None,
                "recent_mean_delta_nll": float(np.mean(state["recent_delta_nll"])) if count else None,
                "adaptive_observation_rmse_m": float(np.sqrt(state["sum_adaptive_squared_error"] / count)) if count else None,
                "fixed_observation_rmse_m": float(np.sqrt(state["sum_fixed_squared_error"] / count)) if count else None,
            }
            for model in ("adaptive", "fixed"):
                sensors[sensor][model + "_mean_mahalanobis_squared"] = state["sum_" + model + "_d2"] / count if count else None
                sensors[sensor][model + "_predictive_coverage_95"] = state[model + "_covered_95"] / count if count else None
                sensors[sensor][model + "_floor_hit_trials"] = state[model + "_floor_hit_trials"]
        return {
            "schema_version": 1,
            "trial_count": self.trial_count,
            "latest_operation_id": self.latest_operation_id,
            "latest_learning_sample_count_before_trial": self.latest_learning_sample_count,
            "window_size": self.window_size,
            "interpretation": "paired_sensor_prediction_surrogate_unverified_target_and_independence",
            "sensors": sensors,
        }
