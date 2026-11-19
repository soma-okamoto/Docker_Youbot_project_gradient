#!/usr/bin/env python3
"""KL-divergence utilities for multivariate Gaussian distributions."""
from typing import Dict

import numpy as np


def _validated_gaussian(mean, covariance):
    mean = np.asarray(mean, dtype=float)
    covariance = np.asarray(covariance, dtype=float)
    if mean.ndim != 1:
        raise ValueError("mean must be a one-dimensional vector")
    if covariance.shape != (mean.size, mean.size):
        raise ValueError("covariance shape does not match mean dimension")
    if not np.all(np.isfinite(mean)) or not np.all(np.isfinite(covariance)):
        raise ValueError("mean and covariance must be finite")
    covariance = 0.5 * (covariance + covariance.T)
    if np.any(np.linalg.eigvalsh(covariance) <= 0.0):
        raise ValueError("covariance must be positive definite")
    return mean, covariance


def gaussian_kl_components(mean_p, covariance_p, mean_q, covariance_q) -> Dict[str, float]:
    """Return D_KL(P||Q) and its mean/covariance components.

    P and Q are multivariate Gaussian distributions. All returned KL values
    are dimensionless (nats).
    """
    mean_p, covariance_p = _validated_gaussian(mean_p, covariance_p)
    mean_q, covariance_q = _validated_gaussian(mean_q, covariance_q)
    if mean_p.shape != mean_q.shape:
        raise ValueError("Gaussian dimensions do not match")

    dimension = mean_p.size
    delta = mean_q - mean_p
    sign_p, logdet_p = np.linalg.slogdet(covariance_p)
    sign_q, logdet_q = np.linalg.slogdet(covariance_q)
    if sign_p <= 0.0 or sign_q <= 0.0:
        raise ValueError("covariance determinant must be positive")

    trace_term = float(
        np.trace(np.linalg.solve(covariance_q, covariance_p))
    )
    mean_term = 0.5 * float(
        delta.T @ np.linalg.solve(covariance_q, delta)
    )
    covariance_term = 0.5 * float(
        trace_term - dimension + logdet_q - logdet_p
    )
    total = mean_term + covariance_term
    if total < 0.0 and total > -1.0e-10:
        total = 0.0
    if covariance_term < 0.0 and covariance_term > -1.0e-10:
        covariance_term = 0.0
    return {
        "total": total,
        "mean": mean_term,
        "covariance": covariance_term,
    }


def evaluate_prior_posterior(
    prior_mean,
    prior_covariance,
    posterior_mean,
    posterior_covariance,
) -> Dict[str, float]:
    """Evaluate posterior/prior KL in both directions and symmetrically."""
    posterior_to_prior = gaussian_kl_components(
        posterior_mean,
        posterior_covariance,
        prior_mean,
        prior_covariance,
    )
    prior_to_posterior = gaussian_kl_components(
        prior_mean,
        prior_covariance,
        posterior_mean,
        posterior_covariance,
    )
    return {
        "kl_posterior_to_prior": posterior_to_prior["total"],
        "kl_prior_to_posterior": prior_to_posterior["total"],
        "symmetric_kl": 0.5 * (
            posterior_to_prior["total"] + prior_to_posterior["total"]
        ),
        "mean_shift_m": float(
            np.linalg.norm(
                np.asarray(posterior_mean, dtype=float)
                - np.asarray(prior_mean, dtype=float)
            )
        ),
        "posterior_to_prior_mean_component": posterior_to_prior["mean"],
        "posterior_to_prior_covariance_component": posterior_to_prior[
            "covariance"
        ],
        "prior_to_posterior_mean_component": prior_to_posterior["mean"],
        "prior_to_posterior_covariance_component": prior_to_posterior[
            "covariance"
        ],
    }
