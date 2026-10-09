"""Unit tests for Gaussian KL evaluation."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np


path = Path(__file__).resolve().parents[1] / 'evaluation/gaussian_kl.py'
spec = importlib.util.spec_from_file_location('gaussian_kl_under_test', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class GaussianKLTests(unittest.TestCase):
    def test_identical_distributions_have_zero_kl(self):
        mean = np.array([1., 2., 3.])
        covariance = np.diag([.1, .2, .3])
        result = module.evaluate_prior_posterior(
            mean, covariance, mean, covariance
        )
        self.assertAlmostEqual(result['kl_posterior_to_prior'], 0.)
        self.assertAlmostEqual(result['kl_prior_to_posterior'], 0.)
        self.assertAlmostEqual(result['symmetric_kl'], 0.)
        self.assertAlmostEqual(result['mean_shift_m'], 0.)

    def test_mean_shift_component_matches_closed_form(self):
        covariance = np.eye(3)
        result = module.evaluate_prior_posterior(
            np.zeros(3), covariance, np.array([1., 0., 0.]), covariance
        )
        self.assertAlmostEqual(result['kl_posterior_to_prior'], .5)
        self.assertAlmostEqual(result['kl_prior_to_posterior'], .5)
        self.assertAlmostEqual(
            result['posterior_to_prior_mean_component'], .5
        )
        self.assertAlmostEqual(
            result['posterior_to_prior_covariance_component'], 0.
        )
        self.assertAlmostEqual(
            result['prior_to_posterior_mean_component'], .5
        )
        self.assertAlmostEqual(
            result['prior_to_posterior_covariance_component'], 0.
        )

    def test_covariance_change_is_asymmetric_and_nonnegative(self):
        result = module.evaluate_prior_posterior(
            np.zeros(3), np.eye(3), np.zeros(3), 2. * np.eye(3)
        )
        expected_posterior_to_prior = .5 * (3. - 3. * np.log(2.))
        expected_prior_to_posterior = .5 * (-1.5 + 3. * np.log(2.))
        self.assertAlmostEqual(
            result['kl_posterior_to_prior'], expected_posterior_to_prior
        )
        self.assertAlmostEqual(
            result['kl_prior_to_posterior'], expected_prior_to_posterior
        )
        self.assertAlmostEqual(
            result['prior_to_posterior_covariance_component'],
            expected_prior_to_posterior,
        )
        self.assertAlmostEqual(
            result['symmetric_kl'],
            .5 * (expected_posterior_to_prior + expected_prior_to_posterior),
        )

    def test_invalid_covariance_is_rejected(self):
        invalid_covariances = [
            np.zeros((3, 3)),
            np.diag([1., 1., -1.]),
            np.full((3, 3), np.nan),
        ]
        for covariance in invalid_covariances:
            with self.subTest(covariance=covariance), self.assertRaises(
                ValueError
            ):
                module.gaussian_kl_components(
                    np.zeros(3), covariance, np.zeros(3), np.eye(3)
                )


if __name__ == '__main__':
    unittest.main()
