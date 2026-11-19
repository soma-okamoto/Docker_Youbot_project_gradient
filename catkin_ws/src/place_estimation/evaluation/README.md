# Predictive and distribution-update evaluation

## Paired sensor prediction scores

The default pipeline starts `predictive_evaluation_node.py` with
`config/predictive_evaluation.yaml`. `PriorContext` contains both the actual
adaptive prior and a fixed prior made from the same raw Current/TF inputs using
the startup biases/covariances and the same CI search rule. The fixed model
never learns. `learning_sample_count` is captured at prior generation, before
this operation's feedback. Its value counts accepted updates, not Place trials.
TF positions, the operational selection/fusion rules, and the learning equations
are unchanged by this evaluation feature.

Fusion copies the prior snapshot when an operation starts. At finalization it
publishes `/predictive_evaluation_input` (`std_msgs/String`, JSON) containing:

- explicit operation ID, bottle ID, prior timestamp/frame and pre-trial learning count;
- both frozen priors, source hypotheses and their CI weights;
- per-sensor quality-valid representative positions/covariances, before prior
  candidate-distance gates, including candidates rejected by those gates;
- selected indices, selection for physical feedback, trigger and operational status.

In stream mode these are temporal track representatives, not individual frames.
In packed mode they are the first accepted packed message's candidates. Sensor
quality and temporal stability checks are the existing checks. The snapshot
publication happens after the operational output; later feedback cannot alter
the frozen priors. Evaluation failure does not replace a Place result or change
learning feedback. An explicit `PriorContext` is required; legacy split prior
inputs do not generate predictive snapshots.

The evaluator publishes `/predictive_evaluation` (JSON) and the latched
`/predictive_evaluation_summary` (JSON). For each sensor separately it computes:

```text
V = Sigma_prior + R_sensor
NLL = (3 log(2 pi) + log det(V) + (y - mu)' V^-1 (y - mu)) / 2
Delta_NLL = NLL_fixed - NLL_adaptive
```

Positive Delta_NLL favors the adaptive prediction on that observation. Natural
logarithms, metres and m² are used; negative NLL is valid. Cholesky factorization
is used, and invalid covariance is rejected. Numerical eigenvalue flooring and
prior floor hits are reported. Position error, squared error, Mahalanobis
residual, predictive covariance diagnostics and membership in the nominal 95%
3-D Gaussian ellipsoid are also reported.

Trial summaries require **exactly one quality-valid candidate**, without using
the tested priors to choose among multiple bottles. More than one candidate is
`ambiguous_candidates` and has no aggregate trial score. All quality candidates
and their paired scores remain in the JSON/CSV for subsequent target annotation.
`selected_diagnostic` scores the operationally selected candidate on both models,
with `selected_diagnostic_selection_dependent: true`; it is never included in
the aggregate trial scores. Its selected index may describe a candidate from a
sensor rejected in a sensor conflict; consult `selected_for_feedback` and status.

Missing/disabled/invalid observations have no score (`null`), rather than zero.
Each sensor's summary includes scored/missing/ambiguous counts, receipt/scoring
rates, average and cumulative paired Delta_NLL, a recent scored-trial window,
observation RMSE, Gaussian coverage diagnostics and covariance-floor hits.
Scores are not pooled across sensors. The running node suppresses duplicate
operation/timestamp/frame keys within a bounded recent history.

### Interpretation and assumptions

These are **sensor prediction surrogate scores**, not verified physical accuracy
or a verdict on personal adaptation. A unique candidate does not establish that
the detected bottle is the manipulated bottle; `target_identity_verified` is
false. The Sigma+R model assumes independent prior/observation errors, Gaussian
predictive errors and correctly calibrated covariance. Those assumptions remain
unverified, and the outputs explicitly record that fact. Common registration/TF
errors, temporal correlation, Meta covariance double counting, and the learned
observation-covariance subtraction still require separate calibration. Nominal
coverage diagnostics are subject to these same assumptions. The current learning
equations, minimum sample count and covariance floor are retained.

For research claims, establish target correspondence independently, calibrate
sensor covariance on separate data, and compare user-specific history with
common/other-user models under matched operation conditions. No evaluation result
is fed back to the estimator. Use one process session per recorded experiment;
cross-session operation-ID reuse in a single bag is not supported by the legacy
trial CSV extractor.

### Build and use

`PriorContext.msg` has new fields and a new ROS message MD5. Rebuild the catkin
workspace and source the rebuilt setup in **every terminal running these nodes**
before starting a new experiment. Mixing old and new generated message classes
will prevent their connection. Existing bags retain their recorded old schema;
the analyzer reads them but cannot invent missing fixed priors or representatives.

```bash
catkin_make
source devel/setup.bash
roslaunch place_estimation preliminary_experiment.launch output_bag:=/tmp/place_nll_01
```

Run these commands from the catkin workspace root. Wait for the recorder and
evaluation subscribers to connect before performing Place operations. Allow the
previous observation window/feedback to finish if each next trial must include
that update. Overlapping operations retain their actual pre-trial learning count.

## KL evaluation

This folder is intentionally separated from the position-estimation nodes.
It evaluates how much the observation result changes the prior distribution;
it does not affect `P_place`.

The primary metric is

`D_KL(prior || posterior)`

where the posterior is `(P_place, Sigma_place)` and the prior is
`(P_pred, Sigma_pred)`. The node also reports the reverse KL, their symmetric
mean, Euclidean mean shift, and the mean/covariance components of the primary
KL. KL values use natural logarithms and are measured in nats.

`/kl_evaluation` contains:

1. operation ID
2. prior-to-posterior KL (legacy primary direction)
3. posterior-to-prior KL (diagnostic)
4. symmetric KL
5. mean shift [m]
6. prior-to-posterior mean component
7. prior-to-posterior covariance component
8. running mean of prior-to-posterior KL
9. running sample standard deviation
10. exponential moving average

A decreasing KL trend means that prior and posterior distributions are becoming
more similar. It does not by itself demonstrate improved accuracy against the
physical ground truth.

## Trial-sequence adaptation summary

`/kl_adaptation_summary` is a latched JSON message updated after every trial.
It summarizes the manuscript-direction KL sequence with:

- the first, latest, minimum and maximum values;
- initial-window and recent-window means;
- relative improvement from the initial to the recent window;
- full-history and recent-window slopes; and
- separate trends for total KL, mean shift, mean contribution and covariance
  contribution.

The legacy `adaptation_state` labels are retained for compatibility. They describe
disagreement trends, and do not establish prediction improvement even after
`trend_min_samples`. Summaries now include
`interpretation: prior_posterior_disagreement_only` and
`adaptation_claim_supported: false`. A fallback to the prior can give zero KL
without successful observation or learning. CI can also select zero observation
weight. Both KL directions remain available as update diagnostics.

The number of trials is not predetermined because it depends on the human
operator and the experiment. The summary is updated after every completed
trial. The last `/kl_adaptation_summary` message recorded when the operator
ends the experiment is treated as the final summary.
