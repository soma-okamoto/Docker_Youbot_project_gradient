# KL evaluation

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
2. prior-to-posterior KL (the manuscript's adaptation index)
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

The `adaptation_state` is `insufficient_data`, `adapting`, `adapted_stable`,
`worsening`, or `mixed`. It becomes a reliable judgment after
`trend_min_samples`. A `mixed` result is intentionally reported when the KL
and physical mean-shift trends disagree, because covariance contraction can
increase KL even while the position difference decreases.

The number of trials is not predetermined because it depends on the human
operator and the experiment. The summary is updated after every completed
trial. The last `/kl_adaptation_summary` message recorded when the operator
ends the experiment is treated as the final summary.
