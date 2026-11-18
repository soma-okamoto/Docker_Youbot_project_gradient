# Local registration data contract

For bottle-ID propagation and the Unity receiver changes, see
[bottle_id_integration.md](bottle_id_integration.md). With `enable_bottle_id: true`,
the default prior/fusion handoff uses `/P_pred_context`, and `/P_place_with_id` carries
the result with the original bottle ID. Existing output topics remain available.

For the node, launch, topic, and configuration overview, see
[system_architecture.md](system_architecture.md).

The ROS package estimates and publishes the three-dimensional bottle-center
position. Applying that position to an MR object is the Unity application's
responsibility.

## Common coordinate contract

All input positions must be expressed in metres in `base_footprint` and must
refer to the bottle centre. Upstream producers are responsible for:

- transforming sensor measurements into `base_footprint`;
- transforming the EE/grasp offset into a bottle-centre estimate;
- adding coordinate-transform and grasp-offset uncertainty to the supplied
  covariance; and
- publishing observations from the active Place operation only.

Array messages do not contain a ROS header. The fusion node therefore cannot
transform or timestamp-check `/P_current`, `/P_yolo`, or array-form `/P_meta`.

## Inputs

### `/P_current` (`std_msgs/Float32MultiArray`)

`[bottle_id, x, y, z]`. With `enable_bottle_id: true`, the prior preserves index 0
as the manipulated bottle's identity and returns it with the final position.
Without that setting, only indices 1--3 are used, as before. The analyzer keeps
the bottle ID separate from the operation ID; repeated operations on one bottle
produce separate trial rows. Context supplies the atomic raw source positions
when available; old bags retain arrival-based Current/prior pairing.

### `/P_tf` (`geometry_msgs/PoseStamped`)

Bottle-centre estimate derived from robot kinematics. `header.frame_id` must
match `expected_frame`.

### `/P_pred_context` (`place_estimation/PriorContext`)

The default pipeline enables `publish_prior_context` in the prior node and
`use_prior_source_guard` in fusion. One atomic message contains an explicit
`operation_id`, `bottle_id` (-1 with identity disabled), the prior mean/covariance,
raw Current/TF positions, and the bias-corrected Current/TF means and covariances
used before CI. This preserves alternative source hypotheses even when prior CI
gives one source zero weight. Fusion subscribes only to this context in guard
mode; it does not separately pair `/P_current`, `/P_tf`, or legacy prior topics.
Malformed context is rejected as a whole. Sources are cleared at finalization.
With the guard disabled, the previous prior input paths remain available.

The context also carries `learning_sample_count` at prior generation, the actual
Current CI weight, and a fixed-reference position/covariance/CI weight with
`fixed_prior_available`. The fixed reference uses immutable startup error
parameters and the same raw inputs and CI rule. These fields are evaluation data;
fusion still uses `position`/`covariance` and the existing source hypotheses.
Rebuild all message consumers after this schema change.

### Predictive evaluation snapshots

`/predictive_evaluation_input`, `/predictive_evaluation` and
`/predictive_evaluation_summary` use `std_msgs/String` JSON, schema version 1.
The input is an atomic operation-level snapshot combining frozen priors and
quality-valid sensor representatives before prior-distance rejection. Its
explicit `operation_id` comes from `PriorContext`, not a publisher's Header.seq.
The result retains this data and adds per-sensor paired scores. Missing scores
are JSON null; NaN and infinity are not emitted. See
[evaluation/README.md](../evaluation/README.md) for selection rules, assumptions,
outputs, and build instructions.

### `/P_yolo` (`std_msgs/Float32MultiArray`)

The configured packed record is:

`[x, y, z, Sxx, Sxy, Sxz, Syx, Syy, Syz, Szx, Szy, Szz]`.

In `packed` mode, one message contains already formed candidates. In `stream`
mode, one message represents one observation frame and the node forms temporal
tracks, representative means, and representative covariances.

### `/P_meta` (`std_msgs/Float32MultiArray` by default)

The current stream configuration accepts the same 12-value position/covariance
record as YOLO, as published by `track_P_meta.py`. One message represents one
observation frame and may contain multiple bottle candidates. Meta observations
must be published on `/P_meta`, separately from RealSense observations on
`/P_yolo`. Temporal sample covariance and the supplied per-frame covariance
contribute to the representative-mean covariance.

For an XYZ-only producer, set `meta_candidate_stride: 3` and remove
`meta_covariance_indices`; the node then uses the fixed per-frame sensor
covariance from `sigma_meta`.

For both sensors, a candidate must be within `candidate_gate_yolo` /
`candidate_gate_meta` (squared Mahalanobis distance, default 25.0) of at least one
reference: the fused prior or either same-operation, bias-corrected source.
Each distance uses that reference's covariance plus observation covariance.
Candidates near the fused prior take priority; source-only matches are rescues.
Among pairs satisfying `gate_yolo_meta`, selection minimizes the number of
rescued candidates, then the sum of ranking distances, then inter-sensor
distance. Ranking distance is prior distance for prior-supported candidates,
otherwise the minimum source distance. Single-sensor selection uses the same
priority. Candidates far from every reference are rejected even if they agree.

The tighter `gate_yolo` / `gate_meta` still use only the fused-prior distance to
decide whether the prior participates in CI. Rescuing an observation near Current
does not pull a distant TF-dominated prior into the final result. Existing
candidate-distance topics retain fused-prior distances; scores use
`exp(-ranking_distance/2)` and are zero for rejected candidates. No eligible
candidate gives selected index -1.

## Outputs

### `/P_place` (`geometry_msgs/PoseStamped`)

Estimated bottle-centre position for Unity. Orientation is the identity and is
not an estimated result.

### `/Sigma_place` (`std_msgs/Float32MultiArray`)

Row-major 3-by-3 covariance corresponding to `/P_place`.

### Diagnostics

`/observation_status`, `/observation_distances`, per-sensor candidate distances,
scores, and selected indices describe the selection and fusion branch.
`/observation_reference_distances` contains selected-candidate squared distances:
`[yolo_prior, yolo_current, yolo_tf, meta_prior, meta_current, meta_tf]`.
Missing sources or selections are NaN. Preliminary recording includes this
topic and `/P_pred_context`; CSV output includes raw/corrected source positions,
source covariances, and selected reference distances.

### `/used_physical_observation` (`geometry_msgs/PoseWithCovarianceStamped`)

The physical-observation distribution actually selected by the fusion node,
before any prior distribution is included. Its timestamp is copied from the
selected operation's prior. The prior node matches that timestamp to exactly
one pending operation (missing or ambiguous matches are ignored). `header.seq`
is the feedback publisher's message counter, not the Place operation ID;
`rospy` overwrites it during publication, and feedback is sparse.
The prior node uses this feedback to update the `P_current` and
`P_tf` bias and covariance for later operations. No message is published when
the result falls back to the prior without a physical observation.
Offline analysis uses the same timestamp matching. Old bags may contain
learning updates attributed to an incorrect operation by the former sequence
matching; the analyzer reports unmatched learning updates without rewriting
the recorded learned states.

### Learned-state diagnostics

After each accepted learning update, the prior node publishes:

- `/learning_sample_count` as `std_msgs/Int32`;
- `/learned_bias_current` and `/learned_bias_tf` as
  `[operation_id, sample_count, bx, by, bz]`; and
- `/learned_covariance_current` and `/learned_covariance_tf` as
  `[operation_id, sample_count, Sxx, Sxy, Sxz, Syx, Syy, Syz, Szx, Szy, Szz]`.

The four array topics use `std_msgs/Float64MultiArray` and are latched so that
recording and offline calibration can capture the latest learned state.
