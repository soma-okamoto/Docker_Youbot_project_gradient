# Local registration data contract

For bottle-ID propagation and the Unity receiver changes, see
[bottle_id_integration.md](bottle_id_integration.md). With `enable_bottle_id: true`,
the prior/fusion handoff uses `/P_pred_with_id`, and `/P_place_with_id` carries
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
Without that setting, only indices 1--3 are used, as before. The existing
preliminary-experiment analyzer may interpret index 0 as an operation ID;
that is distinct from a bottle ID and does not distinguish repeated operations
on the same bottle.

### `/P_tf` (`geometry_msgs/PoseStamped`)

Bottle-centre estimate derived from robot kinematics. `header.frame_id` must
match `expected_frame`.

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

## Outputs

### `/P_place` (`geometry_msgs/PoseStamped`)

Estimated bottle-centre position for Unity. Orientation is the identity and is
not an estimated result.

### `/Sigma_place` (`std_msgs/Float32MultiArray`)

Row-major 3-by-3 covariance corresponding to `/P_place`.

### Diagnostics

`/observation_status`, `/observation_distances`, per-sensor candidate distances,
scores, and selected indices describe the selection and fusion branch.

### `/used_physical_observation` (`geometry_msgs/PoseWithCovarianceStamped`)

The physical-observation distribution actually selected by the fusion node,
before any prior distribution is included. Its header sequence is the Place
operation ID. The prior node uses this feedback to update the `P_current` and
`P_tf` bias and covariance for later operations. No message is published when
the result falls back to the prior without a physical observation.

### Learned-state diagnostics

After each accepted learning update, the prior node publishes:

- `/learning_sample_count` as `std_msgs/Int32`;
- `/learned_bias_current` and `/learned_bias_tf` as
  `[operation_id, sample_count, bx, by, bz]`; and
- `/learned_covariance_current` and `/learned_covariance_tf` as
  `[operation_id, sample_count, Sxx, Sxy, Sxz, Syx, Syy, Syz, Szx, Szy, Szz]`.

The four array topics use `std_msgs/Float64MultiArray` and are latched so that
recording and offline calibration can capture the latest learned state.
