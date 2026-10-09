# Local registration data contract

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

`[metadata, x, y, z]`. The current node preserves the legacy metadata field
but uses only indices 1--3.

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

The current stream configuration accepts `[x, y, z, ...]`, with one message
representing one observation frame. Temporal sample covariance is measured by
the fusion node. A fixed per-frame sensor covariance is added to the
representative-mean covariance.

If Meta supplies per-frame covariance, set `meta_candidate_stride: 12` and
`meta_covariance_indices: [3, 4, 5, 6, 7, 8, 9, 10, 11]` and use the same
12-value record as YOLO.

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
