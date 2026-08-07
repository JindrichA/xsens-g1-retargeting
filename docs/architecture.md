# Target Architecture

## Goal

Make a Unitree G1 reproduce a person’s Xsens whole-body motion. Development is
offline-first and simulation-first; a live suit and physical robot are later
substitutions at the two ends of the same tested pipeline.

```text
MVNX recording                         Live Xsens suit
      |                                      |
      +---------- canonical Xsens frame -----+
                            |
                 calibration and normalization
                            |
             human-to-G1 kinematic retargeting
                            |
              29 G1 joints at 50 Hz + root pose
                            |
          limits, continuity checks, stale watchdog
                            |
                  SONIC ZMQ Protocol v1
                            |
          GR00T SONIC whole-body tracking policy
                            |
              G1 MuJoCo -> eventually real G1
```

## Established Contracts

The canonical recorded input is MVNX, not BVH:

- 240 Hz, 23 body segments, 22 body joints.
- Global positions are meters in X-forward, Y-left, Z-up coordinates.
- Global quaternions are scalar-first `(w, x, y, z)`.
- Heel/toe contact flags and center-of-mass state are present.
- Eight recordings include finger tracking.
- Canonical time is `frame_index / 240`; exported source timestamps contain
  discontinuities and are diagnostic only.

The initial controller output is SONIC ZMQ Protocol v1:

- `joint_pos`: `[N, 29]` radians, IsaacLab order.
- `joint_vel`: `[N, 29]` rad/s, IsaacLab order.
- `body_quat`: root quaternion in `(w, x, y, z)` order.
- `frame_index`: monotonically increasing.
- Reference rate: 50 Hz.

MVNX and SONIC share the same world axes and quaternion ordering. ROS messages
are the exception: ROS quaternions are `(x, y, z, w)` and must be reordered.

## Retargeting Boundary

Human joint angles must not be copied to G1 joints. The embodiment differs in
topology, segment lengths, ranges, and rotation axes. The retargeter should use
global segment targets, robot forward kinematics, and constrained IK:

- Pelvis/root pose and heading.
- Torso orientation with conservative waist limits.
- Wrist pose objectives and elbow position preferences.
- Foot pose and contact objectives with stance-foot locking.
- Posture regularization toward a stable G1 configuration.
- Joint-position, velocity, acceleration, and self-collision constraints.

GMR is the leading existing implementation to evaluate for this kinematic step.
It is not currently present under `~/Projects`. Its output remains a reference
trajectory, not safe motor commands; SONIC provides the dynamic tracking layer.

## Milestones

1. Parse and validate every MVNX recording reproducibly.
2. Retarget one simple clip to G1 and visualize it kinematically.
3. Export 50 Hz SONIC reference files and validate joint order and limits.
4. Track the clip through SONIC in MuJoCo and measure falls, foot slip, tracking
   error, clipping, and target discontinuities.
5. Replace MVNX playback with ROS 2 `/link_states` while keeping the exact same
   canonical frame and retargeting implementation.
6. Add packet-loss, stale-input, pause/resume, and large-pose-jump tests.
7. Introduce real hardware only behind an explicit profile after simulation
   acceptance criteria pass.
