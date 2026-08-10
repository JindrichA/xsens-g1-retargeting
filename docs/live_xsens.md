# Live Xsens to G1 Manual

## Status

The Xsens ROS 2 source, offline G1 retargeter, SONIC protocol publisher, and
SONIC MuJoCo controller work independently. The live adapter between ROS 2
`/link_states` and continuous G1 inverse kinematics is not implemented yet.

This manual documents the measured live input contract, the intended process
layout, operator procedure, safety requirements, and implementation checklist.
Commands marked as working can be used now. The future adapter command is shown
as a target interface, not as an existing executable.

Real-robot output is outside the scope of this manual. Keep SONIC on its
simulation profile throughout development.

## Dataflow

```text
Xsens MVN Studio
        |
        | UDP 9763: position + quaternion and joint angles
        v
xsens_mvn_ros2 (ROS 2 Jazzy)
        |
        | /link_states at 100 Hz
        v
ROS capture process
        |
        | canonical named Xsens frame over ZMQ or an atomic file
        v
Pink retargeting process at 50 Hz
        |
        | 29 G1 joints + root quaternion
        v
SONIC packed ZMQ Protocol v1 on port 5556
        |
        v
GEAR-SONIC -> G1 MuJoCo
```

ROS 2 Jazzy, the Pink/Pinocchio runtime, and SONIC are separate environments.
Do not install their dependencies into one environment. Use ZMQ or an atomic
file at the process boundaries.

## Prerequisites

- Xsens MVN Studio with a calibrated Link or Awinda suit.
- The Ubuntu workstation and MVN Studio machine on the same trusted LAN.
- UDP port `9763` allowed from MVN Studio to Ubuntu.
- ROS 2 Jazzy at `/opt/ros/jazzy`.
- Built sibling checkout at `~/Projects/xsens_mvn_ros2`.
- This repository's recorded-motion retargeting baseline validated first.
- The SONIC simulation setup from [SONIC Simulation Manual](sonic_simulation.md).

If MVN Studio runs on Windows, determine the Ubuntu workstation's LAN address:

```bash
ip -4 address
```

Use the address on the same LAN as the MVN Studio machine. Do not use
`127.0.0.1` unless MVN Studio and ROS run on the same computer.

## Configure MVN Studio

Open **Options -> Network Streamer** and add a destination:

| Setting | Value |
| --- | --- |
| Host | Ubuntu workstation LAN IP |
| Port | `9763` |
| Position + Orientation | Enabled, quaternion format |
| Joint Angles | Enabled |

The existing ROS driver needs both position/quaternion and joint-angle
datagrams while constructing the model. If Windows sends to Ubuntu, allow MVN
Studio through Windows Firewall and verify that both machines can reach each
other.

## Start the ROS Source

The working, minimal launch without RViz is:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Projects/xsens_mvn_ros2/install/setup.bash

ros2 launch xsens_mvn_ros2 xsens_stream.launch.py \
  launch_rviz:=false \
  launch_description:=false
```

The full visualization launch is:

```bash
ros2 launch xsens_mvn_ros2 xsens_stream.launch.py
```

The normal source is MVN Studio's UDP stream. The alternative
`xsens_xme.launch.py` path requires an XME license, body dimensions, and a
separate calibration workflow.

## Verify ROS Data

In another ROS 2 Jazzy terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Projects/xsens_mvn_ros2/install/setup.bash

ros2 topic list
ros2 topic hz /link_states
ros2 topic echo /link_states --once
ros2 topic echo /diagnostics
```

Expected topics:

| Topic | Type | Content |
| --- | --- | --- |
| `/link_states` | `xsens_mvn_msgs/msg/LinkStateArray` | Absolute segment pose, twist, and acceleration |
| `/joint_states` | `sensor_msgs/msg/JointState` | Three angle entries per human joint |
| `/com` | `geometry_msgs/msg/Point` | Absolute center of mass |
| `/tf` | `tf2_msgs/msg/TFMessage` | Root-global and parent-relative transforms |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Stream status at 1 Hz |

The ROS node publishes the latest cached source sample at 100 Hz. A 100 Hz ROS
rate does not prove MVN Studio is providing 100 unique measurements. The current
driver stamps each ROS publication with the current ROS time, including repeated
cached samples. `/link_states` timestamps therefore cannot detect a frozen MVN
UDP source. The one-Hz `/diagnostics` topic reports staleness but is too slow for
a `100-200 ms` dynamic watchdog.

Before live dynamic control, extend the ROS driver to expose the source datagram
counter and last-UDP-receive time in every published frame, or add an equivalent
raw-UDP capture boundary. Do not infer source freshness from ROS topic frequency
or the current publication timestamps.

## `/link_states` Contract

Each `LinkStateArray.states` element is identified by
`state.header.frame_id`. Never depend on array order: the current publisher
iterates an unordered map.

Expected frame names are:

```text
skeleton_pelvis
skeleton_l5
skeleton_l3
skeleton_t12
skeleton_t8
skeleton_neck
skeleton_head
skeleton_right_shoulder
skeleton_right_upper_arm
skeleton_right_forearm
skeleton_right_hand
skeleton_left_shoulder
skeleton_left_upper_arm
skeleton_left_forearm
skeleton_left_hand
skeleton_right_upper_leg
skeleton_right_lower_leg
skeleton_right_foot
skeleton_right_toe
skeleton_left_upper_leg
skeleton_left_lower_leg
skeleton_left_foot
skeleton_left_toe
```

Important conversions:

- Positions are absolute, right-handed, and Z-up.
- ROS quaternion fields are `(x, y, z, w)`; the canonical retargeter uses
  `(w, x, y, z)`.
- Remove the `skeleton_` prefix and map snake-case names to canonical MVNX names
  such as `Pelvis`, `T8`, `LeftHand`, and `RightFoot`.
- The ROS publisher applies local arm-frame rotations: `-90 degrees` about X on
  the right and `+90 degrees` about X on the left. The adapter must either undo
  these rotations or define and validate a live-specific neutral calibration.
- `/link_states` does not contain the MVNX heel/toe contact flags. Contact must
  initially be inferred from foot height and velocity, or omitted explicitly.

## Live Adapter Responsibilities

The adapter should be implemented in this repository as two processes.

### ROS Capture Process

The ROS process should:

1. Subscribe to `/link_states` using ROS 2 Jazzy.
2. Build a dictionary by `state.header.frame_id`.
3. Require all 23 canonical segments before producing a frame.
4. Reorder and normalize quaternions.
5. Read an explicit source datagram sequence and last-receive timestamp from an
   extended driver; these values are not available in the current message.
6. Attach the source sequence/timestamp and local receive timestamp.
7. Reject non-finite values and implausible quaternion norms.
8. Publish only the latest canonical frame over a dedicated boundary such as
   ZMQ port `5558`.

It must not import Isaac Sim, Pink, SONIC, or Unitree controller packages.

### Retargeting Process

The retargeting process should:

1. Require an explicit upright neutral-calibration action.
2. Reuse the same pelvis-relative torso, wrist, ankle, and root target logic as
   recorded MVNX playback.
3. Solve constrained G1 IK at 50 Hz.
4. Enforce joint limits plus position, velocity, acceleration, and pose-jump
   limits.
5. Maintain monotonically increasing live frame indexes.
6. Pack short rolling Protocol v1 chunks rather than a complete CSV recording.
7. Publish to SONIC's local `pose` endpoint on port `5556`.

The existing `retargeting.sonic_stream` module provides the validated packed
wire format and accepts explicit strictly increasing `frame_indices`, but its
command-line interface currently loads a complete exported trajectory. The live
implementation should reuse `pack_protocol_v1`, not write temporary CSV files
for every frame.

## Target Operator Procedure

This is the intended procedure after the adapter exists:

1. Start SONIC MuJoCo and keep the elastic support enabled.
2. Start the Xsens ROS source and verify `/diagnostics` and `/link_states`.
3. Stand upright in a documented neutral pose.
4. Start the ROS capture process.
5. Request neutral calibration in the retargeting process.
6. Verify the retargeted G1 preview before enabling dynamic control.
7. Press `]` in SONIC to start simulation control.
8. Press `ENTER` in SONIC to enable ZMQ streaming.
9. Release the MuJoCo elastic support with `9`.
10. Explicitly enable live publication.
11. Keep the operator ready to press `O` in SONIC.

The future command interface should make calibration and enable actions
explicit, for example:

```text
python -m retargeting.live_xsens capture ...      # planned, not implemented
python -m retargeting.live_xsens calibrate ...    # planned, not implemented
python -m retargeting.live_xsens stream --sim ... # planned, not implemented
```

## Watchdog and Fail-Safe Requirements

Before live dynamic testing, implement and unit-test these states:

| State | Required behavior |
| --- | --- |
| No data | Do not initialize or publish |
| Incomplete skeleton | Reject frame and report missing names |
| Waiting for calibration | Preview only; no SONIC output |
| Calibrated but disabled | Hold local state; no SONIC output |
| Enabled | Publish bounded 50 Hz references |
| Stale input | Stop publication and enter a defined simulated hold/stop state |
| Large pose jump | Reject or blend; never snap immediately |
| Restart after pause | Require pose proximity or a new blend/calibration |

After a true UDP freshness signal exists, start with a `100-200 ms` stale
threshold and tune it from measured network jitter. Stopping ZMQ publication
alone may leave SONIC holding its last target, so the final fail-safe must be
tested with the controller's state behavior in MuJoCo.

Inject frozen, delayed, duplicated, reordered, and dropped frames in unit tests.
Tests must not launch ROS, MuJoCo, Isaac Sim, or the controller.

## Initial Simulation Acceptance Criteria

Do not connect a physical G1 until the live simulation can demonstrate:

- No unexpected publication before calibration and enable.
- Bounded startup and resume transitions.
- Correct left/right and quaternion mapping.
- Automatic stale-input response.
- No joint-limit or configured speed/acceleration violations.
- Recorded fall count, root height and tilt, foot slip, target tracking error,
  and torque saturation.
- Repeatable restricted upper-body trials before stepping or floor motions.

The current recorded baseline already falls on an easy upright clip, so live
input is an integration milestone, not evidence that unrestricted live motion
is dynamically safe.

## Troubleshooting

### No `/link_states`

- Confirm both MVN datagram types are enabled.
- Confirm destination IP and UDP port `9763`.
- Check Windows Firewall or the Ubuntu firewall.
- Confirm the ROS lifecycle node reached the active state.
- Start a subscriber: the driver only constructs link-state output when it has
  at least one subscriber.

### ROS shows 100 Hz but motion freezes

The ROS timer republishes cached data. Inspect `/diagnostics` and timestamps;
do not treat topic frequency or current `/link_states` publication timestamps as
source freshness. Dynamic streaming must wait for the driver freshness extension
described above.

### Arms have incorrect orientation

Check quaternion reordering and the left/right local arm rotations added by the
ROS publisher. Compare an upright neutral pose against the same pose exported
to MVNX before changing IK weights.

### SONIC receives nothing

- Confirm the controller says `ZMQ STREAMING MODE: ENABLED`.
- Confirm only one process binds TCP port `5556`.
- Confirm the publisher uses topic prefix `pose` and a 1280-byte packed header.
- Keep ROS capture on a different port, such as `5558`.

## GMR Alternative

GMR supports live Xsens MVN UDP retargeting to Unitree G1 and is useful as an
independent comparison. It can reduce the amount of retargeting work, but a
GMR-to-SONIC protocol adapter, watchdog, calibration, and the same simulation
safety validation are still required. Do not describe GMR visualization alone
as dynamically controlled whole-body retargeting.
