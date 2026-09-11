# Live Xsens to G1 Manual

## Status

The freshness-aware Xsens ROS 2 source, atomic ROS capture boundary, continuous
Pink retargeter, SONIC protocol publisher, and SONIC MuJoCo controller are
implemented. Live suit validation is still pending; keep this simulation-only.

This manual documents the measured live input contract, process layout,
operator procedure, and safety requirements. The adapter uses explicit
calibration and enable controls plus a `250 ms` unique-source watchdog. A stale
stream requests SONIC emergency stop and requires new calibration before resume.

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
        | atomic /tmp/xsens-live-frame.json snapshot
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
- Pink installed in the existing `tv` environment:

  ```bash
  ~/miniforge3/bin/conda install -n tv -c conda-forge pink -y
  ```

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

The adapter is implemented in this repository as two environment-isolated processes.

### ROS Capture Process

The implemented ROS process:

1. Subscribe to `/link_states` using ROS 2 Jazzy.
2. Build a dictionary by `state.header.frame_id`.
3. Require all 23 canonical segments before producing a frame.
4. Reorder and normalize quaternions.
5. Reads the quaternion datagram sample counter, frame time, and steady-clock
   receive timestamp from the extended `LinkStateArray` message.
6. Attach the source sequence/timestamp and local receive timestamp.
7. Reject non-finite values and implausible quaternion norms.
8. Atomically replaces `/tmp/xsens-live-frame.json`, preserving latest-frame
   semantics without adding non-ROS dependencies to Jazzy.

It must not import Isaac Sim, Pink, SONIC, or Unitree controller packages.

### Retargeting Process

The implemented retargeting process:

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
wire format. `retargeting.live_retarget` reuses `pack_protocol_v1` directly and
does not write temporary trajectory CSV files.

## Target Operator Procedure

Launch the PyQt console and use this procedure:

```bash
.conda-sonic-stream/bin/python -m retargeting.sonic_gui
```

1. Start SONIC MuJoCo and keep the elastic support enabled.
2. Click `START LIVE SUIT PIPELINE`; it starts the ROS source, atomic capture,
   and headless Pink process.
3. Stand upright in a documented neutral pose.
4. Wait for `LIVE WAITING_FOR_CALIBRATION`.
5. Click `CALIBRATE LIVE NEUTRAL`.
6. Verify the retargeted G1 preview before enabling dynamic control.
7. Click `ENABLE LIVE XSENS`; the GUI arms ZMQ, starts SONIC, and releases the
   graphical MuJoCo support after stabilization.
11. Keep the operator ready to press `O` in SONIC.

The GUI displays these steps as a live next-action panel. `ENABLE LIVE XSENS`
is accepted only when the complete pipeline is running, the current state is
`CALIBRATED_DISABLED`, and frame age is below `250 ms`. It then waits for a
confirmed rolling Protocol v1 window before arming or starting SONIC. If no
window appears within five seconds, control remains disabled and the GUI reports
an actionable error.

The header identifies exactly one requested input mode: `RECORDED PLAYBACK`,
`LIVE SUIT`, or `MOCK SIMULATION`. The control deck keeps the current phase and
wait reason visible, with countdown feedback during bounded warmup, decode, and
controller-start waits. Recorded playback is blocked while any live or mock
pipeline session exists; stop that pipeline before switching back. Switching,
stopping, resetting, disabling, or using emergency stop invalidates delayed GUI
callbacks so an earlier action cannot start control afterward.

The underlying commands are:

```text
python3 -m retargeting.live_capture_ros
conda run -n tv python -m retargeting.live_retarget
```

`retargeting.live_retarget` owns `tcp://127.0.0.1:5556` while running. Stop the
live pipeline before playing recorded clips from the GUI. Recorded and live
publishers are intentionally mutually exclusive.

## Mock Streaming Without a Suit

Select a recording in the GUI and click `START MOCK SIMULATION FROM SELECTED`.
This starts an MVNX replay process and the normal Pink live retargeter without
launching ROS or opening an Xsens UDP port. The replay writes the same atomic
`/tmp/xsens-live-frame.json` snapshots as the ROS capture process at 50 Hz and
loops at the end of the recording. While output is disabled, it holds the first
frame so calibration is deterministic. After enable, it keeps that frame for
three seconds to cover the live-reference warmup and supported controller
startup before advancing through the recording.

The mock path deliberately preserves the normal safety sequence: start SONIC
MuJoCo, wait for `LIVE WAITING_FOR_CALIBRATION`, click `CALIBRATE LIVE NEUTRAL`,
and only then click `ENABLE LIVE XSENS`. SONIC control starts only after the
50-frame warmup and confirmation that SONIC decoded `motion name: streamed`.
`RESET AFTER FALL` restarts the same mock recording and requires calibration
again.

The standalone source can also be run without the GUI:

```bash
python3 -m retargeting.mock_live_stream \
  "xsens_TJ_08072026/MVNX export/Bending forward.mvnx" \
  --loop
```

This command only produces live-frame snapshots. It does not start SONIC,
MuJoCo, the retargeter, or any hardware output.

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

The implemented watchdog rejects repeated and backward source counters, handles
uint32 wraparound, and requests SONIC stop after `250 ms` without a unique frame.
After a stale stop, incrementing the calibration generation is required before
live output resumes.

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
