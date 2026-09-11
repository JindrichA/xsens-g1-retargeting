# Start Guide: Xsens to Unitree G1 Retargeting

This guide explains the project for someone new to robotics, motion capture,
Isaac Sim, and robot control.

The project goal is:

> A person wears an Xsens suit, moves their body, and a Unitree G1 humanoid
> reproduces the motion as closely and safely as its body allows.

We are developing this in stages. We first use recorded Xsens data and a
kinematic robot in simulation. A dynamically balanced simulation and live Xsens
stream come later. The real robot is the final stage, not the development
default.

## 1. Current Status

The following path works now:

```text
Recorded Xsens MVNX file
        |
        v
Read 23 human body-segment poses at 240 Hz
        |
        v
Resample to 50 Hz
        |
        v
Retarget human torso, wrists, feet, and pelvis to Unitree G1
        |
        v
Solve G1 inverse kinematics with Pink
        |
        v
Display the resulting G1 motion in Isaac Sim
```

This is **kinematic visualization**. The program places the simulated robot into
the calculated poses. Gravity is disabled so we can inspect retargeting without
the robot falling.

The dynamic recorded-motion path now runs, but is not stable yet:

```text
Recorded Xsens -> SONIC -> G1 under gravity (first test completed with one fall)
```

The missing major stages are dynamically stable retargeting, quantitative
dynamic metrics, live input conversion, runtime safety, and hardware validation.

## 2. Important Concepts

### Motion Capture

Motion capture measures a person's movement. Xsens uses inertial sensors worn
on the body. Xsens software estimates the position and orientation of body
segments such as the pelvis, torso, hands, upper legs, feet, and toes.

### Skeleton

A skeleton is a simplified model of a person or robot made from rigid links and
joints.

- The Xsens skeleton represents a human.
- The Unitree G1 skeleton represents a robot.
- Their proportions, joint axes, and joint ranges are different.

### Retargeting

Retargeting converts motion from one skeleton to another.

It is not safe or correct to copy human joint angles directly to robot joints.
For example, the human shoulder and G1 shoulder have different structures. The
retargeter instead asks the G1 to place its wrist, torso, and foot near target
poses derived from the human.

### Pose

A pose describes both:

- Position: where something is, represented by `(x, y, z)`.
- Orientation: which way it points, represented here by a quaternion.

### Quaternion

A quaternion represents 3D orientation without the singularities associated
with Euler angles. Xsens MVNX and the current simulation path use:

```text
(w, x, y, z)
```

ROS usually stores a quaternion in a different order:

```text
(x, y, z, w)
```

This ordering difference must be handled when live ROS input is implemented.

### Degrees of Freedom

A degree of freedom, or DOF, is one independently movable joint axis. The G1
body model used for retargeting has 29 controlled DOFs. The visible Isaac asset
may contain additional hand joints.

### Forward Kinematics

Forward kinematics calculates body-part positions from known joint angles:

```text
joint angles -> wrist and foot positions
```

### Inverse Kinematics

Inverse kinematics, or IK, solves the reverse problem:

```text
desired wrist and foot positions -> robot joint angles
```

Pink is the IK solver currently used by this project.

### Kinematic Versus Dynamic Simulation

Kinematic simulation places the robot into requested poses. It answers:

- Does the mapping look correct?
- Are left and right mirrored correctly?
- Are the joint angles valid?
- Does the robot roughly reproduce the person's movement?

Dynamic simulation includes gravity, mass, collisions, torque, momentum, and
balance. It answers:

- Can a controller physically execute the motion?
- Does the robot fall?
- Do its feet slip?
- Are the required forces and torques feasible?

The current Isaac viewer is kinematic by default. Running it with `--dynamic`
enables gravity without a balancing controller, so falling is expected.

## 3. Tools Used by the Project

### Xsens MVN / Xsens Humanoid

Purpose: capture and process human motion from the suit.

It can provide:

- Segment positions and orientations.
- Human joint angles.
- Center of mass.
- Estimated heel and toe contacts.
- Optional finger tracking.
- Recorded files and live network streaming.

The Xsens application runs outside this repository. Recorded exports are
already available here for offline development.

### MVNX

Purpose: canonical recorded Xsens input for this project.

MVNX is an XML file containing motion frames and metadata. Our recordings use:

- 240 frames per second.
- 23 body segments.
- 22 body joints.
- Positions in meters.
- X-forward, Y-left, Z-up coordinates.
- Scalar-first `(w, x, y, z)` quaternions.
- Four heel/toe contact flags.
- Center-of-mass position, velocity, and acceleration.
- Optional finger data.

MVNX is preferred over BVH because MVNX preserves contacts, center of mass,
metadata, and the real first frame.

### ROS 2 Jazzy

Purpose: carry live Xsens data between processes.

ROS 2 is middleware commonly used in robotics. It lets independent programs
publish and subscribe to named data streams called topics.

The sibling repository `../xsens_mvn_ros2` can publish live Xsens data on topics
including:

```text
/link_states
/joint_states
/tf
/com
/diagnostics
```

ROS is not needed to play the recorded MVNX files. It will become important
when recorded playback is replaced by a live suit.

### Isaac Sim

Purpose: visualize and simulate robots in a 3D environment.

The installed Isaac Sim provides:

- A 3D viewport.
- USD scene and robot assets.
- Articulated robot support.
- PhysX dynamics.
- GPU rendering and simulation.
- A bundled Pink integration.

Our launcher imports the Unitree G1 URDF into a correctly scaled Isaac USD asset
and caches it under `.cache/isaac_g1`.

### URDF

Purpose: describe the robot's kinematic structure.

URDF defines robot links, joints, axes, limits, geometry, and inertial data. The
G1 URDF currently used is:

```text
../unitree_rl_gym/resources/robots/g1_description/g1_29dof.urdf
```

### USD

Purpose: represent the visual and physical scene in Isaac Sim.

The G1 URDF is converted into USD on the first launch. Later launches reuse the
cached USD rather than importing the robot again.

### Pinocchio

Purpose: calculate robot kinematics efficiently.

Pinocchio loads the G1 URDF and calculates link transforms, Jacobians, and joint
limits used by IK.

### Pink

Purpose: solve constrained inverse kinematics.

Pink receives targets for G1 body frames and calculates a valid 29-joint robot
configuration. The current target frames are:

```text
Human T8       -> G1 torso
Human left hand  -> G1 left wrist
Human right hand -> G1 right wrist
Human left foot  -> G1 left ankle
Human right foot -> G1 right ankle
Human pelvis movement -> G1 root movement
```

Pink also applies posture regularization and the joint limits from the G1 URDF.

### MuJoCo

Purpose: another physics simulator widely used for robot control research.

Several sibling projects use MuJoCo. It is the current dynamic test environment
for the SONIC controller. Isaac Sim remains useful for kinematic visualization
and may also support later dynamic experiments.

### GR00T SONIC

Purpose: dynamically track whole-body G1 motion while maintaining balance.

IK produces target joint trajectories, but IK alone does not create motor
torques or keep a humanoid balanced. SONIC is intended to provide that tracking
and balancing layer.

The SONIC source repository exists at:

```text
../GR00T-WholeBodyControl
```

Its MuJoCo environment, released policy assets, TensorRT runtime, and compiled
controller are installed locally. The first 28.62-second pouring-water reference
completed through SONIC, with one MuJoCo fall/reset. SONIC is still separate
from the Isaac playback command and remains simulation-only.

### Unitree SDK2

Purpose: communicate with Unitree simulators and eventually the physical robot.

The local Python SDK is in:

```text
../unitree_sdk2_python
```

SDK transport is not used by the current kinematic viewer. Hardware commands
must not be added until simulation and safety tests are complete.

## 4. Project Directories

The important directories are:

```text
~/Projects/
|
|-- Humonoid_retargeting_082026/   This control and retargeting project
|   |-- START_GUIDE.md             This guide
|   |-- README.md                  Short project reference
|   |-- motionctl.py               Inspect and validate MVNX files
|   |-- retargetctl.py             Run/check older experiment stacks
|   |-- run_isaac_g1_playback.sh   Start G1 retargeting in Isaac Sim
|   |-- retargeting/
|   |   |-- mvnx.py                Streaming MVNX parser
|   |   `-- isaac_g1_playback.py   IK and Isaac visualization
|   |-- xsens_TJ_08072026/         Recorded Xsens data
|   |-- Documentation/             Supplied project documents
|   `-- docs/architecture.md       Technical target architecture
|
|-- xsens_mvn_ros2/                Live Xsens ROS 2 driver
|-- unitree_rl_gym/                G1 model and locomotion policy experiments
|-- unitree_mujoco/                Unitree-compatible MuJoCo simulation
|-- unitree_sdk2_python/           Unitree DDS/SDK Python library
|-- xr_teleoperate/                Arm IK and hand-retargeting references
`-- GR00T-WholeBodyControl/        SONIC controller source
```

The sibling projects remain separate because they use different Python, ROS,
CUDA, and simulator environments.

## 5. Recorded Input Data

The recordings are under:

```text
xsens_TJ_08072026/
|-- MVNX export/           Preferred input
|-- BVH export/            Compatibility and animation format
|-- XLSX export/           Spreadsheet analysis format
`-- MVN files for viewing/ Native Xsens project files
```

There are twelve recorded motions, including:

- Bending forward.
- Lifting boxes.
- Pouring water.
- Picking and placing small objects.
- Standing up from prone and supine positions.

The dataset contains approximately 589 seconds and 141,386 frames in total.

## 6. Current Offline Dataflow

The exact current dataflow is:

```text
MVNX file
  |
  | retargeting/mvnx.py
  | - reads metadata
  | - validates frame indexes and dimensions
  | - reads global segment positions and wxyz orientations
  v
Canonical Xsens frames at 240 Hz
  |
  | isaac_g1_playback.py
  | - chooses samples corresponding to 50 Hz
  | - treats frame 0 as calibration
  | - expresses torso, hands, and feet relative to the pelvis
  | - scales human displacement for the smaller G1 body
  v
Pink IK targets
  |
  | - torso task
  | - left/right wrist tasks
  | - left/right ankle tasks
  | - stable posture objective
  | - G1 joint limits
  v
29 G1 joint positions
  |
  | - maximum joint speed limit
  | - map joints by name into the Isaac articulation
  v
Isaac Sim G1 visualization at 50 Hz
```

The current body-displacement scale defaults to `0.75`. This is a provisional
global value. Future work should calculate separate scales for the torso, arms,
and legs.

## 7. Run the Current Demo

Open a terminal and enter:

```bash
cd ~/Projects/Humonoid_retargeting_082026
./run_isaac_g1_playback.sh --loop
```

What happens:

1. Isaac Sim starts. Startup may take several seconds.
2. The G1 asset is loaded at the world origin.
3. The original Xsens skeleton appears beside the G1.
4. `Bending forward.mvnx` is read and retargeted.
5. Both figures repeat the synchronized motion until you close Isaac Sim.

The human skeleton uses cyan for the left side, orange for the right side, and
green feet for active Xsens contacts. Larger points mark the pelvis, torso,
hands, and feet used as retargeting sources. Use `--hide-xsens` to hide the
human or `--xsens-offset-y 2.0` to increase the space between the figures.

Around the G1, diagnostic markers explain what inverse kinematics is doing:

- Red point: desired G1 torso, wrist, or ankle location.
- Green point: location the solved G1 pose actually reaches.
- Yellow line: distance between desired and achieved locations.

Short yellow lines mean good positional tracking. A long line means the target
may be unreachable, incorrectly scaled, constrained by another task, or limited
by joint speed. Use `--hide-diagnostics` when only the motion is needed.

The terminal should print:

```text
Playback mode: KINEMATIC (zero gravity; pose and velocities reset at 50 Hz)
```

To play once and retain the final pose:

```bash
./run_isaac_g1_playback.sh
```

To choose another recording:

```bash
./run_isaac_g1_playback.sh \
  --mvnx "xsens_TJ_08072026/MVNX export/Lifting boxes.mvnx" \
  --loop
```

To display only one frame:

```bash
./run_isaac_g1_playback.sh --max-frames 1
```

To run a fast test without opening a window:

```bash
./run_isaac_g1_playback.sh --headless --max-frames 50 --no-realtime
```

To save detailed diagnostics for later comparison:

```bash
./run_isaac_g1_playback.sh --headless --no-realtime \
  --diagnostics-json outputs/bending-forward.json \
  --diagnostics-csv outputs/bending-forward.csv
```

The terminal and JSON summary report, for each torso/wrist/ankle target:

- RMS position error in centimeters: typical distance over the recording.
- Maximum position error in centimeters: worst observed frame.
- RMS and maximum orientation error in degrees.
- Frames where the `4 rad/s` joint-speed limiter changed the IK result.
- Frames and joint names within `0.05 rad` of a URDF joint limit.

The CSV contains one row per 50 Hz output frame. Generated files under
`outputs/` remain local and are not committed to Git.

### Export All Recordings

Process all twelve MVNX recordings without opening a window:

```bash
python3 batch_export.py
```

This can take a long time because every 50 Hz output frame requires several IK
solutions. Progress is printed as `[current/total]`. One failed recording does
not discard reports already produced for earlier recordings.

The default output structure is:

```text
outputs/xsens-g1-batch/
|-- manifest.json
|-- summary.csv
|-- bending-forward/
|   |-- diagnostics.json
|   |-- diagnostics.csv
|   `-- sonic_reference/
|       |-- joint_pos.csv
|       |-- joint_vel.csv
|       |-- body_pos.csv
|       |-- body_quat.csv
|       |-- metadata.txt
|       |-- trajectory_metadata.json
|       `-- info.txt
`-- one equivalent directory for every other recording
```

`summary.csv` is the first file to inspect. It provides one row per recording
with frame count, duration, torso/wrist/ankle errors, speed-limited frames, and
near-limit frames.

The `sonic_reference` files follow SONIC's documented 50 Hz joint-reference
layout. Joint values are mapped by name into its 29-joint IsaacLab order rather
than assuming the URDF, Pinocchio, Isaac, and SONIC array indexes match. Root
position is XYZ and root orientation is WXYZ.

These files are reference trajectories, not motor commands. Producing them does
not prove that SONIC can track the movement under gravity.

To review every clip visually instead of running headlessly:

```bash
python3 batch_export.py --visible
```

Isaac pauses after each clip. Close its window to launch the next recording.
Use `--dry-run` to inspect all commands without launching anything, or
`--max-frames 50` for a short pipeline test of every recording.

Do not use `--dynamic` expecting stable movement. It enables gravity without a
balance controller and demonstrates why IK alone is insufficient.

## 8. Inspect Xsens Data

Display recording metadata:

```bash
python3 motionctl.py inspect \
  "xsens_TJ_08072026/MVNX export/Bending forward.mvnx"
```

Display one frame as JSON:

```bash
python3 motionctl.py sample \
  "xsens_TJ_08072026/MVNX export/Bending forward.mvnx" \
  --frame 100
```

Validate every MVNX recording:

```bash
python3 motionctl.py validate "xsens_TJ_08072026/MVNX export"
```

These commands do not launch Isaac Sim, ROS, or a robot controller.

## 9. Future Live Dataflow

Recorded and live input should feed the same canonical representation:

```text
Person wearing Xsens suit
        |
        v
Xsens MVN / Humanoid Network Streamer
        |
        v
xsens_mvn_ros2 on ROS 2 Jazzy
        |
        v
ROS /link_states with live human segment poses
        |
        v
Live adapter
  - reorder ROS xyzw quaternions to wxyz
  - neutral-pose calibration
  - reject stale, duplicate, or invalid frames
  - convert to the same canonical Xsens frame used by MVNX
        |
        v
The same human-to-G1 retargeter
        |
        v
29 G1 joint targets at 50 Hz
        |
        v
Safety filter and whole-body tracking controller
        |
        +----------> MuJoCo or Isaac dynamic simulation
        |
        `----------> Real G1 only after simulation acceptance
```

Using one canonical input contract prevents the offline and live retargeters
from behaving differently.

## 10. Planned SONIC Dataflow

SONIC can accept a joint-based streamed motion reference. The initial planned
contract is ZMQ Protocol v1:

```text
joint_pos   [N, 29]  radians, IsaacLab joint order
joint_vel   [N, 29]  radians per second, IsaacLab joint order
body_quat   [N, 4]   root orientation in wxyz order
frame_index [N]      monotonically increasing frame numbers
```

The target rate is 50 Hz.

The current SONIC wire format places a `pose` topic prefix before a 1280-byte
JSON header and packed binary fields. The project publisher creates this exact
message from an exported `sonic_reference` directory. This ignored output must
first be generated from the approved local MVNX dataset:

```bash
.conda-sonic-stream/bin/python -m retargeting.sonic_stream \
  outputs/xsens-g1-batch/pouring-water/sonic_reference --dry-run
```

Create that small, isolated publisher environment once with the installed
Miniforge:

```bash
~/miniforge3/bin/conda create -p .conda-sonic-stream python=3.12 pyzmq -y
```

SONIC's `--input-type zmq` mode is used because it tracks streamed G1 references
directly. The `zmq_manager` mode adds planner and mode-switching behavior that is
not needed for the first offline dynamic test.

The intended dynamic path is:

```text
Retargeted G1 reference trajectory
        |
        v
Safety and continuity checks
        |
        v
ZMQ Protocol v1 publisher
        |
        v
SONIC whole-body tracking policy
        |
        v
G1 simulation with gravity and contacts
```

SONIC does not replace retargeting. Retargeting produces robot-compatible
reference motion; SONIC attempts to track that motion dynamically.

The deployed encoder, decoder, and planner are NVIDIA's externally pretrained
Unitree G1 models from `https://huggingface.co/nvidia/GEAR-SONIC`. They were not
trained on this project's Xsens recordings. The recordings are converted to
runtime G1 references; SONIC modifies the requested movement as needed for
dynamic balance.

The recommended recorded-playback interface is:

```bash
cd ~/Projects/Humonoid_retargeting_082026
.conda-sonic-stream/bin/python -m retargeting.sonic_gui
```

Click `START MUJOCO + SONIC`, wait for `STACK ONLINE` and `Init Done`, select a
recording, and click `PLAY SELECTED + START CONTROLLER`. The GUI handles stream
arming, the nominal standing prefix, controller activation, and graphical
MuJoCo support release. Isaac playback remains a separate kinematic comparison
and does not use the SONIC controller.

For live suit input, first install Pink in the `tv` Conda environment, then use
the GUI's `START LIVE SUIT PIPELINE`, `CALIBRATE LIVE NEUTRAL`, and `ENABLE LIVE
XSENS` controls. Live mode remains simulation-only until suit/network failure
injection and dynamic telemetry acceptance criteria pass.

## 11. Safety Layers

Robotics safety must be designed as layers, not as one emergency-stop button.

### Offline Safety

- Validate all numbers are finite.
- Verify frame continuity and timestamps.
- Enforce G1 joint limits.
- Limit joint speed and acceleration.
- Detect abrupt pose jumps.
- Inspect motions visually before dynamic simulation.

### Live-Stream Safety

- Reject stale Xsens data.
- Stop or enter a controller-defined safe state when input disappears.
- Require explicit calibration and enable actions.
- Blend smoothly when starting or resuming.
- Never snap from a distant paused pose.

### Dynamic-Simulation Safety

- Measure falls and foot slip.
- Monitor contact state and tracking error.
- Check torque feasibility.
- Test frozen, delayed, reordered, and dropped input frames.

### Real-Robot Safety

- Real hardware must require an explicit profile.
- A trained operator must control the emergency stop.
- Only one controller may own the robot command interface.
- Do not bypass CRC, joint limits, policy checks, or Unitree safety modes.
- Begin with restricted upper-body range and low speed.
- Keep people and obstacles outside the test area.

This repository intentionally does not make real-robot execution the default.

## 12. What to Learn First

You do not need to learn every tool before using this project. A practical
learning order is:

1. Run one recorded clip in Isaac Sim.
2. Inspect the same clip with `motionctl.py`.
3. Understand position, orientation, links, and joints.
4. Learn the difference between forward and inverse kinematics.
5. Compare the human Xsens skeleton with the G1 result.
6. Learn why dynamic balance needs a controller rather than IK alone.
7. Learn basic ROS topics before starting the live adapter.
8. Learn Unitree SDK and hardware safety only after simulation is reliable.

## 13. Recommended Next Development Step

The synchronized Xsens/G1 viewer, IK diagnostics, batch export, and first SONIC
dynamic playback are complete. The next useful feature is dynamic telemetry
containing:

- Root height and roll/pitch.
- Fall time and reset count.
- Desired-versus-observed G1 joint tracking error.
- Foot contact and slip.
- Torque and action saturation.
- A machine-readable summary for every recording.

This is more valuable than immediately connecting a live suit because the first
upright SONIC test already fell once. We need to distinguish reference problems
from controller limits before adding network timing and live calibration.

After that, the sequence should be:

1. Improve body scaling and contact-aware retargeting.
2. Export validated 29-joint trajectories at 50 Hz.
3. Add dynamic tracking telemetry and define simulation acceptance criteria.
4. Improve the references until restricted upright clips track without falls.
5. Add the live ROS 2 Xsens adapter described in `docs/live_xsens.md`.
6. Add live watchdog and operator controls.
7. Validate delayed, frozen, dropped, and discontinuous live input.
8. Consider restricted real-robot tests only after simulation acceptance.

## 14. Common Questions

### Why does the robot not have exactly the same pose as the person?

The human and G1 have different dimensions and joints. Some human poses are
unreachable or unsafe for the robot. Retargeting seeks a useful approximation.

### Why disable gravity in the current viewer?

The viewer validates kinematics. Pink calculates poses but does not calculate
the motor torques required for balance. Enabling gravity without a balancing
controller causes the G1 to fall.

### Does successful kinematic playback mean the real robot can do it?

No. It means the joint mapping is plausible. Dynamic feasibility and hardware
safety remain separate validation stages.

### Why use 50 Hz when Xsens records at 240 Hz?

The SONIC reference and control loop use 50 Hz. Resampling early makes offline
output match the controller contract.

### Why use joint names instead of array indexes?

Pinocchio, Isaac Sim, MuJoCo, SONIC, and Unitree may order the same joints
differently. Mapping by name prevents one joint value from being accidentally
sent to another joint.

### Can we use the live Xsens suit now?

The ROS 2 source works, but the live adapter into the new whole-body retargeter
has not been implemented. Recorded MVNX lets us improve and validate that
retargeter first.

### Can we run this on the real G1 now?

No. Kinematic visualization is not sufficient evidence for safe hardware use.
We first need dynamic tracking, failure tests, watchdog behavior, explicit
operator controls, and simulation acceptance criteria.

## 15. Further Reading in This Workspace

- `README.md`: concise commands and current project status.
- `docs/architecture.md`: technical architecture and interface contracts.
- `docs/collaboration.md`: clone layout, local data, and contribution workflow.
- `docs/sonic_simulation.md`: reproducible SONIC setup and recorded playback.
- `docs/live_xsens.md`: live MVN/ROS contract and planned safe adapter workflow.
- `AGENTS.md`: project development and safety rules.
- `Documentation/navod_Xsens_ROS2_Unitree_G1_retargeting.pdf`: supplied Czech
  technical guide.
- `../GR00T-WholeBodyControl/docs/source/tutorials/zmq.md`: SONIC streaming
  protocol.
- `../GR00T-WholeBodyControl/docs/source/references/motion_reference.md`: SONIC
  motion-reference files.
- `../GR00T-WholeBodyControl/docs/source/references/conventions.md`: SONIC axes,
  quaternion, and joint-order conventions.

When documentation and executable code disagree, treat the current code and
measured runtime behavior as authoritative, then update this guide.
