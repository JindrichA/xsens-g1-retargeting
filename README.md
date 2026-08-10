# Humanoid Retargeting Runner

Tools for turning recorded or live Xsens whole-body motion into dynamically
trackable Unitree G1 references.

New to robotics or these tools? Start with [`START_GUIDE.md`](START_GUIDE.md).
For cloning, data placement, and contribution rules, see
[`docs/collaboration.md`](docs/collaboration.md).
The evaluated dynamic-controller options and SONIC selection are recorded in
[`docs/controller_selection.md`](docs/controller_selection.md).
For reproducible dynamic playback, see
[`docs/sonic_simulation.md`](docs/sonic_simulation.md). For the measured live
ROS interface and planned adapter, see [`docs/live_xsens.md`](docs/live_xsens.md).

The immediate development path is recorded Xsens MVNX to G1 retargeting to
SONIC MuJoCo. Live ROS 2 input will then replace file playback without changing
the retargeting contract. Physical robot output is deliberately not enabled.

## Inspect Recorded Motion

The new recordings are canonical MVNX inputs. Inspect one without installing
any Python packages:

```bash
python3 motionctl.py inspect \
  "xsens_TJ_08072026/MVNX export/Stand up from floor prone position.mvnx"

python3 motionctl.py sample \
  "xsens_TJ_08072026/MVNX export/Bending forward.mvnx" --frame 100

python3 motionctl.py validate "xsens_TJ_08072026/MVNX export"
```

`motionctl.py` streams the XML, so even the 350 MB recordings do not need to be
loaded into memory. It validates frame continuity, dimensions, finite values,
quaternion norms, contacts, and known source timestamp irregularities.

See `docs/architecture.md` for the measured input contract and SONIC Protocol
v1 output.

## Isaac Sim G1 Playback

The first retargeting milestone maps recorded Xsens pelvis-relative torso,
wrists, and feet onto the 29-DOF G1 with constrained Pink IK, then displays the
result on the installed 43-DOF G1 asset in Isaac Sim:

```bash
./run_isaac_g1_playback.sh --mvnx \
  "xsens_TJ_08072026/MVNX export/Bending forward.mvnx"
```

The default clip is `Bending forward.mvnx`, so this is also sufficient:

```bash
./run_isaac_g1_playback.sh
```

After playback, Isaac Sim keeps the final G1 pose visible until you close the
window. Add `--loop` to repeat the clip continuously. The synchronized Xsens
skeleton appears beside the G1: left limbs are cyan, right limbs are orange,
active foot contacts are green, and larger points identify retargeting sources.
Use `--hide-xsens` to display only the G1 or `--xsens-offset-y` to change the
separation.

The G1 also shows IK diagnostics: red points are desired torso/wrist/ankle
targets, green points are the achieved G1 frames, and yellow lines show position
error. Use `--hide-diagnostics` to hide them. Playback prints RMS and maximum
position/orientation error plus joint-speed and joint-limit warnings.

Write machine-readable reports for comparing retargeting changes:

```bash
./run_isaac_g1_playback.sh --headless --no-realtime \
  --diagnostics-json outputs/bending-forward.json \
  --diagnostics-csv outputs/bending-forward.csv
```

Reports contain errors after joint-speed limiting, so they describe the exact
configuration sent to Isaac. The ignored `outputs/` directory is intended for
local generated reports.

Export one clip as a 50 Hz SONIC reference directory:

```bash
./run_isaac_g1_playback.sh --headless --no-realtime \
  --trajectory-dir outputs/bending-forward/sonic_reference
```

Export all local MVNX recordings with diagnostics and trajectories:

```bash
python3 batch_export.py
```

The batch output includes `manifest.json`, an aggregate `summary.csv`, per-clip
diagnostic JSON/CSV, and SONIC-format `joint_pos.csv`, `joint_vel.csv`, root
`body_pos.csv`, root `body_quat.csv`, and metadata. Joints are explicitly
reordered by name into SONIC's 29-DOF IsaacLab order. These are kinematic
references and still require SONIC dynamic validation before hardware use.
The first complete measured run and its main findings are recorded in
[`docs/baseline.md`](docs/baseline.md).

Preview every generated command without launching Isaac:

```bash
python3 batch_export.py --dry-run
```

For a finite headless check:

```bash
./run_isaac_g1_playback.sh --headless --max-frames 50 --no-realtime
```

This playback is kinematic retargeting for visual validation. It does not yet
prove that the G1 can dynamically track the motion without falling; that
requires the SONIC tracking-policy stage.

The playback defaults to a `4 rad/s` per-joint rate limit. Use
`--max-joint-speed` and `--scale` to tune conservative visualization behavior.
On first launch, the local G1 URDF is imported into `.cache/isaac_g1`; later
launches reuse that correctly scaled visual asset.

Kinematic playback is the safe default: gravity is zero, root and joint
velocities are reset each frame, and physics pauses on the final pose. The
explicit `--dynamic` option restores gravity without a balancing controller and
is expected to fall; it exists only to demonstrate the difference between IK
and controlled dynamic simulation.

## Existing Demos

`retargetctl.py` describes and checks the earlier velocity-command demos. These
are useful integration baselines but are not whole-body retargeting.

No installation is required. The runner uses tmux when available; otherwise it
supervises all components in the foreground and stops them together on `Ctrl-C`.
Use `--backend foreground` or `--backend tmux` to override automatic selection.

```bash
cd ~/Projects/Humonoid_retargeting_082026
python3 retargetctl.py list
python3 retargetctl.py check xsens-g1-mujoco
python3 retargetctl.py start xsens-g1-mujoco --attach
```

When using tmux, detach with `Ctrl-b d` and inspect or stop the stack later:

```bash
python3 retargetctl.py status xsens-g1-mujoco
python3 retargetctl.py logs xsens-g1-mujoco bridge
python3 retargetctl.py stop xsens-g1-mujoco
```

Preview exact commands without starting anything:

```bash
python3 retargetctl.py start sonic-g1-mujoco --dry-run
```

If sibling repositories are stored elsewhere, set `RETARGET_PROJECTS_ROOT`.

The local SONIC simulation environment, release policy, planner, TensorRT
10.13.3, ONNX Runtime 1.16.3, and controller binary are installed. Build-only
C++ dependencies are isolated under `~/Projects/sonic-build-deps`; ROS 2 is not
enabled in the SONIC controller. The stack launches the verified binary directly
instead of rebuilding through upstream `deploy.sh` on every run.

Once `python3 retargetctl.py check sonic-g1-mujoco` reports ready, run the
gravity-enabled test in two terminals:

```bash
# One-time local publisher environment (separate from SONIC and ROS)
~/miniforge3/bin/conda create -p .conda-sonic-stream python=3.12 pyzmq -y

# Terminal 1: SONIC MuJoCo simulation and controller
PATH="$HOME/Projects/sonic-build-deps/bin:$PATH" \
  python3 retargetctl.py start sonic-g1-mujoco --backend tmux --attach

# Terminal 2: validate first, then publish the exported clip
# (outputs are generated locally from the ignored approved MVNX dataset)
.conda-sonic-stream/bin/python -m retargeting.sonic_stream \
  outputs/xsens-g1-batch/pouring-water/sonic_reference --dry-run
.conda-sonic-stream/bin/python -m retargeting.sonic_stream \
  outputs/xsens-g1-batch/pouring-water/sonic_reference
```

In the SONIC controller pane, press `]` to start control and `ENTER` to enable
ZMQ streaming before publishing. In the MuJoCo window, press `9` to release the
elastic support. Press `O` to stop. The publisher only binds a local ZMQ source;
it has no Unitree hardware output path. The first complete `Pouring water` test
was accepted end to end but fell once; see [`docs/baseline.md`](docs/baseline.md).

## Available Stacks

| Stack | Source | Target | Status |
| --- | --- | --- | --- |
| `xsens-g1-mujoco` | Xsens ROS `/link_states` | Unitree RL G1 in MuJoCo | Existing hand-to-velocity demo |
| `xsens-h1-isaac` | Xsens ROS `/link_states` | Isaac Sim H1 policy | Existing locomotion-intent demo |
| `sonic-g1-mujoco` | Protocol v1 G1 reference | SONIC G1 in MuJoCo | Dynamic whole-body simulation; publisher separate |

Stack definitions live in `config/stacks.json`. Each component gets its own
environment setup, which avoids mixing ROS 2 Jazzy, Conda, Isaac Sim, and SONIC
Python dependencies.

## What Was Found

- `xsens_mvn_ros2` is the working Xsens ROS 2 Jazzy source. It publishes
  `/link_states`, `/joint_states`, `/tf`, `/com`, and diagnostics.
- Its G1 demo writes hand-derived velocity commands to
  `/tmp/xsens_hand_cmd.json`; it is stable locomotion intent, not pose imitation.
- Its H1 Isaac Sim demo similarly drives a pretrained locomotion policy.
- `unitree_mujoco` is useful for controllers that already speak Unitree DDS.
- `xr_teleoperate` contains reusable arm IK, hand retargeting, limits, and filters.
- `GR00T-WholeBodyControl` SONIC is the best candidate for dynamically stable G1
  whole-body tracking. Its streaming-only `zmq` input accepts the current 29-DOF
  references directly; `zmq_manager` is only needed for planner mode switching.
- GMR now supports offline Xsens BVH and live Xsens MVN UDP for G1, making it the
  strongest alternative retargeter. TWIST and TWIST2 provide G1 checkpoints,
  but require separate Python 3.8, Isaac Gym, and Redis environments.
- `xsens_TJ_08072026` contains 12 synchronized recordings in MVNX, BVH, and MVN,
  plus 11 XLSX exports: 141,386 frames and about 589 seconds at 240 Hz.
- MVNX is X-forward/Y-left/Z-up with `wxyz` quaternions, matching SONIC. It also
  preserves contacts, CoM, and frame 0. BVH changes axes, omits those channels,
  and replaces frame 0 with a synthetic pose.
- GMR is recommended by the supplied technical guides and supports Unitree G1,
  but no local GMR checkout is currently present.
- No older `AGENTS.md` was present under `/home/humanoid` when this project was
  initialized, so this repository provides its own guidance.

## Integration Roadmap

1. Offline MVNX parsing, Pink IK, synchronized visualization, diagnostics, and
   SONIC Protocol v1 export are implemented.
2. The SONIC MuJoCo loop accepts and completes exported clips, but the first
   upright baseline fell once.
3. Add machine-readable dynamic telemetry and simulation acceptance criteria.
4. Improve calibration, body scaling, foot contacts, and reference feasibility.
5. Implement the ROS capture and live retargeting processes with a stale-input
   watchdog.
6. Validate restricted live input in simulation, including pause/resume and
   network-failure injection.
7. Consider hardware only after simulation acceptance criteria and emergency
   procedures are documented and passed.

The live adapter does not exist yet. Its complete measured input contract,
process split, operator sequence, and safety checklist are documented in
[`docs/live_xsens.md`](docs/live_xsens.md).

The main protocol references are in the sibling SONIC repository:

- `docs/source/tutorials/manager.md`
- `docs/source/tutorials/zmq.md`
- `docs/source/references/motion_reference.md`
- `docs/source/references/conventions.md`
