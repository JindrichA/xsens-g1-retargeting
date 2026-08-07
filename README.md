# Humanoid Retargeting Runner

Tools for turning recorded or live Xsens whole-body motion into dynamically
trackable Unitree G1 references.

New to robotics or these tools? Start with [`START_GUIDE.md`](START_GUIDE.md).
For cloning, data placement, and contribution rules, see
[`docs/collaboration.md`](docs/collaboration.md).

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

See `docs/architecture.md` for the measured input contract and target SONIC
Protocol v1 output.

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
is expected to fall; it exists only to demonstrate the difference until SONIC
is integrated.

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

The SONIC source, reference motions, and configuration are present, but its
simulation virtual environment, compiled deployment binary, policy model, and
planner model are not currently present in that checkout. The checker reports
these separately rather than treating source checkout as a runnable install.

## Available Stacks

| Stack | Source | Target | Status |
| --- | --- | --- | --- |
| `xsens-g1-mujoco` | Xsens ROS `/link_states` | Unitree RL G1 in MuJoCo | Existing hand-to-velocity demo |
| `xsens-h1-isaac` | Xsens ROS `/link_states` | Isaac Sim H1 policy | Existing locomotion-intent demo |
| `sonic-g1-mujoco` | SONIC default input | SONIC G1 in MuJoCo | Whole-body controller baseline, no Xsens |

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
  whole-body tracking. Its `zmq_manager` motion-reference input is the likely
  adapter boundary.
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

1. Record and inspect normalized Xsens segment poses from `/link_states`.
2. Implement neutral calibration, coordinate conversion, stale-input handling,
   smoothing, and G1 limits in this repository.
3. Emit the motion-reference format consumed by SONIC's ZMQ manager input.
4. Validate tracking and failure behavior with `sonic-g1-mujoco`.
5. Add an explicit Xsens-to-SONIC simulation stack.
6. Consider hardware only after simulation acceptance criteria and emergency-stop
   behavior are documented and tested.

The main protocol references are in the sibling SONIC repository:

- `docs/source/tutorials/manager.md`
- `docs/source/tutorials/zmq.md`
- `docs/source/references/motion_reference.md`
- `docs/source/references/conventions.md`
