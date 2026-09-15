# Runtime And Dependency Map

All processes run on the local Ubuntu workstation unless explicitly noted. The
Xsens MVN Studio sender may run on another machine on the trusted LAN. No current
stack sends commands to a physical robot.

For exact stack-specific details, use:

```bash
python3 retargetctl.py list
python3 retargetctl.py describe live-xsens-sonic-sim
python3 retargetctl.py check live-xsens-sonic-sim
```

`config/stacks.json` is the machine-readable source of truth. This document
explains boundaries shared by multiple stacks.

## Environment Boundaries

| Runtime | Runs | Must not be mixed with |
| --- | --- | --- |
| Host Python 3 | `retargetctl.py`, `motionctl.py`, batch orchestration | Project-specific simulation packages |
| ROS 2 Jazzy | Xsens source and `live_capture_ros` | Isaac Sim and SONIC virtual environments |
| Conda `tv` | Pink, Pinocchio, NumPy, quadprog, live retargeting | ROS 2 package installation |
| `.conda-sonic-stream` | PyQt GUI and pyzmq recorded publisher | ROS 2 and SONIC simulator packages |
| SONIC `.venv_sim` | Grounded MuJoCo simulator | ROS 2 and Conda `tv` |
| Native SONIC runtime | Controller binary, TensorRT, ONNX Runtime, libzmq | Python environments |
| Isaac Sim Python | Recorded kinematic visualization/export | ROS and SONIC Python environments |

The narrow ROS-to-Pink boundary is an atomically replaced JSON file. The
Pink-to-SONIC boundary is loopback ZMQ. This separation is intentional.

## Main Simulation Dataflow

```text
MVN Studio on LAN
  -> UDP 9763
  -> xsens_mvn_ros2 [ROS 2 Jazzy]
  -> ROS /link_states
  -> live_capture_ros [ROS 2 Jazzy]
  -> /tmp/xsens-live-frame.json [atomic]
  -> live_retarget [Conda tv]
  -> tcp://127.0.0.1:5556 topic pose [SONIC Protocol v1]
  -> native SONIC controller
  -> Unitree DDS domain 0 on interface lo
  -> grounded MuJoCo [SONIC .venv_sim]
```

Mock simulation replaces the first three stages with `mock_live_stream`, which
replays MVNX into the same atomic frame file. Recorded playback publishes a
complete exported reference directly from the GUI or `sonic_stream`.

## Local IPC Ownership

| Boundary | Producer | Consumer |
| --- | --- | --- |
| `/tmp/xsens-live-frame.json` | ROS capture or mock replay | Live retargeter |
| `/tmp/xsens-live-control.json` | GUI | Live retargeter and gated mock replay |
| `/tmp/xsens-live-status.json` | Live retargeter | GUI |
| `tcp://127.0.0.1:5556` | One of GUI, stream CLI, playlist, or live retargeter | SONIC controller |
| Unitree DDS domain 0, `lo` | SONIC and simulator | SONIC and simulator |
| tmux sessions | `retargetctl` or GUI backend | Operator and GUI status parser |

Only one process may bind ZMQ port `5556`. Recorded and live publishers are
mutually exclusive. The GUI enforces this for its own workflows; standalone
commands rely on operator discipline.

## External Repositories

Sibling projects stay under `RETARGET_PROJECTS_ROOT` (default: the parent of
this repository). Important active dependencies are:

- `xsens_mvn_ros2`: ROS 2 Xsens source and freshness-aware message fields.
- `unitree_rl_gym`: G1 URDF/assets and legacy locomotion baseline.
- `GR00T-WholeBodyControl`: SONIC controller, model assets, and simulator.
- `sonic-build-deps`: isolated tmux, xdotool, and C++ libraries.

`unitree_mujoco`, `unitree_sdk2_python`, and `xr_teleoperate` are references or
indirect dependencies, not current top-level stack components.

## Known Structural Debt

- Recorded and live Pink retargeters are separate implementations and use
  different solvers.
- Startup ordering uses fixed delays rather than readiness probes.
- SONIC state is inferred from terminal output instead of structured health IPC.
