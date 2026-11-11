# Xsens To Unitree G1 Retargeting

Simulation-first tools for converting recorded or live Xsens whole-body motion
into Unitree G1 references tracked by the GR00T SONIC controller.

The project currently supports recorded playback, a device-free mock live path,
and an experimental ROS 2 live adapter. Physical G1 support is limited to
read-only diagnostics, offline packet checks, and an explicit guarded Arm SDK
upper-body executor; physical whole-body SONIC control is not enabled.

## Start Here

- [Beginner start guide](START_GUIDE.md)
- [Documentation index](docs/index.md)
- [Current implementation status](docs/status.md)
- [Runtime and dependency map](docs/runtime.md)
- [Retargeting scale tuning](docs/retargeting_tuning.md)
- [SONIC simulation manual](docs/sonic_simulation.md)
- [Live Xsens manual](docs/live_xsens.md)
- [G1 hardware readiness](docs/g1_hardware_readiness.md)
- [G1 physical experiment log](docs/g1_physical_experiments.md)

## System At A Glance

```text
Recorded MVNX ---------------------> recorded reference publisher --+
                                                                    |
Mock MVNX -> atomic live frame -> Pink live retargeter -------------+-> SONIC -> MuJoCo
                                                                    |
MVN Studio -> ROS 2 -> atomic live frame -> Pink live retargeter ---+
```

The environments are deliberately separate:

| Part | Runtime |
| --- | --- |
| Repository control tools | Host Python 3, no third-party packages |
| Xsens source and capture | ROS 2 Jazzy |
| Pink live retargeting | Conda environment `tv` |
| GUI and recorded ZMQ publisher | `.conda-sonic-stream` |
| SONIC MuJoCo | Upstream `.venv_sim` |
| SONIC controller | Native binary with TensorRT and ONNX Runtime |
| Recorded IK/export | Isaac Sim Python |

Do not combine these environments. Atomic files and loopback ZMQ are the
intentional compatibility boundaries.

## Understand A Stack

`config/stacks.json` is the authoritative process inventory. Inspect it through
the dependency-free control tool:

```bash
python3 retargetctl.py list
python3 retargetctl.py describe sonic-g1-mujoco
python3 retargetctl.py describe live-xsens-sonic-sim
python3 retargetctl.py check sonic-g1-mujoco
```

The `sonic-g1-mujoco` stack runs the simulator and controller. The
`live-xsens-sonic-sim` stack runs only the suit source, ROS capture, and Pink
adapter; the two stacks are intentionally separate.

## Recommended Device-Free Workflow

The safest end-to-end development path uses a recorded MVNX file as mock suit
input while preserving live calibration, warmup, watchdog, and stop behavior.

```bash
.conda-sonic-stream/bin/python -m retargeting.sonic_gui
```

In the GUI:

1. Select an exported recording.
2. Click `START MOCK SIMULATION FROM SELECTED`.
3. Click `START MUJOCO + SONIC` and wait for initialization.
4. Calibrate the neutral pose.
5. Enable mock live input only after the GUI reports it is ready.
6. Keep the emergency-stop control available throughout the trial.

The `G1 Hardware` workspace performs local evidence checks and exposes a
subscriber-only `rt/lowstate` diagnostic. A separate CLI-only Arm SDK executor
supports explicit, finite upper-body commissioning in Regular Mode. Whole-body
`rt/lowcmd` and physical SONIC control remain blocked.

See [Getting Started](START_GUIDE.md) for setup and the exact recorded,
mock, and live distinctions.

## Main Commands

| Command | Purpose | Runtime |
| --- | --- | --- |
| `python3 motionctl.py ...` | Inspect and validate MVNX | Host Python |
| `python3 batch_export.py` | Export recordings through Isaac/Pink | Host orchestrator + Isaac |
| `python -m retargeting.compare_retargeting` | Compare named scale-profile manifests | Host Python |
| `python -m retargeting.scale_sweep` | Run and rank bounded multi-clip scale profiles | Host orchestrator + Isaac |
| `python3 retargetctl.py ...` | Describe, check, start, and stop stacks | Host Python |
| `./run_isaac_g1_playback.sh` | Kinematic G1 visualization/export | Isaac Sim Python |
| `python -m retargeting.sonic_gui` | Simulation operator console | `.conda-sonic-stream` |
| `python -m retargeting.sonic_stream` | Publish one recorded reference | `.conda-sonic-stream` |
| `python -m retargeting.live_capture_ros` | ROS pose to atomic frame | ROS 2 Jazzy |
| `python -m retargeting.live_retarget` | Atomic frame to SONIC reference | Conda `tv` |
| `python -m retargeting.mock_live_stream` | MVNX to atomic live frame | GUI/host-compatible Python |
| `python -m retargeting.g1_dds_diagnostics ...` | Read-only physical G1 diagnostics | `.venv-g1-readonly` |
| `python -m retargeting.g1_lowcmd_dry_run ...` | Build and validate G1 low-level packets without transport | `.venv-g1-readonly` |
| `python -m retargeting.g1_arm_sdk_playback ...` | Explicit guarded G1 upper-body playback | `.venv-g1-readonly` |

## Repository Layout

```text
config/          stack definitions and runtime metadata
docs/            maintained architecture, status, operations, and results
Documentation/   supplied historical PDF references
retargeting/     parsers, adapters, retargeting, SONIC, simulator, and GUI code
tests/           dependency-light unit tests; no simulator or ROS launches
outputs/         ignored generated artifacts
```

Sibling projects remain under `RETARGET_PROJECTS_ROOT`, which defaults to this
repository's parent directory. They must not be vendored or silently modified.

## Verification

```bash
python3 -m unittest discover -s tests
```

Current limitations and safety gaps are tracked in [Current Status](docs/status.md),
not in this landing page.
