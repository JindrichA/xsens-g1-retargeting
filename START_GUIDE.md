# Start Guide

This page provides the shortest safe path into the project. Use the
[documentation index](docs/index.md) when you need architecture, dependency, or
operator details.

## What The Project Does

Xsens describes a human skeleton. Unitree G1 has different proportions, joints,
and limits, so human joint angles cannot be copied directly. Pink inverse
kinematics converts selected human segment poses into a 29-joint G1 reference.
SONIC then attempts to track that reference while balancing the simulated robot.

```text
Xsens motion -> canonical body frame -> Pink IK -> SONIC reference -> MuJoCo G1
```

The real robot is not part of the current workflow. All operator procedures in
this repository are simulation-only.

## Current Choices

| Mode | Input | What it proves |
| --- | --- | --- |
| Recorded playback | Exported 50 Hz reference | SONIC transport and dynamic playback |
| Mock simulation | MVNX replay through the live adapter | Live software integration without a suit |
| Live suit | MVN Studio through ROS 2 | Experimental network and capture integration |
| Isaac playback | MVNX through kinematic Pink IK | Pose mapping and visual diagnostics only |

Mock simulation is the recommended development mode. Live suit behavior has not
yet passed network-failure and operator acceptance testing.

## 1. Inspect The Installation

From this repository:

```bash
python3 retargetctl.py list
python3 retargetctl.py describe sonic-g1-mujoco
python3 retargetctl.py check sonic-g1-mujoco
python3 -m unittest discover -s tests
```

`describe` explains which process runs in which environment and what crosses
each boundary. `check` verifies required files and executables.

## 2. Inspect A Recording

This command needs only host Python:

```bash
python3 motionctl.py inspect \
  "xsens_TJ_08072026/MVNX export/Bending forward.mvnx"
```

Raw recordings are ignored because motion capture may identify participants.
See [Data and Artifacts](docs/data.md) before adding or sharing datasets.

## 3. Run Device-Free Mock Simulation

Launch the GUI in its isolated environment:

```bash
.conda-sonic-stream/bin/python -m retargeting.sonic_gui
```

Then:

1. Select a recording with an available source MVNX.
2. Click `START MOCK SIMULATION FROM SELECTED`.
3. Click `START MUJOCO + SONIC`.
4. Wait for `Init Done` and fresh mock frames.
5. Click `CALIBRATE LIVE NEUTRAL` while frame zero is held.
6. Click `ENABLE LIVE XSENS` only when the status says calibration is ready.
7. Use `DISABLE LIVE XSENS` or emergency stop before changing the setup.

Mock mode writes the same atomic frame snapshots used by live ROS capture and
runs the same live Pink process. It does not open a robot interface.

## 4. Choose The Next Manual

- Use [SONIC Simulation](docs/sonic_simulation.md) for installation, recorded
  playback, controller keys, and MuJoCo operation.
- Use [Live Xsens](docs/live_xsens.md) only when MVN Studio and the suit are
  available.
- Use [Runtime and Dependencies](docs/runtime.md) when a process starts in the
  wrong environment or cannot find a sibling project.
- Use [Current Status](docs/status.md) before interpreting a successful demo as
  a safety or validation result.

## Safety Rules

- Keep DDS and ZMQ on loopback simulation interfaces.
- Never bypass calibration, stale-input handling, limits, or emergency stop.
- Do not run two publishers on ZMQ port `5556`.
- Do not infer dynamic safety from kinematic Isaac playback.
- Do not connect a physical G1; no hardware profile or acceptance gate exists.
