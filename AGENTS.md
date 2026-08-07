# Project Guidance

## Purpose

This repository is the control plane for Xsens-to-humanoid experiments. Keep the
large upstream projects as siblings under `~/Projects`; do not vendor or silently
modify them here.

The intended data path is:

```text
Xsens MVN -> xsens_mvn_ros2 -> retargeting adapter -> simulator/controller -> robot
```

The long-term target is Xsens whole-body input driving the GR00T SONIC G1
controller, validated in MuJoCo before any real-robot deployment.

## Safety

- Simulation is the default. Never make real-robot execution the default profile.
- Require an explicit profile and preserve fail-safe behavior for stale input.
- Do not bypass joint limits, policy safety checks, CRC checks on hardware, or
  emergency-stop procedures.
- Keep environment boundaries explicit. ROS 2 Jazzy, SONIC virtual environments,
  Isaac Sim, and the `tv` Conda environment are not interchangeable.

## Development

- Put orchestration definitions in `config/stacks.json`.
- Keep `retargetctl.py` dependency-free so checks and lifecycle commands work
  before project-specific environments are activated.
- Add adapters to a dedicated Python package when implementation begins; do not
  add new experiment scripts to sibling repositories.
- Use atomic files or ZMQ at incompatible ROS/Python environment boundaries.
- Add a prerequisite entry whenever a stack depends on a new repository, model,
  virtual environment, device, or setup file.
- Tests must not launch ROS, simulators, GUIs, or robot controllers.

## Known Sibling Projects

- `../xsens_mvn_ros2`: ROS 2 Jazzy Xsens source and earlier bridges.
- `../unitree_rl_gym`: pretrained G1 locomotion policy and direct MuJoCo runner.
- `../unitree_mujoco`: Unitree DDS-compatible MuJoCo simulation.
- `../unitree_sdk2_python`: Unitree Python DDS transport.
- `../xr_teleoperate`: reusable arm IK and hand retargeting references.
- `../GR00T-WholeBodyControl`: SONIC whole-body controller and MuJoCo loop.

## Current Reality

The existing Xsens demos convert body motion into locomotion velocity commands;
they are not whole-body imitation. Do not describe a stack as full-body
retargeting until an adapter emits SONIC-compatible motion references and it has
been validated in simulation.
