# Collaboration Setup

This repository contains the retargeting code, orchestration configuration,
tests, and documentation. Large recordings, generated simulator assets, policy
models, and sibling upstream projects are deliberately kept outside Git.

## Repository Contents

```text
.
|-- config/                     Experiment stack definitions
|-- docs/                       Architecture, setup, and operator documentation
|-- Documentation/              Supplied lightweight PDF references
|-- retargeting/                MVNX parser and G1 retargeting implementation
|-- tests/                      Unit tests that need no simulator or ROS
|-- batch_export.py             All-recording diagnostics and trajectory export
|-- motionctl.py                Recorded-motion inspection CLI
|-- retargetctl.py              Stack checks and process orchestration
|-- run_isaac_g1_playback.sh    Isaac Sim launcher
|-- README.md                   Project overview and command reference
`-- START_GUIDE.md              Beginner guide
```

The MVNX tools and tests use the Python standard library. Isaac-specific
dependencies come from Isaac Sim's bundled Python environment. The SONIC
publisher uses `requirements-sonic-stream.txt`; the GUI uses
`requirements-sonic-gui.txt`. Both remain isolated from ROS, Isaac, and SONIC.
See [`runtime.md`](runtime.md) for the complete environment and IPC map.

## Clone Layout

The default layout expects related upstream repositories to be siblings:

```text
~/Projects/
|-- Humonoid_retargeting_082026/
|-- xsens_mvn_ros2/
|-- unitree_rl_gym/
|-- unitree_mujoco/
|-- unitree_sdk2_python/
|-- xr_teleoperate/
|-- GR00T-WholeBodyControl/
|-- TensorRT-10.13.3.9/
|-- onnxruntime-linux-x64-1.16.3/
`-- sonic-build-deps/
```

If the sibling projects are elsewhere, set:

```bash
export RETARGET_PROJECTS_ROOT=/path/to/projects
```

The Isaac launcher defaults to `$HOME/isaacsim/python.sh`. Override it when
Isaac Sim is installed elsewhere:

```bash
export ISAACSIM_PYTHON=/path/to/isaacsim/python.sh
```

The G1 URDF defaults to the `unitree_rl_gym` sibling. It can be selected per
command:

```bash
./run_isaac_g1_playback.sh \
  --g1-urdf /path/to/g1_29dof.urdf \
  --mvnx /path/to/motion.mvnx
```

It can also be configured for a terminal session:

```bash
export RETARGET_G1_URDF=/path/to/g1_29dof.urdf
```

## Recording Data

The local `xsens_TJ_08072026/` and `GS_data/` directories are ignored by Git.
They contain large motion-capture recordings. Do not force-add these files to
the source repository. See [`data.md`](data.md) for dataset and artifact rules.

Motion capture can be identifiable research data. Confirm participant consent,
organizational policy, and the allowed recipients before transferring it. Use
an approved data store, shared drive, or object store rather than ordinary Git.

To preserve the existing example commands, place the approved dataset at:

```text
Humonoid_retargeting_082026/
`-- xsens_TJ_08072026/
    `-- MVNX export/
        `-- Bending forward.mvnx
```

Recordings can also live anywhere outside the repository and be passed
explicitly:

```bash
python3 motionctl.py inspect /approved/data/motion.mvnx

./run_isaac_g1_playback.sh \
  --mvnx /approved/data/motion.mvnx \
  --loop
```

## Development Checks

Run all tests without launching ROS, Isaac Sim, MuJoCo, or robot controllers:

```bash
python3 -m unittest discover -s tests -v
```

Check local prerequisites for the configured experiment stacks:

```bash
python3 retargetctl.py list
python3 retargetctl.py describe sonic-g1-mujoco
python3 retargetctl.py check
```

Validate an approved recording before using it:

```bash
python3 motionctl.py validate /path/to/recordings
```

See [`sonic_simulation.md`](sonic_simulation.md) for the external model/runtime
setup and [`live_xsens.md`](live_xsens.md) for the live ROS contract.

## Contribution Rules

- Keep simulation as the default; never make hardware execution automatic.
- Put stack orchestration in `config/stacks.json`.
- Keep `retargetctl.py` dependency-free.
- Add implementation to the `retargeting` package rather than sibling projects.
- Do not commit recordings, generated USD caches, models, virtual environments,
  logs, credentials, or machine-specific configuration.
- Add a prerequisite check when a feature needs a new repository, model,
  environment, device, or setup file.
- Keep unit tests independent of ROS, simulators, GUIs, and controllers.

## Publishing

Before pushing to the established private remote:

1. Review `git status`, `git diff`, and recent history.
2. Run the dependency-free unit test suite.
3. Verify the ignored recordings, generated outputs, environments, models, and
   external dependencies are not staged.
4. Confirm any newly added documents or data are approved for redistribution.
5. Commit only the intended source and documentation changes.
6. Push without bypassing hooks or force-updating shared history.
