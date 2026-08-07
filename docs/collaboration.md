# Collaboration Setup

This repository contains the retargeting code, orchestration configuration,
tests, and documentation. Large recordings, generated simulator assets, policy
models, and sibling upstream projects are deliberately kept outside Git.

## Repository Contents

```text
.
|-- config/                     Experiment stack definitions
|-- docs/                       Architecture and collaboration documentation
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

The repository currently has no normal Python package dependencies. The MVNX
tools and tests use the Python standard library. Isaac-specific dependencies
come from Isaac Sim's bundled Python environment.

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
`-- GR00T-WholeBodyControl/
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

The local `xsens_TJ_08072026/` directory is approximately 5 GB and is ignored by
Git. It includes large MVN, MVNX, BVH, and XLSX exports. Do not force-add these
files to the source repository.

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
python3 retargetctl.py check
```

Validate an approved recording before using it:

```bash
python3 motionctl.py validate /path/to/recordings
```

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

Before pushing to a shared remote:

1. Choose the organization and repository visibility.
2. Choose a software license with the project owner.
3. Confirm the supplied PDFs may be redistributed.
4. Review `git status` and the complete initial commit.
5. Verify the ignored 5 GB dataset is not tracked.
6. Add the remote and push only after those decisions are recorded.
