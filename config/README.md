# Stack Configuration

`stacks.json` is the authoritative inventory of runnable multi-process stacks.
It describes both how each component starts and where its runtime boundaries
are. Keep command definitions here rather than duplicating them in new scripts.

Inspect a stack without activating ROS, Conda, Isaac Sim, or SONIC:

```bash
python3 retargetctl.py list
python3 retargetctl.py describe sonic-g1-mujoco
python3 retargetctl.py check sonic-g1-mujoco
```

## Fields

- `profile`: safety scope. Current profiles are simulation-only; no hardware
  profile is defined. `retargetctl start` requires exactly `simulation` or
  `simulation-adapter` and rejects missing or unknown profiles.
- `maturity`: whether the stack is a legacy baseline, experimental, or awaiting
  validation.
- `dependencies`: sibling repositories, environments, or external runtimes.
- `requirements`: paths, executables, and required file contents checked before
  startup.
- `components`: processes launched in order.
- `runtime`: the environment in which one component executes.
- `role`: the component's single responsibility.
- `inputs` and `outputs`: ROS topics, files, sockets, DDS, or viewer controls
  crossing a process boundary.

The `live-xsens-sonic-sim` stack is only the live input adapter. It intentionally
does not launch `sonic-g1-mujoco`. The GUI coordinates both sessions.
