# G1 Dynamic Controller Selection

## Decision

Use NVIDIA GR00T Whole-Body Control / GEAR-SONIC for the first dynamic G1 test.
Keep GMR as the leading alternative retargeter and TWIST2 as the leading
alternative complete tracking stack.

This decision is based on the public projects as checked on 2026-08-10. It is a
simulation decision, not approval for real-robot deployment.

## Why SONIC First

- The local sibling checkout is current and already contains the MuJoCo loop,
  C++ deployment stack, and protocol documentation.
- Public Unitree G1 encoder/decoder checkpoints and configuration are available
  from `nvidia/GEAR-SONIC` on Hugging Face.
- Protocol v1 directly accepts the project's existing 29-joint positions and
  velocities in IsaacLab order at 50 Hz.
- The streaming-only `--input-type zmq` path does not need SMPL conversion,
  latent-token generation, VR hardware, or planner mode switching.
- SONIC includes motions beyond standing/walking, including kneeling, crawling,
  getting up, and whole-body teleoperation.
- Source is Apache-2.0; model weights use the NVIDIA Open Model License.

The current packed wire format is one ZMQ message containing a topic prefix, a
1280-byte null-padded JSON header, and concatenated binary fields. Protocol v1
itself remains supported even though SONIC also supports protocols v3 and v4.

## Alternatives

| Project | Strength | Cost for this repository | Decision |
| --- | --- | --- | --- |
| GMR | MIT; Unitree G1; offline Xsens BVH and live Xsens MVN UDP; tuned for tracking policies | Replaces the current Pink retargeter but does not provide dynamic control by itself | Evaluate later against Pink, especially for difficult floor clips |
| TWIST | MIT; public G1 general-motion checkpoint; sim2sim and sim2real | Python 3.8, Isaac Gym, Redis, and conversion to its motion-file/server contract | Viable fallback |
| TWIST2 | MIT; public G1 ONNX checkpoint; explicit cable-connected simulation and hardware workflow; uses GMR | Separate Python 3.8 and Python 3.10 environments, Isaac Gym, Redis, and a less mature setup | Best complete fallback if SONIC cannot track these references |
| BeyondMimic | Strong open motion-tracking training base | Deployment lives in another repository and checkpoint/input integration is less direct | Research/training option |
| ASAP | MIT; agile G1 tracking and sim2sim tooling | Primarily a training/adaptation workflow, not the shortest released-checkpoint path | Research fallback |
| Human2Humanoid | Established humanoid imitation work | CC BY-NC license and older stack make it less suitable here | Not selected |

## Required External Setup

The SONIC checkout is a sibling project. Its documented simulation setup is:

```bash
cd ~/Projects/GR00T-WholeBodyControl
bash install_scripts/install_mujoco_sim.sh
python download_from_hf.py
```

The local setup now includes the release encoder, decoder, observation
configuration, planner, MuJoCo environment, TensorRT 10.13.3, ONNX Runtime
1.16.3, and a compiled controller. Verify readiness from this repository with:

```bash
python3 retargetctl.py check sonic-g1-mujoco
```

The planner remains a controller argument even when `--input-type zmq` is
selected. The Xsens publisher itself does not use planner commands.

The upstream CMake configuration only adds the msgpack include directory when
ROS 2 is enabled even though a non-ROS output header includes msgpack. The local
build works around this with a compiler include path and does not modify SONIC
source. Its main binary builds successfully; optional C++ test targets currently
fail because upstream CMake mixes fetched and Conda GoogleTest versions.

The publisher has one runtime dependency, `pyzmq`, isolated in this project's
`.conda-sonic-stream` environment. `requirements-sonic-stream.txt` supports a
standard Python virtual environment on other machines. It must not be installed
into the ROS, Isaac Sim, or SONIC environments.
