# SONIC MuJoCo Simulation Manual

## Scope

This manual reproduces the validated path:

```text
recorded MVNX -> Pink G1 reference -> Protocol v1 -> SONIC -> MuJoCo
```

It is simulation-only. None of these commands selects a physical robot network
interface. The first validated `Pouring water` run completed all 1,432 frames
but fell once, so this is an integration baseline rather than a safety claim.

## External Dependencies

Keep these outside this Git repository under `~/Projects`:

```text
~/Projects/
|-- GR00T-WholeBodyControl/
|-- TensorRT-10.13.3.9/
|-- onnxruntime-linux-x64-1.16.3/
`-- sonic-build-deps/
```

TensorRT is proprietary. Review and accept NVIDIA's TensorRT license yourself
before downloading it. SONIC requires TensorRT 10.13 on x86_64; do not silently
substitute a newer version because upstream documents incorrect inference with
version mismatches.

SONIC source is Apache-2.0. Released SONIC model weights use the NVIDIA Open
Model License.

## Install MuJoCo Environment

From the current SONIC sibling checkout:

```bash
cd ~/Projects/GR00T-WholeBodyControl
bash install_scripts/install_mujoco_sim.sh
```

This creates `.venv_sim` and installs MuJoCo plus the Unitree Python DDS bridge.

## Download SONIC Models

The deployment models come from NVIDIA's public
[`nvidia/GEAR-SONIC`](https://huggingface.co/nvidia/GEAR-SONIC) repository. They
are externally pretrained for Unitree G1; they were not trained or fine-tuned on
the Xsens recordings in this workspace. The local Pink retargeter converts each
recording into a 29-joint runtime reference, and the pretrained policy generates
balanced control actions from that reference and simulated robot observations.

Install the Hugging Face client in the SONIC simulation environment and run the
official public downloader:

```bash
cd ~/Projects/GR00T-WholeBodyControl
~/.local/bin/uv pip install --python .venv_sim/bin/python huggingface_hub
.venv_sim/bin/python download_from_hf.py
```

Expected files:

```text
gear_sonic_deploy/policy/release/model_encoder.onnx
gear_sonic_deploy/policy/release/model_decoder.onnx
gear_sonic_deploy/policy/release/observation_config.yaml
gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx
```

`model_encoder.onnx` encodes the G1 motion reference,
`model_decoder.onnx` produces the 29 controlled G1 body-joint actions, and
`planner_sonic.onnx` is the separately downloaded motion planner used by the
deployment binary. The GUI's recorded Protocol v1 path uses the encoder and
decoder for tracking; it does not train any model.

## Install TensorRT and ONNX Runtime

Download and extract the TensorRT 10.13.3 TAR package compatible with the
workstation from NVIDIA's TensorRT 10.x download page. The validated local path
is:

```text
~/Projects/TensorRT-10.13.3.9
```

Install the ONNX Runtime version expected by SONIC without changing system
packages:

```bash
curl -fL --retry 3 \
  -o /tmp/onnxruntime-linux-x64-1.16.3.tgz \
  https://github.com/microsoft/onnxruntime/releases/download/v1.16.3/onnxruntime-linux-x64-1.16.3.tgz

tar -xzf /tmp/onnxruntime-linux-x64-1.16.3.tgz -C ~/Projects
```

## Install Isolated C++ Build Tools

The upstream dependency installer requires `sudo` and probes system CUDA
packages. The validated setup keeps build libraries in an isolated Conda
prefix:

```bash
~/miniforge3/bin/conda create \
  -p ~/Projects/sonic-build-deps \
  -c conda-forge \
  clangxx just tmux xdotool yaml-cpp eigen msgpack-c msgpack-cxx \
  zeromq cppzmq nlohmann_json gtest \
  -y
```

## Build the Controller

Configure with explicit dependency paths. The `CMAKE_CXX_FLAGS` include is a
workaround for the current upstream CMake file, which includes `msgpack.hpp` in
a non-ROS output header but only adds the msgpack include directory when ROS 2
is enabled.

```bash
cd ~/Projects/GR00T-WholeBodyControl/gear_sonic_deploy

export TensorRT_ROOT="$HOME/Projects/TensorRT-10.13.3.9"
export onnxruntime_ROOT="$HOME/Projects/onnxruntime-linux-x64-1.16.3"
export CMAKE_PREFIX_PATH="$HOME/Projects/sonic-build-deps:$onnxruntime_ROOT"
export LD_LIBRARY_PATH="$TensorRT_ROOT/lib:$onnxruntime_ROOT/lib:$HOME/Projects/sonic-build-deps/lib:${LD_LIBRARY_PATH:-}"

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DMSGPACK_INCLUDE_DIR="$HOME/Projects/sonic-build-deps/include" \
  -DCMAKE_CXX_FLAGS="-I$HOME/Projects/sonic-build-deps/include"

cmake --build build --target g1_deploy_onnx_ref -j4
```

Build only the controller target. The current upstream optional test build can
mix fetched GoogleTest headers with Conda GoogleTest libraries and fail even
after the main controller binary succeeds.

Verify runtime linkage:

```bash
LD_LIBRARY_PATH="$TensorRT_ROOT/lib:$onnxruntime_ROOT/lib:$HOME/Projects/sonic-build-deps/lib:${LD_LIBRARY_PATH:-}" \
  ldd target/release/g1_deploy_onnx_ref
```

No dependency should say `not found`.

## Create the Protocol Publisher Environment

From this repository:

```bash
cd ~/Projects/Humonoid_retargeting_082026
~/miniforge3/bin/conda create \
  -p .conda-sonic-stream \
  python=3.12 pyzmq -y
```

`requirements-sonic-stream.txt` contains the equivalent dependency for a
standard Python virtual environment on another machine.

## Check and Launch

### PyQt Console

Install the GUI dependency in the existing publisher environment:

```bash
.conda-sonic-stream/bin/python -m pip install -r requirements-sonic-gui.txt
~/miniforge3/bin/conda install \
  -p "$PWD/.conda-sonic-stream" -c conda-forge xcb-util-cursor -y
```

Launch the simulation console from this repository:

```bash
.conda-sonic-stream/bin/python -m retargeting.sonic_gui
```

The console controls the `mujoco` and `sonic` tmux windows, while MuJoCo's
native viewer remains the 3D display. Use `START MUJOCO + SONIC`, wait for
`STACK ONLINE` and `Init Done`, select a recording, and click `PLAY SELECTED +
START CONTROLLER`. The GUI arms the recorded stream automatically; the separate
arm button is only needed for manual diagnostics. The GUI publishes the selected reference before
starting controller output, preventing SONIC's bundled default dance motion from
running during startup. It prepends a two-second nominal standing hold and keeps
MuJoCo's elastic support enabled for the first 1.5 seconds of controller output.
The GUI then sends exactly one `9` event to the graphical MuJoCo viewer through
`xdotool`; writing `9` to the tmux terminal does not reach MuJoCo's GLFW callback.
The robot remains visibly suspended after arming until `PLAY SELECTED + START
CONTROLLER` or `PLAY ALL 12 RECORDINGS` is selected; arming alone does not send a
trajectory or release support.
The nominal G1 stance has visibly bent hips, knees, and ankles; small leg motion
is expected from dynamic balance, but repeated stepping or falling is not a
successful standing baseline. `STOP QUEUE AFTER CURRENT` prevents the
next clip from being sent but cannot pause a trajectory already accepted by
SONIC. `EMERGENCY STOP CONTROLLER` sends SONIC's stop command immediately.

`VIEW SELECTED IN ISAAC SIM` starts the existing kinematic retargeting playback
for that recording in a separate `retarget-isaac-g1` tmux session. Isaac Sim is
an independent visual IK validator: it does not use SONIC, gravity, contacts, or
the MuJoCo controller path. `STOP ISAAC SIM` closes only that session. Running
Isaac and SONIC simultaneously increases GPU memory and compute usage, so stop
the simulator that is not being inspected when resources are constrained.

The GUI binds only to `tcp://127.0.0.1:5556` and does not provide a hardware
profile. Do not run the command-line playlist publisher at the same time because
only one publisher can bind that endpoint.

The GUI normalizes isolated tmux launches to `TERM=xterm-256color` and the
`sonic-build-deps` terminfo database. This is required when a host terminal such
as Ghostty exports a custom `TERM` entry unavailable in the isolated prefix.

### Command Line

```bash
cd ~/Projects/Humonoid_retargeting_082026
python3 retargetctl.py check sonic-g1-mujoco

PATH="$HOME/Projects/sonic-build-deps/bin:$PATH" \
  python3 retargetctl.py start sonic-g1-mujoco --backend tmux --attach
```

The first TensorRT launch generates cached `.trt` engines and takes longer.
Later starts load those engines directly.

The MuJoCo process can print a duplicate CycloneDDS initialization warning. It
is a known upstream issue. Confirm both `run_sim_loop.py` and
`g1_deploy_onnx_ref` remain running before treating the warning as fatal.

## Start Simulation Control

The stack creates `mujoco` and `sonic` tmux windows.

1. In the SONIC window, wait for `Init Done`.
2. Press `]` to enter control mode.
3. Press `ENTER` to enable `ZMQ STREAMING MODE`.
4. In the MuJoCo window, press `9` to release the elastic support.
5. Keep focus available to press `O` in SONIC to stop control.

Attach later with:

```bash
PATH="$HOME/Projects/sonic-build-deps/bin:$PATH" \
  tmux attach -t retarget-sonic-g1
```

## Publish a Recording

Recordings and generated outputs are intentionally not stored in Git. Obtain an
approved MVNX recording and export it first if `outputs/xsens-g1-batch` is
absent:

```bash
python3 batch_export.py \
  --input /approved/data/recording.mvnx \
  --output outputs/xsens-g1-batch
```

The generated subdirectory is the ASCII slug of the recording filename. The
examples below use the local approved `Pouring water.mvnx` recording; replace
`pouring-water` with the generated slug on another machine.

Validate without opening a socket:

```bash
.conda-sonic-stream/bin/python -m retargeting.sonic_stream \
  outputs/xsens-g1-batch/pouring-water/sonic_reference \
  --dry-run
```

Publish the clip:

```bash
.conda-sonic-stream/bin/python -m retargeting.sonic_stream \
  outputs/xsens-g1-batch/pouring-water/sonic_reference
```

Any generated clip can be selected by replacing the directory, for example:

```bash
.conda-sonic-stream/bin/python -m retargeting.sonic_stream \
  outputs/xsens-g1-batch/lifting-boxes/sonic_reference
```

The publisher checks the 50 Hz rate, 29-joint IsaacLab order, CSV row counts,
finite values, and normalized root quaternions before transmission.

To validate and list every generated clip:

```bash
.conda-sonic-stream/bin/python -m retargeting.sonic_playlist \
  outputs/xsens-g1-batch --dry-run
```

To play all generated references alphabetically through one running SONIC
session, with a three-second gap between clips:

```bash
.conda-sonic-stream/bin/python -m retargeting.sonic_playlist \
  outputs/xsens-g1-batch
```

The playlist validates and packs every clip before opening its loopback-only ZMQ
socket. MuJoCo can automatically reset after a fall; a reset does not make the
following reference dynamically valid.

## Stop

Press `O` in SONIC first, then stop the stack:

```bash
PATH="$HOME/Projects/sonic-build-deps/bin:$PATH" \
  python3 retargetctl.py stop sonic-g1-mujoco
```

## Known Limitations

- The first pouring-water run fell once and MuJoCo reset the robot.
- Dynamic tracking error, foot slip, root tilt, torque saturation, and precise
  fall timing are not yet exported.
- Current Pink calibration assumes frame zero is neutral, which is invalid for
  prone and supine recordings.
- Foot contacts are not locked during IK.
- The stack paths describe the validated local layout and must be adjusted if
  external dependencies are installed elsewhere.
- Real-robot execution is intentionally not defined by this repository.
