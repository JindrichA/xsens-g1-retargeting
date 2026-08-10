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
