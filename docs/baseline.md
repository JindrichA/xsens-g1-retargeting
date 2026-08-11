# Retargeting Baseline

## 2026-08-11: Grounded GUI Playback

Input: `outputs/xsens-g1-batch/lifting-boxes/sonic_reference`

Configuration:

- PyQt simulation console on loopback-only ZMQ.
- Two-second nominal standing prefix at 50 Hz.
- Selected reference decoded before SONIC control activation.
- MuJoCo elastic support retained for 1.5 seconds after control activation, then
  released by one key event sent to the graphical MuJoCo viewer.

Results:

- SONIC established Protocol v1 and selected `Reference motion name: streamed`.
- The 841-frame combined standing-prefix/reference message completed.
- MuJoCo and SONIC remained connected and the controller remained active after
  completion.
- The earlier floating behavior was traced to sending `9` to the tmux terminal;
  MuJoCo's GLFW callback only receives the event through its graphical window.

This confirms the repeatable GUI lifecycle and grounded controller path. It does
not replace dynamic telemetry or prove that every recording is fall-free.

## 2026-08-10: First SONIC Dynamic Playback

Input: `outputs/xsens-g1-batch/pouring-water/sonic_reference`

Configuration:

- NVIDIA GEAR-SONIC default G1 release policy and encoder.
- TensorRT 10.13.3, ONNX Runtime 1.16.3, MuJoCo 3.11.
- SONIC joint-based ZMQ Protocol v1 at 50 Hz.
- 1,432 frames, 28.62 seconds, `catch_up=false`.
- MuJoCo gravity/contact simulation on loopback DDS only.
- Elastic support released before streaming.

Results:

- SONIC decoded all five fields and accepted all 1,432 frames in IsaacLab joint
  order.
- Encoder and policy observation dimensions matched their released models.
- The reference completed without protocol, stale-input, or inference errors.
- Policy inference was normally about 68-141 microseconds per control step.
- MuJoCo reported one fall at `0.199 m` root height and automatically reset the
  robot.

This proves the offline Xsens-to-Pink-to-SONIC transport and dynamic-control loop
works end to end. It does not establish dynamic feasibility: even the easiest
upright clip fell once, and tracking error, foot slip, torque saturation, and
root tilt are not yet captured in a machine-readable report.

## 2026-08-07: Initial 12-Recording Export

Command:

```bash
python3 batch_export.py
```

Configuration:

- Pink IK with four solver iterations per 50 Hz frame.
- Global human displacement scale `0.75`.
- Joint-speed limit `4 rad/s`.
- Near-limit threshold `0.05 rad`.
- First MVNX frame used as neutral calibration for each recording.
- Kinematic Isaac playback; no gravity or balancing controller.

Results:

- 12 of 12 recordings exported successfully.
- 29,460 G1 reference frames.
- 588.96 seconds at 50 Hz.
- Matching row counts in joint position, joint velocity, root position, and root
  quaternion files.
- Total local output size: approximately 71 MB.

The complete local reports are under `outputs/xsens-g1-batch/` and are ignored
by Git because they are generated artifacts.

## Main Findings

The current retargeter performs best on upright, small-range tasks:

| Recording | Torso RMS | Left wrist RMS | Right wrist RMS | Ankle RMS range |
| --- | ---: | ---: | ---: | ---: |
| Pouring water | 2.50 cm | 0.00 cm | 0.06 cm | < 0.01 cm |
| Pouring water with gloves | 3.06 cm | 0.01 cm | 0.08 cm | < 0.01 cm |
| Small object picking and placing | 2.08 cm | 0.00 cm | 0.02 cm | < 0.01 cm |

Large posture changes expose major limitations:

| Recording | Important failure |
| --- | --- |
| Bending forward | 11.45 cm torso RMS; 7.52-8.72 cm wrist RMS |
| Stand up from prone | 26.20-31.48 cm wrist RMS; 209/247 near-limit frames |
| Stand up from supine | 11.09 cm left-wrist RMS; 28.04 cm maximum right-ankle error |
| Put pens in a box | 1,874/5,241 rate-limited frames |
| Put pen in box with gloves | 1,736/5,522 rate-limited and 1,572/5,522 near-limit frames |

## Interpretation

- A single global `0.75` scale is insufficient across torso, arms, and legs.
- Treating frame 0 as a standing neutral pose is invalid for prone and supine
  recordings.
- Feet are continuously retargeted even during Xsens contact, so stance-foot
  locking is still required.
- Long manipulation clips accumulate frequent speed limiting and wrist/hip
  limit pressure.
- Successful export means the files satisfy the measured format and dimensions.
  The first SONIC test accepted them but fell once on `Pouring water`, so export
  remains insufficient evidence of dynamic feasibility.

## Next Comparison

The next baseline should use:

1. Recording-aware neutral calibration.
2. Separate torso, arm, and leg scales.
3. Xsens heel/toe contact-based foot locking.
4. Dynamic metrics for root height/tilt, foot slip, tracking error, limits, and
   torque saturation.
5. The same twelve recordings and output metrics for a direct comparison.
