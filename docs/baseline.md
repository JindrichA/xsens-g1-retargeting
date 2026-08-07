# Retargeting Baseline

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
- Successful export means the files satisfy the measured format and dimensions;
  it does not mean SONIC can dynamically track them under gravity.

## Next Comparison

The next baseline should use:

1. Recording-aware neutral calibration.
2. Separate torso, arm, and leg scales.
3. Xsens heel/toe contact-based foot locking.
4. The same twelve recordings and output metrics for a direct comparison.
