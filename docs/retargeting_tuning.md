# Retargeting Scale Tuning

The shared retargeter supports separate translation scales for torso, wrist, and
ankle tasks. Orientation targets are unchanged. The global `--scale` remains the
fallback and defaults to the historical `0.75`, so existing commands retain the
baseline behavior.

## Parameters

| Option | Targets | Default |
| --- | --- | --- |
| `--scale` | Root translation and any task without an override | `0.75` |
| `--torso-scale` | T8 to G1 torso displacement | Global scale |
| `--arm-scale` | Left and right wrist displacement | Global scale |
| `--leg-scale` | Left and right ankle displacement | Global scale |

Do not select values from appearance alone. Lower IK error can produce a less
trackable reference or more joint-limit pressure.

## Reproducible Comparison

Use separate output directories because a batch output root can retain artifacts
from an older run. Begin with short representative clips before exporting the
complete dataset.

```bash
python3 batch_export.py \
  --input "xsens_TJ_08072026/MVNX export/Bending forward.mvnx" \
  --output outputs/scales/baseline \
  --profile-name baseline \
  --max-frames 500

python3 batch_export.py \
  --input "xsens_TJ_08072026/MVNX export/Bending forward.mvnx" \
  --output outputs/scales/longer-arms \
  --profile-name longer-arms \
  --torso-scale 0.65 \
  --arm-scale 0.80 \
  --leg-scale 0.70 \
  --max-frames 500

python3 -m retargeting.compare_retargeting \
  outputs/scales/baseline/manifest.json \
  outputs/scales/longer-arms/manifest.json \
  --output outputs/scales/comparison.csv
```

Each batch manifest records its profile and resolved scales. Each individual
SONIC trajectory records the full solver configuration used to generate it.

Compare at least:

- torso and wrist RMS/max position and orientation error;
- ankle error, while remembering that low kinematic ankle error does not prove
  stable contact;
- rate-limited frame count and fraction;
- near-limit frame count and fraction;
- SONIC dynamic falls and tracking metrics once telemetry is available.

No non-baseline scale profile is a project default yet. A new default should be
selected only after the same profile improves representative clips without
making dynamic simulation less stable.

## Preliminary Result

A 500-frame comparison on `Bending forward.mvnx` tested the baseline `0.75`
against torso `0.65`, arms `0.80`, and legs `0.70`. The candidate reduced torso
position RMS from `10.82 cm` to `9.37 cm` and reduced ankle position RMS from
`0.13/0.08 cm` to `0.07/0.05 cm`. However, left and right wrist position RMS
increased from `7.90/7.16 cm` to `9.07/8.58 cm`. Near-limit incidence was
unchanged at `41%`; rate-limited incidence changed from `9.4%` to `9.0%`.

This candidate is therefore not a replacement default. Test future candidates
on multiple upright, manipulation, and lower-body clips, then validate the
resulting references in SONIC dynamics before changing the baseline.
