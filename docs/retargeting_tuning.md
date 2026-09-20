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

## Multi-Clip Sweep

Run the default axis sweep on representative torso, manipulation, and
lower-body clips. It evaluates the baseline plus lower and higher values for one
chain at a time, for seven profiles total:

```bash
python3 -m retargeting.scale_sweep \
  --input "xsens_TJ_08072026/MVNX export" \
  --output outputs/scales/representative-axis \
  --recording "Bending forward.mvnx" \
  --recording "Lifting boxes.mvnx" \
  --recording "Small object picking and placing.mvnx" \
  --recording-start "Small object picking and placing.mvnx=1000" \
  --max-frames 500
```

Use `--dry-run` to inspect every command without launching Isaac. The default
values are `0.65,0.75,0.85` for each chain. Custom comma-separated values are
accepted through `--torso-scales`, `--arm-scales`, and `--leg-scales`.
`--cartesian` tests every combination and is bounded by `--max-profiles` to
prevent an accidental large simulator run.

`--recording-start NAME.mvnx=FRAME` selects a per-clip window in 50 Hz output
frames. The retargeter still calibrates on frame zero and solves every preceding
frame as pre-roll, preserving solver history before diagnostics begin. Use this
for long recordings whose opening interval is static.

The sweep writes `comparison.csv`, `ranking.csv`, and `ranking.json`. Ranking is
frame-weighted across clips. A candidate becomes ineligible if any clip's
individual task position or orientation RMS regresses by more than 5%, or if
that clip's rate-limited or near-limit fraction rises by more than one
percentage point. Regression checks use floors of `0.1 cm` and `0.5 deg` so
numerical noise around an almost-zero baseline is not treated as a large
percentage regression.
These thresholds can be changed explicitly, but a favorable rank remains only
a kinematic shortlist for SONIC dynamic validation. Use `--rank-only` with the
same profile arguments and output directory to apply new ranking thresholds to
completed manifests without rerunning Isaac.

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

## Multi-Clip Result

A 500-frame, three-clip axis sweep covered bending, lifting, and small-object
manipulation. A focused 12-profile Cartesian refinement then tested torso
`{0.65, 0.75}`, arms `{0.65, 0.70, 0.75}`, and legs `{0.65, 0.75}`. The best
eligible profile was:

| Scale | Value |
| --- | --- |
| Global/root | `0.75` |
| Torso | `0.65` |
| Arms | `0.75` |
| Legs | `0.65` |

Relative to baseline, its frame-weighted position score was `0.8780` and its
orientation score was `0.9494`. Torso position RMS decreased by `13.4%`, leg
position RMS decreased by `24.2%`, and arm position RMS was effectively
unchanged. Rate-limited incidence remained `13.0%`; near-limit incidence changed
from `32.47%` to `32.53%`.

Arm scales `0.65` and `0.70` improved wrist tracking but were rejected because
the lifting clip's near-limit incidence increased by `1.8` percentage points.
The torso/leg candidate is a shortlist for broader-window and SONIC dynamic
validation, not the new project default.

A second 500-frame window beginning at output frame `3750` of the 89-second
small-object recording preserved the result. The combined candidate scored
`0.9729` for position and `0.9999` for orientation, with zero rate-limited
frames and the same `38.4%` near-limit incidence as baseline. Most of that
window's measurable gain came from torso scaling; its ankle errors were already
below the ranking noise floor.
