# Current Project Status

Last reviewed: 2026-11-11.

This project is an experimental, simulation-first Xsens-to-G1 control plane.
No repository stack starts physical whole-body control. A separate guarded
command supports finite physical G1 upper-body commissioning.

| Capability | Implemented | Validated | Remaining work |
| --- | --- | --- | --- |
| MVNX parsing and validation | Yes | 12-recording source dataset | Preserve dataset provenance |
| Shared recorded/live Pink IK | Yes | Three-clip axis and focused Cartesian scale sweeps | Validate the torso/leg candidate in broader windows and SONIC, then improve calibration and contacts |
| SONIC Protocol v1 export | Yes | Format and transport tests | Improve reference feasibility |
| SONIC MuJoCo playback | Yes | Initial dynamic run | Add dynamic telemetry and achieve stable baselines |
| Mock live input | Yes | Atomic boundary and retarget process | Repeat operator-level scenario testing |
| Live ROS capture and retargeting | Yes | Synthetic/unit tests only | Validate suit traffic and network failures |
| Unique-frame stale watchdog | Yes, 250 ms | Unit tested | Add acknowledged stop behavior |
| Joint position and velocity limiting | Yes | Unit/integration coverage is partial | Add direct retargeter tests |
| Acceleration and pose-jump limiting | No | No | Implement before unrestricted live trials |
| Physical G1 output | Subscriber-only diagnostics, offline `LowCmd` construction, and explicit `rt/arm_sdk` upper-body playback; whole-body transmission remains absent | Regular Mode FSM 501 tracked corrected 17-slot upper-body commands up to 0.2946 rad with 0.0382 rad peak error; CRC, waist hold, release, and motor checks passed | Cool and evaluate the prepared bounded full arm sequence; obtain the manufacturer control contract before any whole-body transport |

## Safety Interpretation

“Implemented” does not mean dynamically safe. The current live adapter can
calibrate, gate output, detect duplicate/stale source frames, and request a SONIC
stop. It does not yet enforce every target constraint in the architecture, and
the stop request has no structured controller acknowledgement.

Simulation acceptance still needs machine-readable fall, root tilt, foot slip,
tracking-error, and torque-saturation metrics. Real-robot work remains blocked
until those criteria and emergency procedures are defined and passed.

Physical G1 experiments established that Regular Mode can retain onboard
balance while `rt/arm_sdk` overlays the three waist and fourteen arm slots. The
executor holds the measured waist pose and applies only arm deltas. This is not
whole-body Xsens imitation: Regular Mode still owns the legs and floating-base
behavior. The current full-arm v3 candidate is documented but unexecuted because
its 45 C cool-start gate observed a 53 C shoulder. Physical SONIC/`rt/lowcmd`
remains blocked by unknown firmware watchdog, stale-command, gain, emergency
stop, and controller-restoration contracts. See the
[G1 physical experiment log](g1_physical_experiments.md) for measured trials and
next steps.
