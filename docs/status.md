# Current Project Status

Last reviewed: 2026-09-15.

This project is an experimental, simulation-only Xsens-to-G1 control plane. No
repository stack targets a physical robot.

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
| Physical G1 output | No | No | Requires a separate explicit profile and acceptance gate |

## Safety Interpretation

“Implemented” does not mean dynamically safe. The current live adapter can
calibrate, gate output, detect duplicate/stale source frames, and request a SONIC
stop. It does not yet enforce every target constraint in the architecture, and
the stop request has no structured controller acknowledgement.

Simulation acceptance still needs machine-readable fall, root tilt, foot slip,
tracking-error, and torque-saturation metrics. Real-robot work remains blocked
until those criteria and emergency procedures are defined and passed.
