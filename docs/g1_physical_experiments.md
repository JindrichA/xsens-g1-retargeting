# G1 Physical Experiment Log

Last updated: 2026-11-11.

## Scope

This log records the physical commissioning performed on one Unitree G1 EDU+
with 29 body joints, dummy hands, and operator-recorded firmware `V1.5.1.1`.
The firmware string is not available in human-readable form through
`rt/lowstate`, so it remains operator-recorded rather than DDS-verified.

The physical work described here is limited to subscriber-only diagnostics and
the high-level `rt/arm_sdk` upper-body overlay. Physical whole-body
`rt/lowcmd` transmission has never been enabled by this repository.

## Hardware And Transport Evidence

- A dedicated wired Ethernet interface reached DDS domain 0.
- `rt/lowstate` supplied advancing ticks and valid CRCs on repeated samples.
- The observed controller fields were `mode_machine=5` and `mode_pr=0`.
- All 29 expected body-state slots were present.
- No motor error words were observed during the recorded experiments.
- Unitree Regular Mode was identified as locomotion FSM `501`.
- The motion switcher continued to report the onboard `ai` service name; the
  locomotion FSM, not that label, distinguished inactive FSM 0 from Regular
  Mode FSM 501.

## Simulation Qualification

SOMA `Pouring water` frames 600 through 700 were evaluated in MuJoCo before
physical actuation. Two eligible trials completed without falls or resets, with
100% command freshness and contact, no torque saturation, maximum root tilt of
approximately 3.49 degrees, and tracking RMS of approximately 0.162 rad.

This qualified only the short simulation window. It did not qualify physical
whole-body control or the complete 1,432-frame recording.

## Command-Authority Experiments

Initial `rt/arm_sdk` publications in inactive FSM 0 were accepted by DDS but
were not tracked. A commanded displacement of 0.1375 rad produced only
approximately 0.0001 rad of measured arm motion.

The official Arm Action service was reachable, but tested action IDs returned
firmware result `7404`. Calling `SetFsmId(500)` returned RPC success while the
reported FSM remained 0. The Unitree user-control and internal-control APIs
`7110` and `7111` were also reachable. A `7110` handoff changed controller
behavior but did not make changing Arm SDK targets track in the inactive mode.
Internal control was restored after every handoff trial.

The robot began tracking Arm SDK commands only after the operator selected
Regular Mode with three waist degrees of freedom and the locomotion service
reported FSM 501. The executor now preserves an already-active Regular Mode
instead of replacing it with the user-control handoff.

## Arm SDK Contract Correction

Unitree's 29-DOF Arm SDK example treats `rt/arm_sdk` as a 17-joint upper-body
interface: waist yaw, waist roll, waist pitch, and both seven-joint arms. Early
packets populated only the 14 arm slots. During one trial the operator observed
waist bending while the expected hand motion was absent, and the tilt guard
aborted the run.

That result invalidated the earlier 14-slot physical qualification. The
executor was corrected to hold all three measured waist positions in every
packet, apply recorded deltas only to the arms, and monitor tracking over all 17
upper-body joints.

## Successful Physical Trials

All successful trials below ran in Regular Mode FSM 501, returned the Arm SDK
weight to zero, preserved the locomotion controller, and passed post-run CRC,
motor-state, posture, and temperature checks.

| Source window | Source/playback | Scale | Planned maximum | Observed maximum | Peak tracking error | Result |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| 600-700 | 2.0 s / 20.0 s | 1.0 under the former v1 envelope | 0.1375 rad | 0.1355 rad | 0.0371 rad | Passed after the 17-slot correction |
| 500-800 | 6.0 s / 60.0 s | 0.25 | 0.1492 rad | 0.1483 rad | 0.0321 rad | Passed |
| 500-800 | 6.0 s / 60.0 s | 0.50 under the former v2 envelope | 0.2984 rad | 0.2946 rad | 0.0382 rad | Passed |

Post-run maximum upper-body temperature was 52 C in the successful trials. The
waist remained near its held pose after the corrected runs, and no motor error
word was reported.

## Prepared Full Arm Sequence

The startup ramp was removed and frames 150 through 1431 were analyzed as the
complete recorded arm movement. Unrestricted full-scale playback was rejected:

- Peak relative arm displacement is 1.29 rad from frame 150 and 1.81 rad from
  frame 0.
- Peak source speed is 4.0 rad/s.
- The frame-0 full-scale plan consumes approximately 98% of the available left
  elbow position margin from the measured robot pose.
- The complete recording has not received equivalent full-window dynamic
  acceptance evidence.

The current `g1-edu-plus-arm-sdk-v3` candidate uses frames 150 through 1431,
scale 0.399, and speed scale 0.1. Its validated plan is:

- 25.62 seconds of source motion.
- 256.2 seconds of physical playback.
- 0.5148 rad maximum relative arm displacement.
- 0.1596 rad/s maximum commanded arm speed.
- Measured waist hold throughout the sequence.
- A 45 C cool-start limit and 55 C runtime abort.

The candidate has not been physically executed. The last preflight measured a
left shoulder temperature of 53 C, so the 45 C cool-start gate rejected the
run before any command publication. A requested 58 C threshold was not adopted
because no authoritative Unitree thermal rating supporting it was found in the
available SDK or documentation.

## Current Capability Boundary

Regular Mode can combine onboard balance and locomotion with the Arm SDK
upper-body overlay. It cannot exactly replay Xsens hip, knee, ankle, or floating
base trajectories because the onboard locomotion controller owns those degrees
of freedom. Dummy hands also cannot reproduce finger motion.

A hybrid Regular Mode demonstration may replay arms and bounded waist motion
while deriving high-level walking velocity commands from Xsens. That would be
an approximation, not whole-body imitation.

Full-body Xsens imitation remains a SONIC goal. It requires SONIC to own the
whole-body control loop, so it cannot safely compete with Regular Mode for the
same joints.

## Open Blockers

Physical whole-body SONIC/`rt/lowcmd` remains blocked pending authoritative
answers for firmware `V1.5.1.1`:

- Supported transition into and out of external whole-body control.
- Required command cadence and firmware watchdog timeout.
- Behavior after stale commands, network loss, or publisher termination.
- Approved gains and commissioning limits for all 29 joints.
- Emergency-stop, safe-hold, and controller-restoration procedures.
- Supported thermal limits.
- Official status of APIs 7110 and 7111 and any supported SONIC integration.

The local `g1_lowcmd_dry_run` validates message layout, joint mapping, URDF
bounds, interpolation, disabled templates, and CRC without creating transport.
It is evidence about packet construction only, not permission to transmit.

## Next Steps

1. Allow all upper-body motors to cool to 45 C or below before considering the
   prepared v3 arm sequence.
2. Keep the robot in Regular Mode FSM 501 for any further Arm SDK trial and
   retain workspace, support, and emergency-stop requirements.
3. Obtain the low-level firmware control contract from Unitree and the expected
   physical controller lifecycle from the GR00T WholeBodyControl maintainers.
4. Run complete SONIC references through MuJoCo with eligible dynamic telemetry,
   stale-input injection, and controller-failure tests.
5. Implement and review a fail-closed SONIC hardware bridge only after those
   requirements are known.
6. Commission any future whole-body path from supported zero-motion holds and
   small movements before attempting a complete recording.

## Verification State

The repository test suite passed 219 tests after the current v3 plan, thermal
gate, Regular Mode preservation, 17-slot upper-body contract, and tracking
telemetry were added. Tests do not launch ROS, simulators, GUIs, or robot
controllers.
