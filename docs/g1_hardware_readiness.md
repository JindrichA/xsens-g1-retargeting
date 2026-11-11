# G1 Hardware Readiness

## Scope

The G1 hardware tools separate three capabilities: an offline readiness report,
a subscriber-only DDS diagnostic, and an explicit guarded Arm SDK executor. The
readiness and diagnostic commands cannot transmit. The Arm SDK executor can
publish only after an explicit physical profile and acknowledgement; physical
whole-body `rt/lowcmd` remains unavailable.

The operator identified the robot as a `G1 EDU+` with dummy hands and firmware
`V1.5.1.1`. Read-only DDS observed 29 body-state slots with valid CRCs and no
motor error words. The serial, authoritative actuator limits, and firmware
control contract remain unavailable. The DDS low-state message
contains raw version words, not the human-readable firmware string, so the
diagnostic does not claim to verify `V1.5.1.1` from DDS.

## Control Ownership Mental Model

The critical question is not only which topic receives a message, but which
controller currently owns each joint.

| Operating model | Leg and balance owner | Waist and arm owner | Exact Xsens whole-body replay |
| --- | --- | --- | --- |
| Regular Mode without an overlay | Unitree | Unitree | No |
| Regular Mode with `rt/arm_sdk` | Unitree | External overlay while weight is nonzero | No; upper body only |
| Physical SONIC/`rt/lowcmd` | SONIC would need exclusive authority | SONIC would need exclusive authority | Intended, but not commissioned |

Regular Mode FSM 501 is not the same as unrestricted external control. It keeps
the onboard locomotion and balance controller active. The Arm SDK weight blends
external upper-body targets into that controller; weight zero returns upper-body
ownership to Unitree.

The Arm SDK packet has 35 motor slots, but on this 29-DOF robot its active
contract covers 17 upper-body joints: three waist joints and fourteen arm
joints. The executor holds the measured waist targets and changes only the arm
targets. Leg slots remain disabled in every Arm SDK packet.

SONIC is different. A physical SONIC controller would generate the coordinated
whole-body command, including balance and leg behavior. Regular Mode cannot
remain the competing owner of those same joints. A clean mode transition and a
verified fallback are therefore prerequisites, not optional setup details.

## What Can Be Replayed Now

| Xsens content | Regular Mode Arm SDK | Future physical SONIC |
| --- | --- | --- |
| Shoulder, elbow, and wrist joints | Yes, as bounded relative motion | Intended |
| Waist motion | Technically part of the overlay; currently held for stability | Intended |
| Hip, knee, and ankle angles | No | Intended |
| Floating-base position and orientation | No | Intended through whole-body tracking |
| Walking path | Approximate only through high-level velocity commands | Intended as coordinated tracking |
| Finger motion | No; this robot has dummy hands | No without actuated hands |

"Full recording" can mean two different things. The complete recording timeline
can be played through the arm overlay, but only its bounded arm deltas are sent.
That is not full-body replay. Exact lower-body and root imitation requires the
whole-body controller path.

## Choose The Correct Path

Use the read-only diagnostic when the goal is to verify connectivity, firmware
identity evidence, controller mode, CRC, temperatures, or motor faults.

Use the offline `LowCmd` dry run when the goal is to inspect joint order,
interpolation, CRC construction, URDF bounds, or stale-template behavior. It
does not establish physical command safety.

Use the Arm SDK executor when the goal is a finite, bounded physical arm
demonstration while Regular Mode maintains balance. Require FSM 501, a clear
workspace, remote emergency stop, cool motors, and the explicit profile.

Use SONIC in MuJoCo when the goal is coordinated whole-body imitation,
controller tuning, or failure testing. Do not move that path to the physical
robot until the Unitree firmware control contract and fail-safe lifecycle are
known and validated.

## Local Dry Run

Select `Unitree G1`, open `G1 Hardware`, choose a recording in the recording
library, and click `RUN LOCAL DRY-RUN CHECK`. The report checks:

- Exact `sonic-g1-reference-v1` metadata and 50 Hz rate.
- Exact 29-joint SONIC order.
- CSV dimensions and finite values.
- Agreement between position differences and recorded velocities.
- Availability of the sibling `unitree_sdk2_python` source.
- Presence of an explicitly eligible and accepted simulation report.
- Presence of a reviewed physical variant and physical limits.

The readiness report remains `BLOCKED` for whole-body hardware control because
authoritative limits and the firmware control contract are unavailable. The
report itself always states `transmit_capability: false`; that does not describe
the separate, explicitly invoked Arm SDK executor.

```bash
python3 -m retargeting.g1_hardware_readiness \
  --trajectory outputs/xsens-g1-batch/pouring-water/sonic_reference
```

## Read-Only DDS

After the robot arrives, the GUI can run a subscriber-only diagnostic on an
explicit wired interface. It uses:

```text
DDS domain: 0
Topic: rt/lowstate
Type: unitree_hg.msg.dds_.LowState_
Slots inspected: 0-28
```

The worker imports `ChannelFactoryInitialize` and `ChannelSubscriber` only. It
does not import `ChannelPublisher`, `LowCmd_`, or an RPC motion client. It reads
three state samples, verifies every hardware CRC and an advancing state tick,
reports stable raw version words, controller modes, IMU state, joint positions,
velocities, estimated torques, temperatures, voltages, and motor error words,
then closes the reader. Firmware values other than `V1.5.1.1` are rejected
because no other read-only profile has been recorded.

```bash
python3 -m retargeting.g1_dds_diagnostics \
  --interface enp3s0 \
  --recorded-firmware V1.5.1.1
```

DDS discovery itself emits protocol traffic, but no application command topic
or request writer is created.

Prerequisites are the sibling `../unitree_sdk2_python` checkout and its pinned
CycloneDDS dependency. The physical check must use a dedicated non-loopback
wired interface. `lo`, DDS domain 1, and `--disable-crc-check` are simulation
conventions and are not accepted for physical diagnostics.

## Recorded Commissioning Procedure

1. Record the exact model, serial, joint option, firmware, and SDK versions.
2. Verify remote emergency stop and suspension equipment.
3. Connect a dedicated wired interface without starting a controller.
4. Run the subscriber-only diagnostic and require a valid CRC and advancing
   state tick.
5. Record Regular Mode FSM 501 before using the Arm SDK overlay.
6. Repeat simulation acceptance using matching policy and model hashes.
7. Use only the dedicated Arm SDK profile; keep whole-body control blocked.

No successful readiness report should be interpreted as permission to actuate.

## Terms And Signals

| Term | Meaning in this project |
| --- | --- |
| FSM 0 | Locomotion service inactive in the observed experiments; Arm SDK targets were not tracked |
| FSM 501 | Operator-selected Unitree Regular Mode with three waist degrees of freedom |
| `rt/lowstate` | Read-only feedback used for CRC, pose, mode, temperature, and motor checks |
| `rt/arm_sdk` | High-level 17-joint upper-body overlay used with Regular Mode |
| `rt/lowcmd` | Low-level whole-body command topic; this repository does not physically publish it |
| Arm SDK weight | Blend authority in motor slot 29; zero releases the overlay, one requests full upper-body overlay authority |
| `mode_machine=5` | Observed 29-DOF machine mode copied into Arm SDK packets; not by itself proof of command authority |
| SONIC reference | 29-joint positions/velocities plus root orientation consumed by the SONIC controller; not a Unitree motor packet |

## Offline LowCmd Dry Run

The offline generator validates the official 35-slot HG message layout, maps the
29 body joints into hardware order, checks mode-5 URDF position and velocity
bounds, interpolates a finite reference to 500 Hz, and calculates every message
CRC with Unitree SDK2 Python. It imports no channel factory, publisher, or motion
switcher and has no network-interface argument.

```bash
.venv-g1-readonly/bin/python -m retargeting.g1_lowcmd_dry_run \
  outputs/soma-g1-batch-final/pouring-water/sonic_reference \
  --start-frame 600 \
  --frame-count 101 \
  --watchdog-ms 20 \
  --inject-stale-after-ms 1000
```

Active in-memory templates use zero gains and zero feed-forward torque. The
injected watchdog event produces disabled templates only. This demonstrates
local formatting and fail-closed state transitions; it does not establish the
physical meaning of motor disable or the firmware's onboard watchdog behavior.

The gain table from Unitree SDK2 Python `master` commit
`814556d15970dd2ecf1c9984e845ca02ab07e206` can also be checked in memory with
`--gain-profile sdk-master-example`. This option does not add transport and
disabled templates still use zero gains. The upstream example provides no
firmware watchdog, emergency-stop, or commissioning guarantee, so selecting
this profile does not make the report suitable for physical execution.

## Arms-Only Arm SDK Path

`retargeting.g1_arm_sdk_playback` avoids whole-body `rt/lowcmd` control. It uses
Unitree's documented `rt/arm_sdk` overlay while leaving the onboard locomotion
service active. A plan is aligned to the measured start pose, checked against
the active profile's displacement and speed ceilings, returned to the initial
pose, and released by ramping the documented Arm SDK weight in slot 29 to zero.

```bash
.venv-g1-readonly/bin/python -m retargeting.g1_arm_sdk_playback \
  outputs/soma-g1-batch-final/pouring-water/sonic_reference \
  --start-frame 600 \
  --frame-count 101
```

Physical mode is never the default. It additionally requires the explicit
`g1-edu-plus-arm-sdk-v3` profile, wired interface, operator acknowledgement,
three fresh CRC-valid low-state samples, mode 5, bounded tilt, normal arm motor
states, a 45 C cool-start limit, a 55 C runtime limit, and tracking error below
0.25 rad. The remote emergency stop and a clear arm workspace remain operator
requirements.
An execution is rejected if feedback does not show at least 0.01 rad of arm
motion; successful publication alone is not reported as successful tracking.
Each Arm SDK message copies the fresh `mode_machine` value from LowState, sets
`mode_pr=0`, and enables the 17 upper-body slots while leaving all leg slots
disabled. Outside Regular Mode, the executor has a bounded fallback using
`SwitchToUserCtrl` (`7110`) and unconditional `SwitchToInternalCtrl(LAST)`
(`7111`) restoration. Physical target tracking was observed only in Regular
Mode FSM 501; a successful handoff response in inactive FSM 0 did not establish
command authority.
The initial arm pose is captured again after a one-second user-control settling
period so the activation ramp never tracks a stale pre-handoff pose.
State, CRC, mode, tilt, motor, and temperature checks remain active during the
weight ramp. Strict target tracking starts only at weight 1 because lower
weights intentionally blend the internal and user arm targets.
If the sport service already reports Regular Mode FSM 500/501/801/802, that
controller is preserved and no user-control handoff or restoration RPC is sent.
On the 29-DOF G1, Unitree's Arm SDK owns all 17 upper-body slots: waist
yaw/roll/pitch and both seven-joint arms. The executor holds the three measured
waist positions in every packet and applies trajectory deltas only to the arms.
The guarded v3 profile permits the 25.62-second trimmed pouring sequence at up
to 0.4 amplitude, 0.55 rad displacement, and 0.16 rad/s commanded speed. Long
runs require every upper-body motor to be at or below 45 C before activation;
the 55 C runtime abort remains active.

See [G1 Physical Experiment Log](g1_physical_experiments.md) for the complete
chronology, measured trial results, current blockers, and next steps.
