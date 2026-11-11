"""Local-only G1 hardware evidence checks with no transport capability."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

from .g1_dds_diagnostics import G1_RECORDED_FIRMWARE, G1_RECORDED_HANDS, G1_RECORDED_VARIANT
from .trajectory import SONIC_G1_JOINT_ORDER


REPORT_SCHEMA = "g1-hardware-readiness-report-v1"
REFERENCE_FORMAT = "sonic-g1-reference-v1"


@dataclass(frozen=True)
class ReadinessCheck:
    id: str
    status: str
    severity: str
    summary: str
    evidence: tuple[str, ...] = ()


@dataclass(frozen=True)
class ReadinessReport:
    schema: str
    dry_run_only: bool
    transmit_capability: bool
    robot_variant: str | None
    robot_firmware: str
    decision: str
    checks: tuple[ReadinessCheck, ...]

    def to_dict(self) -> dict:
        return asdict(self)


def _read_csv(path: Path, width: int) -> tuple[list[str], list[list[float]]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.reader(stream))
    if len(rows) < 2:
        raise ValueError(f"{path.name} has no data rows")
    header = rows[0]
    if len(header) != width:
        raise ValueError(f"{path.name} header has {len(header)} columns; expected {width}")
    values = []
    for row_index, row in enumerate(rows[1:], start=2):
        if len(row) != width:
            raise ValueError(f"{path.name}:{row_index} has {len(row)} columns; expected {width}")
        converted = [float(value) for value in row]
        if not all(math.isfinite(value) for value in converted):
            raise ValueError(f"{path.name}:{row_index} contains non-finite values")
        values.append(converted)
    return header, values


def evaluate_readiness(
    trajectory_directory: str | Path | None,
    *,
    acceptance_report: str | Path | None = None,
    sdk_root: str | Path | None = None,
) -> ReadinessReport:
    checks = [
        ReadinessCheck(
            "transport.absent",
            "PASS",
            "INFO",
            "This readiness tool has no command publisher or execution path.",
        ),
        ReadinessCheck(
            "robot.variant",
            "RECORDED",
            "WARNING",
            "The operator identified a G1 EDU+ with dummy hands; 29 body-state slots were observed over DDS.",
            ("Serial and manufacturer actuator-limit document are not yet recorded.",),
        ),
        ReadinessCheck(
            "robot.firmware",
            "RECORDED",
            "WARNING",
            "Firmware V1.5.1.1 was recorded by the operator but cannot be verified from the human-readable DDS low-state fields.",
            ("Read-only profile only; this is not command compatibility evidence.",),
        ),
        ReadinessCheck(
            "hardware.limits",
            "UNKNOWN",
            "BLOCKER",
            "Authoritative physical position, velocity, and acceleration limits are unavailable.",
        ),
    ]

    sdk = Path(sdk_root).expanduser().resolve() if sdk_root else None
    if sdk and (sdk / "unitree_sdk2py/core/channel.py").is_file():
        checks.append(
            ReadinessCheck(
                "sdk.source",
                "PASS",
                "INFO",
                "Local Unitree SDK2 Python source is available for future read-only diagnostics.",
                (str(sdk),),
            )
        )
    else:
        checks.append(
            ReadinessCheck(
                "sdk.source",
                "MISSING",
                "BLOCKER",
                "Unitree SDK2 Python source was not found at the configured sibling path.",
            )
        )

    if trajectory_directory is None:
        checks.append(
            ReadinessCheck(
                "trajectory.reference",
                "MISSING",
                "BLOCKER",
                "Select a G1 reference for local dry-run inspection.",
            )
        )
    else:
        directory = Path(trajectory_directory).expanduser().resolve()
        try:
            metadata = json.loads((directory / "trajectory_metadata.json").read_text(encoding="utf-8"))
            if metadata.get("format") != REFERENCE_FORMAT:
                raise ValueError(f"unexpected format {metadata.get('format')!r}")
            if metadata.get("rate_hz") != 50:
                raise ValueError("hardware review requires a 50 Hz reference")
            if metadata.get("joint_names") != list(SONIC_G1_JOINT_ORDER):
                raise ValueError("joint order does not match the 29-joint SONIC contract")
            _, positions = _read_csv(directory / "joint_pos.csv", len(SONIC_G1_JOINT_ORDER))
            _, velocities = _read_csv(directory / "joint_vel.csv", len(SONIC_G1_JOINT_ORDER))
            if len(positions) != len(velocities):
                raise ValueError("position and velocity row counts differ")
            maximum_velocity_error = 0.0
            maximum_acceleration = 0.0
            for frame in range(1, len(positions)):
                derived = [
                    (positions[frame][joint] - positions[frame - 1][joint]) * 50.0
                    for joint in range(len(SONIC_G1_JOINT_ORDER))
                ]
                maximum_velocity_error = max(
                    maximum_velocity_error,
                    *(abs(derived[joint] - velocities[frame][joint]) for joint in range(len(derived))),
                )
                if frame > 1:
                    maximum_acceleration = max(
                        maximum_acceleration,
                        *(
                            abs(velocities[frame][joint] - velocities[frame - 1][joint]) * 50.0
                            for joint in range(len(derived))
                        ),
                    )
            if maximum_velocity_error > 1e-3:
                raise ValueError(
                    f"velocity CSV differs from position derivative by {maximum_velocity_error:.4g} rad/s"
                )
            checks.append(
                ReadinessCheck(
                    "trajectory.reference",
                    "PASS",
                    "INFO",
                    "Reference format, rate, dimensions, finiteness, and velocity consistency pass.",
                    (
                        str(directory),
                        f"frames={len(positions)}",
                        f"max_acceleration_unqualified={maximum_acceleration:.3f} rad/s²",
                    ),
                )
            )
        except (OSError, ValueError, json.JSONDecodeError) as error:
            checks.append(
                ReadinessCheck(
                    "trajectory.reference",
                    "FAIL",
                    "BLOCKER",
                    f"Reference validation failed: {error}",
                    (str(directory),),
                )
            )

    if acceptance_report is None:
        checks.append(
            ReadinessCheck(
                "simulation.acceptance",
                "MISSING",
                "BLOCKER",
                "No eligible unsupported dynamic simulation acceptance report was supplied.",
            )
        )
    else:
        path = Path(acceptance_report).expanduser().resolve()
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            accepted = payload.get("eligible") is True and payload.get("accepted") is True
            checks.append(
                ReadinessCheck(
                    "simulation.acceptance",
                    "PASS" if accepted else "FAIL",
                    "INFO" if accepted else "BLOCKER",
                    "Acceptance report explicitly marks the trial eligible and accepted."
                    if accepted
                    else "Acceptance report is not explicitly both eligible and accepted.",
                    (str(path),),
                )
            )
        except (OSError, json.JSONDecodeError) as error:
            checks.append(
                ReadinessCheck(
                    "simulation.acceptance",
                    "FAIL",
                    "BLOCKER",
                    f"Cannot read acceptance report: {error}",
                    (str(path),),
                )
            )

    return ReadinessReport(
        schema=REPORT_SCHEMA,
        dry_run_only=True,
        transmit_capability=False,
        robot_variant=f"{G1_RECORDED_VARIANT} / {G1_RECORDED_HANDS}",
        robot_firmware=G1_RECORDED_FIRMWARE,
        decision="BLOCKED",
        checks=tuple(checks),
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path)
    parser.add_argument("--acceptance-report", type=Path)
    parser.add_argument(
        "--sdk-root",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "unitree_sdk2_python",
    )
    args = parser.parse_args(argv)
    report = evaluate_readiness(
        args.trajectory,
        acceptance_report=args.acceptance_report,
        sdk_root=args.sdk_root,
    )
    print(json.dumps(report.to_dict(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
