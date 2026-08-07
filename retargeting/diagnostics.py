"""Dependency-free recording and export of retargeting diagnostics."""

from __future__ import annotations

from collections import Counter
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


@dataclass(frozen=True)
class FrameDiagnostics:
    source_frame: int
    time_seconds: float
    position_errors_m: Mapping[str, float]
    orientation_errors_rad: Mapping[str, float]
    rate_limited_joints: tuple[str, ...] = ()
    near_limit_joints: tuple[str, ...] = ()


class DiagnosticsRecorder:
    def __init__(self, task_names: Sequence[str]):
        if not task_names or len(set(task_names)) != len(task_names):
            raise ValueError("diagnostic task names must be non-empty and unique")
        self.task_names = tuple(task_names)
        self.frames: list[FrameDiagnostics] = []

    def add(self, frame: FrameDiagnostics) -> None:
        expected = set(self.task_names)
        if set(frame.position_errors_m) != expected or set(frame.orientation_errors_rad) != expected:
            raise ValueError("frame diagnostic tasks do not match recorder tasks")
        values = [*frame.position_errors_m.values(), *frame.orientation_errors_rad.values()]
        if not all(math.isfinite(value) and value >= 0 for value in values):
            raise ValueError("diagnostic errors must be finite and non-negative")
        self.frames.append(frame)

    def summary(self) -> dict[str, object]:
        tasks = {}
        for name in self.task_names:
            position = [frame.position_errors_m[name] for frame in self.frames]
            orientation = [frame.orientation_errors_rad[name] for frame in self.frames]
            tasks[name] = {
                "rms_position_cm": _rms(position) * 100.0,
                "max_position_cm": max(position, default=0.0) * 100.0,
                "rms_orientation_deg": math.degrees(_rms(orientation)),
                "max_orientation_deg": math.degrees(max(orientation, default=0.0)),
            }
        rate_counts = Counter(joint for frame in self.frames for joint in frame.rate_limited_joints)
        limit_counts = Counter(joint for frame in self.frames for joint in frame.near_limit_joints)
        return {
            "frame_count": len(self.frames),
            "tasks": tasks,
            "rate_limited_frame_count": sum(bool(frame.rate_limited_joints) for frame in self.frames),
            "near_limit_frame_count": sum(bool(frame.near_limit_joints) for frame in self.frames),
            "rate_limited_joint_counts": dict(sorted(rate_counts.items())),
            "near_limit_joint_counts": dict(sorted(limit_counts.items())),
        }

    def write_json(self, path: str | Path) -> Path:
        path = _prepare_path(path)
        frames = []
        for frame in self.frames:
            tasks = {
                name: {
                    "position_error_cm": frame.position_errors_m[name] * 100.0,
                    "orientation_error_deg": math.degrees(frame.orientation_errors_rad[name]),
                }
                for name in self.task_names
            }
            frames.append(
                {
                    "source_frame": frame.source_frame,
                    "time_seconds": frame.time_seconds,
                    "tasks": tasks,
                    "rate_limited_joints": list(frame.rate_limited_joints),
                    "near_limit_joints": list(frame.near_limit_joints),
                }
            )
        path.write_text(json.dumps({"summary": self.summary(), "frames": frames}, indent=2) + "\n", encoding="utf-8")
        return path

    def write_csv(self, path: str | Path) -> Path:
        path = _prepare_path(path)
        task_fields = [
            field
            for name in self.task_names
            for field in (f"{name}_position_error_cm", f"{name}_orientation_error_deg")
        ]
        fieldnames = [
            "source_frame",
            "time_seconds",
            *task_fields,
            "rate_limited_joint_count",
            "near_limit_joint_count",
            "rate_limited_joints",
            "near_limit_joints",
        ]
        with path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=fieldnames)
            writer.writeheader()
            for frame in self.frames:
                row: dict[str, object] = {
                    "source_frame": frame.source_frame,
                    "time_seconds": frame.time_seconds,
                    "rate_limited_joint_count": len(frame.rate_limited_joints),
                    "near_limit_joint_count": len(frame.near_limit_joints),
                    "rate_limited_joints": ";".join(frame.rate_limited_joints),
                    "near_limit_joints": ";".join(frame.near_limit_joints),
                }
                for name in self.task_names:
                    row[f"{name}_position_error_cm"] = frame.position_errors_m[name] * 100.0
                    row[f"{name}_orientation_error_deg"] = math.degrees(frame.orientation_errors_rad[name])
                writer.writerow(row)
        return path


def _rms(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else 0.0


def _prepare_path(path: str | Path) -> Path:
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    return path
