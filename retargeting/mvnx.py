"""Dependency-free, streaming reader for Xsens MVNX motion exports."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import xml.etree.ElementTree as ET
from typing import Iterator


class MvnxError(ValueError):
    pass


@dataclass(frozen=True)
class MvnxMetadata:
    path: Path
    frame_rate: int
    segment_names: tuple[str, ...]
    joint_names: tuple[str, ...]
    sensor_names: tuple[str, ...]
    contact_names: tuple[str, ...]
    finger_joint_count: int
    configuration: str
    processing_quality: str
    recorded_at: str


@dataclass(frozen=True)
class MvnxFrame:
    index: int
    time_seconds: float
    source_time_ms: int | None
    segment_quaternions: tuple[tuple[float, float, float, float], ...]
    segment_positions: tuple[tuple[float, float, float], ...]
    joint_angles_zxy: tuple[tuple[float, float, float], ...]
    foot_contacts: tuple[int, ...]
    center_of_mass: tuple[float, ...]


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _groups(text: str | None, width: int, field: str) -> tuple[tuple[float, ...], ...]:
    values = _values(text, field)
    if len(values) % width:
        raise MvnxError(f"{field} contains {len(values)} values; expected groups of {width}")
    return tuple(tuple(values[offset : offset + width]) for offset in range(0, len(values), width))


def _values(text: str | None, field: str) -> tuple[float, ...]:
    if not text or not text.strip():
        raise MvnxError(f"missing values for {field}")
    try:
        values = tuple(float(value) for value in text.split())
    except ValueError as exc:
        raise MvnxError(f"invalid numeric value in {field}") from exc
    if not all(math.isfinite(value) for value in values):
        raise MvnxError(f"non-finite value in {field}")
    return values


def read_metadata(path: str | Path) -> MvnxMetadata:
    path = Path(path)
    subject: dict[str, str] | None = None
    frame_attributes: dict[str, str] = {}
    segments: list[str] = []
    joints: list[str] = []
    sensors: list[str] = []
    contacts: list[str] = []
    stack: list[str] = []

    try:
        for event, element in ET.iterparse(path, events=("start", "end")):
            tag = _local_name(element.tag)
            if event == "start":
                parent = stack[-1] if stack else ""
                stack.append(tag)
                if tag == "subject":
                    subject = dict(element.attrib)
                elif parent == "segments" and tag == "segment":
                    segments.append(element.attrib["label"])
                elif parent == "joints" and tag == "joint":
                    joints.append(element.attrib["label"])
                elif parent == "sensors" and tag == "sensor":
                    sensors.append(element.attrib["label"])
                elif parent == "footContactDefinition" and tag == "contactDefinition":
                    contacts.append(element.attrib["label"])
                elif tag == "frames":
                    frame_attributes = dict(element.attrib)
                    break
            else:
                stack.pop()
    except (ET.ParseError, OSError, KeyError) as exc:
        raise MvnxError(f"cannot read MVNX metadata from {path}: {exc}") from exc

    if subject is None or not segments or not frame_attributes:
        raise MvnxError(f"incomplete MVNX metadata in {path}")
    try:
        frame_rate = int(subject["frameRate"])
        finger_joint_count = int(frame_attributes.get("fingerJointCount", "0"))
    except (KeyError, ValueError) as exc:
        raise MvnxError(f"invalid frame metadata in {path}") from exc
    if frame_rate <= 0:
        raise MvnxError(f"invalid frame rate {frame_rate} in {path}")

    return MvnxMetadata(
        path=path,
        frame_rate=frame_rate,
        segment_names=tuple(segments),
        joint_names=tuple(joints),
        sensor_names=tuple(sensors),
        contact_names=tuple(contacts),
        finger_joint_count=finger_joint_count,
        configuration=subject.get("configuration", ""),
        processing_quality=subject.get("processingQuality", ""),
        recorded_at=subject.get("recDate", ""),
    )


def iter_frames(path: str | Path, metadata: MvnxMetadata | None = None) -> Iterator[MvnxFrame]:
    path = Path(path)
    metadata = metadata or read_metadata(path)
    expected_index = 0
    try:
        for _, element in ET.iterparse(path, events=("end",)):
            if _local_name(element.tag) != "frame" or element.attrib.get("type") != "normal":
                continue
            try:
                index = int(element.attrib["index"])
            except (KeyError, ValueError) as exc:
                raise MvnxError("normal frame has no valid index") from exc
            if index != expected_index:
                raise MvnxError(f"non-contiguous frame index: got {index}, expected {expected_index}")

            fields = {_local_name(child.tag): child.text for child in element}
            quaternions = _groups(fields.get("orientation"), 4, "orientation")
            positions = _groups(fields.get("position"), 3, "position")
            joint_angles = _groups(fields.get("jointAngle"), 3, "jointAngle")
            contacts = tuple(int(value) for value in _values(fields.get("footContacts"), "footContacts"))
            center_of_mass = _values(fields.get("centerOfMass"), "centerOfMass")

            if len(quaternions) != len(metadata.segment_names):
                raise MvnxError(f"frame {index}: expected {len(metadata.segment_names)} segment quaternions")
            if len(positions) != len(metadata.segment_names):
                raise MvnxError(f"frame {index}: expected {len(metadata.segment_names)} segment positions")
            if len(joint_angles) != len(metadata.joint_names):
                raise MvnxError(f"frame {index}: expected {len(metadata.joint_names)} joint angles")
            if len(contacts) != len(metadata.contact_names):
                raise MvnxError(f"frame {index}: expected {len(metadata.contact_names)} foot contacts")
            if len(center_of_mass) != 9:
                raise MvnxError(f"frame {index}: centerOfMass must contain 9 values")

            source_time = element.attrib.get("time")
            yield MvnxFrame(
                index=index,
                time_seconds=index / metadata.frame_rate,
                source_time_ms=int(source_time) if source_time is not None else None,
                segment_quaternions=quaternions,
                segment_positions=positions,
                joint_angles_zxy=joint_angles,
                foot_contacts=contacts,
                center_of_mass=center_of_mass,
            )
            expected_index += 1
            element.clear()
    except (ET.ParseError, OSError, ValueError) as exc:
        if isinstance(exc, MvnxError):
            raise
        raise MvnxError(f"cannot parse MVNX frames from {path}: {exc}") from exc


def validate(path: str | Path) -> dict[str, int | float]:
    metadata = read_metadata(path)
    frame_count = 0
    timestamp_anomalies = 0
    quaternion_norm_warnings = 0
    previous_source_time: int | None = None
    for frame in iter_frames(path, metadata):
        frame_count += 1
        if previous_source_time is not None and frame.source_time_ms is not None:
            delta = frame.source_time_ms - previous_source_time
            if delta not in {4, 5}:
                timestamp_anomalies += 1
        previous_source_time = frame.source_time_ms
        for quaternion in frame.segment_quaternions:
            norm = math.sqrt(sum(value * value for value in quaternion))
            if abs(norm - 1.0) > 1e-3:
                quaternion_norm_warnings += 1
    if frame_count == 0:
        raise MvnxError(f"no normal motion frames in {path}")
    return {
        "frames": frame_count,
        "duration_seconds": (frame_count - 1) / metadata.frame_rate,
        "timestamp_anomalies": timestamp_anomalies,
        "quaternion_norm_warnings": quaternion_norm_warnings,
    }
