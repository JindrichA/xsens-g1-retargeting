"""Canonical live Xsens frame validation shared across environment boundaries."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import time
from typing import Mapping, Sequence

from .xsens_skeleton import SEGMENT_EDGES


LIVE_FRAME_TOPIC = b"xsens_frame"
LIVE_STALE_MS = 250.0
CANONICAL_SEGMENTS = tuple(dict.fromkeys(name for edge in SEGMENT_EDGES for name in edge))


class LiveFrameError(ValueError):
    """Raised when a live Xsens frame violates the canonical contract."""


@dataclass(frozen=True)
class LiveSegment:
    position: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True)
class LiveFrame:
    source_sample_counter: int
    source_frame_time_ms: int
    source_receive_steady_ns: int
    capture_receive_steady_ns: int
    segments: Mapping[str, LiveSegment]

    def age_seconds(self, now_ns: int | None = None) -> float:
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        return max(0.0, (now_ns - self.capture_receive_steady_ns) / 1e9)


def encode_live_frame(frame: LiveFrame) -> bytes:
    validate_live_frame(frame)
    payload = {
        "version": 1,
        "source_sample_counter": frame.source_sample_counter,
        "source_frame_time_ms": frame.source_frame_time_ms,
        "source_receive_steady_ns": frame.source_receive_steady_ns,
        "capture_receive_steady_ns": frame.capture_receive_steady_ns,
        "segments": {
            name: {
                "position": list(frame.segments[name].position),
                "quaternion_wxyz": list(frame.segments[name].quaternion_wxyz),
            }
            for name in CANONICAL_SEGMENTS
        },
    }
    return LIVE_FRAME_TOPIC + json.dumps(payload, separators=(",", ":")).encode("ascii")


def encode_live_frame_json(frame: LiveFrame) -> bytes:
    return encode_live_frame(frame)[len(LIVE_FRAME_TOPIC) :]


def decode_live_frame(message: bytes) -> LiveFrame:
    if not message.startswith(LIVE_FRAME_TOPIC):
        raise LiveFrameError("unexpected live frame topic")
    try:
        payload = json.loads(message[len(LIVE_FRAME_TOPIC) :])
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise LiveFrameError(f"invalid live frame JSON: {error}") from error
    if payload.get("version") != 1:
        raise LiveFrameError("unsupported live frame version")
    raw_segments = payload.get("segments")
    if not isinstance(raw_segments, dict):
        raise LiveFrameError("live frame segments must be an object")
    segments = {}
    try:
        for name in CANONICAL_SEGMENTS:
            raw = raw_segments[name]
            segments[name] = LiveSegment(
                position=_finite_tuple(raw["position"], 3, f"{name} position"),
                quaternion_wxyz=_finite_tuple(raw["quaternion_wxyz"], 4, f"{name} quaternion"),
            )
        frame = LiveFrame(
            source_sample_counter=_integer(payload, "source_sample_counter"),
            source_frame_time_ms=_integer(payload, "source_frame_time_ms"),
            source_receive_steady_ns=_integer(payload, "source_receive_steady_ns"),
            capture_receive_steady_ns=_integer(payload, "capture_receive_steady_ns"),
            segments=segments,
        )
    except (KeyError, TypeError) as error:
        raise LiveFrameError(f"missing live frame field: {error}") from error
    validate_live_frame(frame)
    return frame


def decode_live_frame_json(message: bytes) -> LiveFrame:
    return decode_live_frame(LIVE_FRAME_TOPIC + message)


def validate_live_frame(frame: LiveFrame) -> None:
    missing = sorted(set(CANONICAL_SEGMENTS) - frame.segments.keys())
    if missing:
        raise LiveFrameError(f"missing Xsens segments: {', '.join(missing)}")
    if frame.source_receive_steady_ns <= 0 or frame.capture_receive_steady_ns <= 0:
        raise LiveFrameError("live frame receive timestamps must be positive")
    for name in CANONICAL_SEGMENTS:
        segment = frame.segments[name]
        _finite_tuple(segment.position, 3, f"{name} position")
        quaternion = _finite_tuple(segment.quaternion_wxyz, 4, f"{name} quaternion")
        norm = math.sqrt(sum(value * value for value in quaternion))
        if not 0.99 <= norm <= 1.01:
            raise LiveFrameError(f"{name} quaternion is not normalized")


def counter_advanced(previous: int | None, current: int) -> bool:
    """Return true for a newer uint32 counter, including wraparound."""
    if previous is None:
        return True
    delta = (current - previous) & 0xFFFFFFFF
    return 0 < delta < 0x80000000


def _integer(payload: dict, name: str) -> int:
    value = payload[name]
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xFFFFFFFFFFFFFFFF:
        raise LiveFrameError(f"{name} must be a non-negative integer")
    return value


def _finite_tuple(values: Sequence[float], width: int, name: str) -> tuple:
    if not isinstance(values, (list, tuple)) or len(values) != width:
        raise LiveFrameError(f"{name} must contain {width} values")
    converted = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in converted):
        raise LiveFrameError(f"{name} contains a non-finite value")
    return converted
