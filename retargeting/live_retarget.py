"""Continuously retarget canonical live Xsens snapshots to SONIC in simulation."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time

from .live_frame import (
    CANONICAL_SEGMENTS,
    LIVE_STALE_MS,
    LiveFrame,
    LiveFrameError,
    counter_advanced,
    decode_live_frame_json,
)
from .sonic_stream import pack_protocol_v1
from .paths import projects_root


@dataclass
class LiveWatchdog:
    stale_seconds: float = LIVE_STALE_MS / 1000.0
    last_counter: int | None = None
    last_unique_ns: int = 0

    def accept(self, frame: LiveFrame, now_ns: int) -> bool:
        if not counter_advanced(self.last_counter, frame.source_sample_counter):
            return False
        self.last_counter = frame.source_sample_counter
        self.last_unique_ns = now_ns
        return True

    def stale(self, now_ns: int) -> bool:
        return self.last_unique_ns == 0 or (now_ns - self.last_unique_ns) / 1e9 > self.stale_seconds


def read_control(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {"calibration_generation": 0, "enabled": False}
    generation = value.get("calibration_generation", 0)
    enabled = value.get("enabled", False)
    if isinstance(generation, bool) or not isinstance(generation, int) or generation < 0:
        generation = 0
    return {"calibration_generation": generation, "enabled": enabled is True}


def write_status(path: Path, **values) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(values, separators=(",", ":")), encoding="utf-8")
    os.replace(temporary, path)


def emergency_stop_sonic(session: str = "retarget-sonic-g1") -> bool:
    bundled_tmux = projects_root() / "sonic-build-deps/bin/tmux"
    tmux = str(bundled_tmux) if bundled_tmux.is_file() else shutil.which("tmux")
    if tmux is None:
        return False
    result = subprocess.run(
        [tmux, "send-keys", "-t", f"{session}:sonic", "O"],
        text=True,
        capture_output=True,
    )
    return result.returncode == 0


def load_snapshot(path: Path) -> LiveFrame:
    try:
        return decode_live_frame_json(path.read_bytes())
    except OSError as error:
        raise LiveFrameError(f"cannot read live snapshot: {error}") from error


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path("/tmp/xsens-live-frame.json"))
    parser.add_argument("--bind", default="tcp://127.0.0.1:5556")
    parser.add_argument(
        "--urdf",
        type=Path,
        default=projects_root() / "unitree_rl_gym/resources/robots/g1_description/g1_29dof.urdf",
    )
    parser.add_argument("--stale-ms", type=float, default=LIVE_STALE_MS)
    parser.add_argument("--window", type=int, default=100)
    parser.add_argument("--warmup", type=int, default=50)
    parser.add_argument("--control", type=Path, default=Path("/tmp/xsens-live-control.json"))
    parser.add_argument("--status", type=Path, default=Path("/tmp/xsens-live-status.json"))
    args = parser.parse_args(argv)
    if args.stale_ms <= 0 or args.warmup < 2 or args.window < args.warmup:
        parser.error("stale-ms must be positive and window must be at least warmup >= 2")
    try:
        import zmq
        from .g1_retargeter import G1Retargeter, mvnx_from_live
    except ImportError as error:
        print(f"error: Pink, Pinocchio, and pyzmq are required: {error}", file=sys.stderr)
        return 2

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(args.bind)
    watchdog = LiveWatchdog(args.stale_ms / 1000.0)
    retargeter = None
    # A process restart must not reinterpret a persisted request as a new
    # calibration. The operator must increment the generation explicitly.
    calibrated_generation = read_control(args.control)["calibration_generation"]
    stale_stop_sent = False
    frame_index = 0
    published_windows = 0
    last_publish_steady_ns = 0
    positions = []
    velocities = []
    quaternions = []
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    print(f"Waiting for freshness-aware Xsens snapshots at {args.input}", flush=True)
    try:
        while running:
            started = time.monotonic()
            now_ns = time.monotonic_ns()
            control = read_control(args.control)
            state = "NO_DATA"
            age_ms = None
            try:
                live = load_snapshot(args.input)
                age_ms = live.age_seconds(now_ns) * 1000
                if age_ms > args.stale_ms:
                    raise LiveFrameError(f"capture snapshot is stale ({age_ms:.0f} ms)")
                state = (
                    "ENABLED" if control["enabled"] else "CALIBRATED_DISABLED"
                ) if retargeter is not None else "WAITING_FOR_CALIBRATION"
                if watchdog.accept(live, now_ns):
                    stale_stop_sent = False
                    frame = mvnx_from_live(live, frame_index)
                    if control["calibration_generation"] > calibrated_generation:
                        retargeter = G1Retargeter(args.urdf, CANONICAL_SEGMENTS, frame)
                        calibrated_generation = control["calibration_generation"]
                        positions.clear()
                        velocities.clear()
                        quaternions.clear()
                        print(f"Calibrated generation {calibrated_generation}", flush=True)
                    if retargeter is not None:
                        if control["enabled"]:
                            joint_pos, joint_vel = retargeter.solve(frame)
                            positions.append(joint_pos)
                            velocities.append(joint_vel)
                            quaternions.append(list(retargeter.root_quaternion(frame)))
                            positions = positions[-args.window :]
                            velocities = velocities[-args.window :]
                            quaternions = quaternions[-args.window :]
                            if len(positions) >= args.warmup:
                                indexes = list(range(frame_index - len(positions) + 1, frame_index + 1))
                                socket.send(
                                    pack_protocol_v1(
                                        positions, velocities, quaternions, frame_indices=indexes, catch_up=True
                                    )
                                )
                                published_windows += 1
                                last_publish_steady_ns = time.monotonic_ns()
                            frame_index += 1
                    else:
                        state = "WAITING_FOR_CALIBRATION"
            except LiveFrameError as error:
                state = "STALE" if watchdog.last_counter is not None else "NO_DATA"
            if watchdog.stale(now_ns) and watchdog.last_counter is not None:
                print("STALE: unique Xsens input stopped; SONIC publication halted", flush=True)
                if control["enabled"] and not stale_stop_sent:
                    emergency_stop_sonic()
                    stale_stop_sent = True
                    print("STALE: SONIC emergency stop requested", flush=True)
                watchdog.last_counter = None
                retargeter = None
                # A stale stop requires an explicit new calibration generation.
                calibrated_generation = control["calibration_generation"]
                positions.clear()
                velocities.clear()
                quaternions.clear()
                state = "STALE"
            write_status(
                args.status,
                state=state,
                age_ms=age_ms,
                source_sample_counter=watchdog.last_counter,
                calibrated_generation=calibrated_generation,
                enabled=control["enabled"],
                published_windows=published_windows,
                buffered_frames=len(positions),
                last_publish_steady_ns=last_publish_steady_ns,
            )
            time.sleep(max(0.0, 0.02 - (time.monotonic() - started)))
    finally:
        socket.close()
        context.term()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
