"""Simulation-only process and recording services for the SONIC GUI."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import shutil
import shlex
import subprocess
import sys

from .sonic_stream import SonicStreamError, load_trajectory, pack_protocol_v1


ROOT = Path(__file__).resolve().parent.parent
STACK_NAME = "sonic-g1-mujoco"
SESSION_NAME = "retarget-sonic-g1"
ISAAC_SESSION_NAME = "retarget-isaac-g1"
GUI_PREROLL_FRAMES = 100
GUI_PREROLL_SECONDS = GUI_PREROLL_FRAMES / 50.0


class ConsoleError(RuntimeError):
    """Raised when the local simulation console cannot complete an action."""


@dataclass(frozen=True)
class Recording:
    name: str
    directory: Path
    frame_count: int
    duration_seconds: float
    source: Path | None


@dataclass(frozen=True)
class StackState:
    session_exists: bool
    running: bool
    mujoco_running: bool
    sonic_running: bool
    sonic_initialized: bool
    control_started: bool
    streaming_enabled: bool


@dataclass(frozen=True)
class IsaacState:
    session_exists: bool
    running: bool
    log: str


def discover_recordings(root: str | Path) -> list[Recording]:
    root = Path(root).expanduser().resolve()
    recordings = []
    for metadata_path in sorted(
        root.glob("*/sonic_reference/trajectory_metadata.json"),
        key=lambda path: path.parent.parent.name.casefold(),
    ):
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ConsoleError(f"cannot read {metadata_path}: {error}") from error
        frame_count = metadata.get("frame_count")
        if metadata.get("format") != "sonic-g1-reference-v1":
            raise ConsoleError(f"unsupported reference format: {metadata_path}")
        if metadata.get("rate_hz") != 50 or not isinstance(frame_count, int) or frame_count <= 0:
            raise ConsoleError(f"invalid 50 Hz reference metadata: {metadata_path}")
        recordings.append(
            Recording(
                name=metadata_path.parent.parent.name,
                directory=metadata_path.parent,
                frame_count=frame_count,
                duration_seconds=(frame_count - 1) / 50.0,
                source=Path(metadata["source"]).expanduser().resolve()
                if isinstance(metadata.get("source"), str)
                else None,
            )
        )
    if not recordings:
        raise ConsoleError(f"no SONIC references found under: {root}")
    return recordings


def pack_recording(recording: Recording, preroll_frames: int = GUI_PREROLL_FRAMES) -> bytes:
    try:
        joint_pos, joint_vel, body_quat = load_trajectory(recording.directory)
        if preroll_frames < 0:
            raise ConsoleError("preroll_frames cannot be negative")
        if preroll_frames:
            joint_pos = [joint_pos[0][:] for _ in range(preroll_frames)] + joint_pos
            joint_vel = [[0.0] * len(joint_vel[0]) for _ in range(preroll_frames)] + joint_vel
            body_quat = [body_quat[0][:] for _ in range(preroll_frames)] + body_quat
        return pack_protocol_v1(joint_pos, joint_vel, body_quat)
    except SonicStreamError as error:
        raise ConsoleError(str(error)) from error


def parse_stack_state(running: bool, panes: str, sonic_log: str, mujoco_log: str = "") -> StackState:
    pane_lines = panes.splitlines()
    mujoco_running = (
        running
        and "[retargetctl] process exited" not in mujoco_log
        and any(line.startswith("mujoco:") and line.endswith(":0") for line in pane_lines)
    )
    sonic_running = (
        running
        and "[retargetctl] process exited" not in sonic_log
        and any(line.startswith("sonic:") and line.endswith(":0") for line in pane_lines)
    )
    enabled_at = sonic_log.rfind("ZMQ STREAMING MODE: ENABLED")
    disabled_at = max(
        sonic_log.rfind("ZMQ STREAMING MODE: DISABLED"),
        sonic_log.rfind("ZMQ STREAMING MODE: FORCE DISABLED"),
    )
    return StackState(
        session_exists=running,
        running=mujoco_running and sonic_running,
        mujoco_running=mujoco_running,
        sonic_running=sonic_running,
        sonic_initialized=sonic_running and "Init Done" in sonic_log,
        control_started=sonic_running and "transitioning to CONTROL state" in sonic_log,
        streaming_enabled=sonic_running and enabled_at >= 0 and enabled_at > disabled_at,
    )


class StackController:
    def __init__(self) -> None:
        bundled_tmux = ROOT.parent / "sonic-build-deps" / "bin" / "tmux"
        self.tmux = str(bundled_tmux if bundled_tmux.exists() else shutil.which("tmux") or "tmux")
        self.environment = os.environ.copy()
        self.environment["PATH"] = f"{Path(self.tmux).parent}:{self.environment.get('PATH', '')}"
        bundled_terminfo = Path(self.tmux).parent.parent / "share" / "terminfo"
        if bundled_terminfo.is_dir():
            # Ghostty's custom TERM entry is not present in the isolated tmux prefix.
            self.environment["TERM"] = "xterm-256color"
            self.environment["TERMINFO"] = str(bundled_terminfo)
            self.environment["TERMINFO_DIRS"] = str(bundled_terminfo)

    def state(self) -> StackState:
        running = self._tmux("has-session", "-t", SESSION_NAME, check=False).returncode == 0
        if not running:
            return parse_stack_state(False, "", "")
        panes = self._tmux(
            "list-panes",
            "-s",
            "-t",
            SESSION_NAME,
            "-F",
            "#{window_name}:#{pane_current_command}:#{pane_dead}",
        ).stdout
        sonic_log = self.capture_log("sonic", 2000)
        mujoco_log = self.capture_log("mujoco", 2000)
        return parse_stack_state(True, panes, sonic_log, mujoco_log)

    def start(self) -> str:
        state = self.state()
        if state.running:
            return "SONIC simulation is already running"
        if state.session_exists:
            self._tmux("kill-session", "-t", SESSION_NAME)
        return self._run_retargetctl("start", STACK_NAME, "--backend", "tmux")

    def stop(self) -> str:
        if not self.state().session_exists:
            return "SONIC simulation is already stopped"
        return self._run_retargetctl("stop", STACK_NAME)

    def arm_stream(self) -> None:
        state = self.state()
        if not state.running or not state.sonic_initialized:
            raise ConsoleError("SONIC must reach 'Init Done' before its stream can be armed")
        if not state.streaming_enabled:
            self.send_key("sonic", "Enter")

    def start_control(self) -> None:
        state = self.state()
        if not state.running or not state.streaming_enabled:
            raise ConsoleError("arm the SONIC stream and publish a reference before starting control")
        if not state.control_started:
            self.send_key("sonic", "]")

    def release_support(self) -> None:
        state = self.state()
        if not state.running or not state.control_started:
            raise ConsoleError("SONIC control must be active before releasing MuJoCo support")
        xdotool = ROOT.parent / "sonic-build-deps" / "bin" / "xdotool"
        if not xdotool.is_file():
            raise ConsoleError(f"xdotool is required to release MuJoCo support: {xdotool}")
        search = subprocess.run(
            [str(xdotool), "search", "--onlyvisible", "--class", "mujoco"],
            env=self.environment,
            text=True,
            capture_output=True,
        )
        window_ids = list(dict.fromkeys(search.stdout.split()))
        if search.returncode or len(window_ids) != 1:
            raise ConsoleError(
                f"expected one visible MuJoCo viewer, found {len(window_ids)}; support remains enabled"
            )
        result = subprocess.run(
            [str(xdotool), "key", "--window", window_ids[0], "9"],
            env=self.environment,
            text=True,
            capture_output=True,
        )
        if result.returncode:
            raise ConsoleError(result.stderr.strip() or "could not release MuJoCo elastic support")

    def emergency_stop(self) -> None:
        if self.state().session_exists:
            self.send_key("sonic", "O")

    def send_key(self, window: str, key: str) -> None:
        result = self._tmux("send-keys", "-t", f"{SESSION_NAME}:{window}", key, check=False)
        if result.returncode:
            raise ConsoleError(result.stderr.strip() or f"cannot send {key!r} to {window}")

    def capture_log(self, window: str, lines: int = 120) -> str:
        result = self._tmux(
            "capture-pane",
            "-p",
            "-t",
            f"{SESSION_NAME}:{window}",
            "-S",
            f"-{lines}",
            check=False,
        )
        return result.stdout if result.returncode == 0 else result.stderr

    def _run_retargetctl(self, *arguments: str) -> str:
        result = subprocess.run(
            [sys.executable, str(ROOT / "retargetctl.py"), *arguments],
            cwd=ROOT,
            env=self.environment,
            text=True,
            capture_output=True,
        )
        output = "\n".join(part.strip() for part in (result.stdout, result.stderr) if part.strip())
        if result.returncode:
            raise ConsoleError(output or f"retargetctl exited with status {result.returncode}")
        return output

    def _tmux(self, *arguments: str, check: bool = True) -> subprocess.CompletedProcess[str]:
        try:
            return subprocess.run(
                [self.tmux, *arguments],
                env=self.environment,
                text=True,
                capture_output=True,
                check=check,
            )
        except (OSError, subprocess.CalledProcessError) as error:
            stderr = getattr(error, "stderr", "")
            raise ConsoleError(stderr.strip() or str(error)) from error


class IsaacController:
    """Run kinematic MVNX playback in an isolated Isaac Sim tmux session."""

    def __init__(self) -> None:
        self.commands = StackController()

    def state(self) -> IsaacState:
        result = self.commands._tmux("has-session", "-t", ISAAC_SESSION_NAME, check=False)
        if result.returncode:
            return IsaacState(False, False, "")
        log = self.commands._tmux(
            "capture-pane",
            "-p",
            "-t",
            f"{ISAAC_SESSION_NAME}:isaac",
            "-S",
            "-200",
            check=False,
        ).stdout
        return IsaacState(True, "[isaac] process exited" not in log, log)

    def start(self, recording: Recording) -> str:
        launcher = ROOT / "run_isaac_g1_playback.sh"
        isaac_python = Path(os.environ.get("ISAACSIM_PYTHON", Path.home() / "isaacsim" / "python.sh"))
        if not launcher.is_file():
            raise ConsoleError(f"Isaac launcher not found: {launcher}")
        if not isaac_python.is_file():
            raise ConsoleError(f"Isaac Sim Python launcher not found: {isaac_python}")
        if recording.source is None or not recording.source.is_file():
            raise ConsoleError(f"source MVNX is unavailable for {recording.name}")
        if self.state().session_exists:
            self.stop()

        command = (
            f"cd {shlex.quote(str(ROOT))} && "
            f"{shlex.quote(str(launcher))} --mvnx {shlex.quote(str(recording.source))} --loop; "
            "rc=$?; printf '\n[isaac] process exited with status %s\n' \"$rc\"; exec bash"
        )
        result = self.commands._tmux(
            "new-session",
            "-d",
            "-s",
            ISAAC_SESSION_NAME,
            "-n",
            "isaac",
            "bash",
            "-lc",
            command,
            check=False,
        )
        if result.returncode:
            raise ConsoleError(result.stderr.strip() or "could not start Isaac Sim")
        return f"Isaac Sim launching {recording.name} in {ISAAC_SESSION_NAME}"

    def stop(self) -> str:
        if not self.state().session_exists:
            return "Isaac Sim is already stopped"
        result = self.commands._tmux("kill-session", "-t", ISAAC_SESSION_NAME, check=False)
        if result.returncode:
            raise ConsoleError(result.stderr.strip() or "could not stop Isaac Sim")
        return "Isaac Sim stopped"
