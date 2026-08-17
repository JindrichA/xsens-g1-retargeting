"""PyQt6 control console for Xsens references in SONIC MuJoCo simulation."""

from __future__ import annotations

import argparse
import ctypes
import json
import os
from pathlib import Path
import sys
import time
from typing import Sequence


# PyPI's Qt wheel expects this X11 helper from the host. A Conda-installed copy
# is outside the wheel's rpath, so preload it before Qt opens the xcb plugin.
if sys.platform.startswith("linux"):
    xcb_cursor = Path(sys.prefix) / "lib" / "libxcb-cursor.so.0"
    if xcb_cursor.exists():
        ctypes.CDLL(xcb_cursor, mode=ctypes.RTLD_GLOBAL)

try:
    import zmq
    from PyQt6.QtCore import QProcess, QTimer, Qt
    from PyQt6.QtGui import QFont
    from PyQt6.QtWidgets import (
        QApplication,
        QFileDialog,
        QFrame,
        QHBoxLayout,
        QHeaderView,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPlainTextEdit,
        QProgressBar,
        QPushButton,
        QSplitter,
        QTableWidget,
        QTableWidgetItem,
        QVBoxLayout,
        QWidget,
    )
except ImportError as error:
    raise SystemExit(
        "PyQt6 and pyzmq are required; install requirements-sonic-gui.txt in the GUI environment"
    ) from error

from .sonic_console import (
    ConsoleError,
    GUI_PREROLL_SECONDS,
    GUI_SUPPORTED_CONTROL_SECONDS,
    IsaacController,
    LivePipelineController,
    Recording,
    StackController,
    StackState,
    discover_recordings,
    import_mvnx_command,
    live_next_action,
    pack_recording,
)


BIND_ENDPOINT = "tcp://127.0.0.1:5556"


class SonicPublisher:
    def __init__(self, bind: str = BIND_ENDPOINT) -> None:
        self.bind = bind
        self.context = None
        self.socket = None

    def _open(self) -> None:
        if self.socket is not None:
            return
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        try:
            self.socket.bind(self.bind)
            time.sleep(0.5)
        except zmq.ZMQError as error:
            self.socket.close()
            self.context.term()
            self.socket = None
            self.context = None
            raise ConsoleError(f"cannot bind {self.bind}; another trajectory publisher may be running: {error}") from error

    def send(self, message: bytes) -> None:
        self._open()
        try:
            self.socket.send(message)
        except zmq.ZMQError as error:
            raise ConsoleError(f"trajectory publish failed: {error}") from error

    def close(self) -> None:
        if self.socket is not None:
            self.socket.close()
            self.context.term()
            self.socket = None
            self.context = None


LIVE_CONTROL = Path("/tmp/xsens-live-control.json")
LIVE_STATUS = Path("/tmp/xsens-live-status.json")


def write_live_control(calibration_generation: int, enabled: bool) -> None:
    temporary = LIVE_CONTROL.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps({"calibration_generation": calibration_generation, "enabled": enabled}),
        encoding="utf-8",
    )
    os.replace(temporary, LIVE_CONTROL)


class MainWindow(QMainWindow):
    def __init__(self, recording_root: Path) -> None:
        super().__init__()
        self.recording_root = recording_root
        self.stack = StackController()
        self.isaac = IsaacController()
        self.live_pipeline = LivePipelineController()
        existing_control = self._read_live_control()
        self.live_generation = existing_control.get("calibration_generation", 0)
        self.live_enable_deadline = 0.0
        self.live_publish_baseline = 0
        self.recordings = discover_recordings(recording_root)
        self.publisher = SonicPublisher()
        self.last_state: StackState | None = None
        self.isaac_running = False
        self.queue: list[int] = []
        self.current_recording: Recording | None = None
        self.motion_started_at = 0.0
        self.motion_ends_at = 0.0
        self.control_start_deadline = 0.0
        self.import_process: QProcess | None = None
        self.import_output = ""

        self.queue_timer = QTimer(self)
        self.queue_timer.setSingleShot(True)
        self.queue_timer.timeout.connect(self._play_next)
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.refresh_status)
        self.status_timer.start(1000)
        self.progress_timer = QTimer(self)
        self.progress_timer.timeout.connect(self._refresh_progress)
        self.progress_timer.start(100)

        self.setWindowTitle("Xsens / SONIC Simulation Console")
        self.setMinimumSize(1080, 700)
        self.resize(1280, 790)
        self._build_ui()
        self._apply_style()
        self.refresh_status()
        self._append_log(
            f"Ready: {len(self.recordings)} validated references from {self.recording_root}\n"
            f"Publisher bound to {BIND_ENDPOINT} (simulation loopback only)."
        )

    def _build_ui(self) -> None:
        root = QWidget()
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(24, 20, 24, 20)
        root_layout.setSpacing(16)

        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title = QLabel("XSENS / SONIC")
        title.setObjectName("title")
        subtitle = QLabel("G1 dynamic simulation console  /  loopback profile")
        subtitle.setObjectName("subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box)
        header.addStretch()
        self.stack_badge = QLabel("STACK OFFLINE")
        self.stack_badge.setObjectName("badge")
        self.control_badge = QLabel("CONTROL IDLE")
        self.control_badge.setObjectName("badge")
        self.stream_badge = QLabel("STREAM OFFLINE")
        self.stream_badge.setObjectName("badge")
        self.isaac_badge = QLabel("ISAAC OFFLINE")
        self.isaac_badge.setObjectName("badge")
        self.live_badge = QLabel("LIVE OFFLINE")
        self.live_badge.setObjectName("badge")
        header.addWidget(self.stack_badge)
        header.addWidget(self.control_badge)
        header.addWidget(self.stream_badge)
        header.addWidget(self.isaac_badge)
        header.addWidget(self.live_badge)
        root_layout.addLayout(header)

        safety = QLabel(
            "SIMULATION ONLY  |  References pass through the SONIC policy before MuJoCo. "
            "The native MuJoCo viewer is the visual display."
        )
        safety.setObjectName("safety")
        safety.setWordWrap(True)
        root_layout.addWidget(safety)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self._build_recordings_panel())
        splitter.addWidget(self._build_control_panel())
        splitter.addWidget(self._build_log_panel())
        splitter.setSizes([390, 360, 440])
        root_layout.addWidget(splitter, 1)
        self.setCentralWidget(root)

    def _panel(self, heading: str, caption: str) -> tuple[QFrame, QVBoxLayout]:
        frame = QFrame()
        frame.setObjectName("panel")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(12)
        label = QLabel(heading)
        label.setObjectName("panelTitle")
        detail = QLabel(caption)
        detail.setObjectName("caption")
        detail.setWordWrap(True)
        layout.addWidget(label)
        layout.addWidget(detail)
        return frame, layout

    def _build_recordings_panel(self) -> QFrame:
        panel, layout = self._panel("RECORDINGS", "Validated 50 Hz references in alphabetical order")
        self.recording_table = QTableWidget(0, 2)
        self.recording_table.setHorizontalHeaderLabels(["Recording", "Duration"])
        self.recording_table.verticalHeader().setVisible(False)
        self.recording_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.recording_table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.recording_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.recording_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.recording_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        self._populate_recordings()
        self.recording_table.itemDoubleClicked.connect(lambda _item: self.play_selected())
        layout.addWidget(self.recording_table, 1)
        self.import_button = QPushButton("IMPORT MVNX RECORDING")
        self.import_button.clicked.connect(self.import_mvnx)
        layout.addWidget(self.import_button)
        return panel

    def _populate_recordings(self, select_name: str | None = None) -> None:
        self.recording_table.setRowCount(len(self.recordings))
        selected_row = 0
        for row, recording in enumerate(self.recordings):
            name_item = QTableWidgetItem(recording.name.replace("-", " ").title())
            name_item.setData(Qt.ItemDataRole.UserRole, row)
            duration_item = QTableWidgetItem(_format_time(recording.duration_seconds))
            duration_item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.recording_table.setItem(row, 0, name_item)
            self.recording_table.setItem(row, 1, duration_item)
            if recording.name == select_name:
                selected_row = row
        if self.recordings:
            self.recording_table.selectRow(selected_row)

    def import_mvnx(self) -> None:
        source, _filter = QFileDialog.getOpenFileName(
            self, "Import Xsens MVNX recording", str(Path.home()), "Xsens MVNX (*.mvnx)"
        )
        if not source:
            return
        command = import_mvnx_command(source, self.recording_root)
        self.import_process = QProcess(self)
        self.import_output = ""
        self.import_process.setWorkingDirectory(str(Path(__file__).resolve().parent.parent))
        self.import_process.setProgram(command[0])
        self.import_process.setArguments(command[1:])
        self.import_process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.import_process.readyReadStandardOutput.connect(self._read_import_output)
        self.import_process.finished.connect(lambda code, _status: self._import_finished(code, Path(source).stem))
        self.import_button.setEnabled(False)
        self.import_button.setText("IMPORTING IN ISAAC...")
        self._append_log(f"Importing MVNX: {source}")
        self.import_process.start()

    def _read_import_output(self) -> None:
        if self.import_process is None:
            return
        output = bytes(self.import_process.readAllStandardOutput()).decode("utf-8", errors="replace")
        if output.strip():
            self.import_output = (self.import_output + output)[-12000:]
            self._append_log(output)

    def _import_finished(self, exit_code: int, source_stem: str) -> None:
        self._read_import_output()
        self.import_button.setEnabled(True)
        self.import_button.setText("IMPORT MVNX RECORDING")
        if exit_code:
            detail = self.import_output.strip()[-3000:] or "No exporter output was captured."
            self._show_error(ConsoleError(f"MVNX export failed with status {exit_code}:\n\n{detail}"))
        else:
            try:
                self.recordings = discover_recordings(self.recording_root)
                from batch_export import slugify

                self._populate_recordings(slugify(source_stem))
                self._append_log(f"MVNX import complete: {source_stem}")
            except ConsoleError as error:
                self._show_error(error)
        self.import_process = None

    def _build_control_panel(self) -> QFrame:
        panel, layout = self._panel("CONTROL DECK", "Stack lifecycle and controller-driven playback")

        self.start_button = QPushButton("START MUJOCO + SONIC")
        self.start_button.setObjectName("primary")
        self.start_button.clicked.connect(self.start_stack)
        self.enable_button = QPushButton("ARM RECORDED XSENS STREAM")
        self.enable_button.clicked.connect(self.arm_stream)
        self.stop_stack_button = QPushButton("STOP SIMULATION STACK")
        self.stop_stack_button.clicked.connect(self.stop_stack)
        layout.addWidget(self.start_button)
        layout.addWidget(self.enable_button)
        layout.addWidget(self.stop_stack_button)

        divider = QFrame()
        divider.setFrameShape(QFrame.Shape.HLine)
        divider.setObjectName("divider")
        layout.addWidget(divider)

        self.play_button = QPushButton("PLAY SELECTED + START CONTROLLER")
        self.play_button.setObjectName("primary")
        self.play_button.clicked.connect(self.play_selected)
        self.all_button = QPushButton("PLAY ALL 12 RECORDINGS")
        self.all_button.clicked.connect(self.play_all)
        self.cancel_button = QPushButton("STOP QUEUE AFTER CURRENT")
        self.cancel_button.clicked.connect(self.cancel_queue)
        self.cancel_button.setEnabled(False)
        layout.addWidget(self.play_button)
        layout.addWidget(self.all_button)
        layout.addWidget(self.cancel_button)

        live_divider = QFrame()
        live_divider.setFrameShape(QFrame.Shape.HLine)
        live_divider.setObjectName("divider")
        layout.addWidget(live_divider)
        live_label = QLabel("LIVE XSENS / SIMULATION ONLY")
        live_label.setObjectName("caption")
        layout.addWidget(live_label)
        self.live_guide = QLabel("STEP 1 / Start MuJoCo + SONIC.")
        self.live_guide.setObjectName("liveGuide")
        self.live_guide.setWordWrap(True)
        layout.addWidget(self.live_guide)
        self.live_calibrate_button = QPushButton("CALIBRATE LIVE NEUTRAL")
        self.live_calibrate_button.clicked.connect(self.calibrate_live)
        self.live_enable_button = QPushButton("ENABLE LIVE XSENS")
        self.live_enable_button.clicked.connect(self.enable_live)
        self.live_disable_button = QPushButton("DISABLE LIVE XSENS")
        self.live_disable_button.clicked.connect(self.disable_live)
        self.live_start_button = QPushButton("START LIVE XSENS PIPELINE")
        self.live_start_button.clicked.connect(self.start_live_pipeline)
        self.live_stop_button = QPushButton("STOP LIVE XSENS PIPELINE")
        self.live_stop_button.clicked.connect(self.stop_live_pipeline)
        layout.addWidget(self.live_start_button)
        layout.addWidget(self.live_stop_button)
        layout.addWidget(self.live_calibrate_button)
        layout.addWidget(self.live_enable_button)
        layout.addWidget(self.live_disable_button)

        isaac_divider = QFrame()
        isaac_divider.setFrameShape(QFrame.Shape.HLine)
        isaac_divider.setObjectName("divider")
        layout.addWidget(isaac_divider)
        isaac_label = QLabel("NVIDIA ISAAC SIM / KINEMATIC")
        isaac_label.setObjectName("caption")
        layout.addWidget(isaac_label)
        self.isaac_start_button = QPushButton("VIEW SELECTED IN ISAAC SIM")
        self.isaac_start_button.clicked.connect(self.start_isaac)
        self.isaac_stop_button = QPushButton("STOP ISAAC SIM")
        self.isaac_stop_button.clicked.connect(self.stop_isaac)
        layout.addWidget(self.isaac_start_button)
        layout.addWidget(self.isaac_stop_button)

        self.now_playing = QLabel("No reference published")
        self.now_playing.setObjectName("nowPlaying")
        self.now_playing.setWordWrap(True)
        self.progress = QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setValue(0)
        self.time_label = QLabel("00:00 / 00:00")
        self.time_label.setObjectName("mono")
        layout.addWidget(self.now_playing)
        layout.addWidget(self.progress)
        layout.addWidget(self.time_label)
        layout.addStretch()

        self.emergency_button = QPushButton("EMERGENCY STOP CONTROLLER")
        self.emergency_button.setObjectName("danger")
        self.emergency_button.clicked.connect(self.emergency_stop)
        layout.addWidget(self.emergency_button)
        return panel

    def _build_log_panel(self) -> QFrame:
        panel, layout = self._panel("SONIC TELEMETRY", "Recent controller output; refreshes once per second")
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.log_view.setFont(QFont("DejaVu Sans Mono", 9))
        layout.addWidget(self.log_view, 1)
        return panel

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QWidget { background: #11161a; color: #dce5e8; font-family: Inter, DejaVu Sans, sans-serif; font-size: 13px; }
            #title { color: #f1f6f7; font-size: 27px; font-weight: 800; letter-spacing: 3px; }
            #subtitle, #caption { color: #7f9299; font-size: 11px; }
            #safety { background: #18242a; color: #81d8dc; border-left: 4px solid #45bec4; padding: 11px 14px; }
            #panel { background: #171e22; border: 1px solid #29343a; border-radius: 4px; }
            #panelTitle { color: #f0f4f5; font-size: 15px; font-weight: 700; letter-spacing: 1px; }
            #badge { background: #202a2f; border: 1px solid #344249; border-radius: 3px; color: #87999f; padding: 7px 10px; font-size: 10px; font-weight: 700; }
            QPushButton { background: #222d32; border: 1px solid #3a484e; border-radius: 3px; padding: 10px; font-weight: 700; text-align: left; }
            QPushButton:hover { background: #2b383e; border-color: #5a6b72; }
            QPushButton:disabled { color: #59666b; background: #192024; border-color: #273137; }
            QPushButton#primary { color: #081215; background: #50c8ce; border-color: #6ed9de; }
            QPushButton#primary:hover { background: #6ed9de; }
            QPushButton#danger { color: #ffd8d2; background: #51251f; border: 1px solid #a34b3e; }
            QPushButton#danger:hover { background: #6a2b24; }
            QTableWidget, QPlainTextEdit { background: #0d1215; border: 1px solid #283238; gridline-color: #222c31; selection-background-color: #244b51; selection-color: #f1ffff; }
            QHeaderView::section { background: #20292e; color: #91a4aa; border: none; border-bottom: 1px solid #354148; padding: 7px; font-size: 10px; font-weight: 700; }
            QTableWidget::item { padding: 7px; border-bottom: 1px solid #1d262b; }
            #nowPlaying { color: #f1f5f6; border-left: 3px solid #f2b84b; padding: 8px 10px; font-weight: 700; }
            #liveGuide { color: #dffbff; background: #173039; border-left: 3px solid #50c8ce; padding: 9px 10px; font-weight: 700; }
            #mono { color: #91a4aa; font-family: DejaVu Sans Mono, monospace; }
            QProgressBar { height: 8px; background: #0c1114; border: 1px solid #29343a; border-radius: 2px; text-align: center; color: transparent; }
            QProgressBar::chunk { background: #f2b84b; }
            #divider { color: #2b353a; }
            QSplitter::handle { background: #11161a; width: 8px; }
            """
        )

    def refresh_status(self) -> None:
        try:
            state = self.stack.state()
            self.last_state = state
            stack_text = "STACK ONLINE" if state.running else "STACK DEGRADED" if state.session_exists else "STACK OFFLINE"
            self._set_badge(self.stack_badge, stack_text, state.running)
            self._set_badge(
                self.control_badge,
                "CONTROL ACTIVE" if state.control_started else "CONTROL IDLE",
                state.control_started,
            )
            self._set_badge(
                self.stream_badge,
                "STREAM ENABLED" if state.streaming_enabled else "STREAM OFFLINE",
                state.streaming_enabled,
            )
            self.start_button.setEnabled(not state.running)
            self.stop_stack_button.setEnabled(state.session_exists)
            self.enable_button.setEnabled(state.sonic_initialized and not state.streaming_enabled)
            self.enable_button.setText("RECORDED STREAM ARMED" if state.streaming_enabled else "ARM RECORDED XSENS STREAM")
            live_pipeline = self.live_pipeline.state()
            self.play_button.setEnabled(state.sonic_initialized and not live_pipeline.retarget_running)
            self.all_button.setEnabled(state.sonic_initialized and not self.queue and not live_pipeline.retarget_running)
            isaac_state = self.isaac.state()
            self.isaac_running = isaac_state.running
            self._set_badge(
                self.isaac_badge,
                "ISAAC RUNNING" if isaac_state.running else "ISAAC OFFLINE",
                isaac_state.running,
            )
            self.isaac_stop_button.setEnabled(isaac_state.session_exists)
            live = self._live_status()
            live_state = live.get("state", "OFFLINE")
            live_active = live_state == "ENABLED"
            self._set_badge(self.live_badge, f"LIVE {live_state}", live_active)
            self.live_calibrate_button.setEnabled(state.sonic_initialized and live_state not in {"OFFLINE", "NO_DATA"})
            self.live_enable_button.setEnabled(live_state == "CALIBRATED_DISABLED")
            self.live_disable_button.setEnabled(live_active)
            self.live_start_button.setEnabled(
                not (live_pipeline.source_running and live_pipeline.capture_running and live_pipeline.retarget_running)
            )
            self.live_stop_button.setEnabled(live_pipeline.session_exists)
            self.live_guide.setText(live_next_action(state.sonic_initialized, live_pipeline, live))
            if state.running:
                self.log_view.setPlainText(self.stack.capture_log("sonic", 100))
                scrollbar = self.log_view.verticalScrollBar()
                scrollbar.setValue(scrollbar.maximum())
        except ConsoleError as error:
            self._append_log(f"Status error: {error}")

    def _set_badge(self, badge: QLabel, text: str, active: bool) -> None:
        badge.setText(text)
        badge.setStyleSheet(
            "color: #8ce3d2; border-color: #32796f; background: #17312e;"
            if active
            else "color: #87999f; border-color: #344249; background: #202a2f;"
        )

    def start_stack(self) -> None:
        try:
            self._append_log(self.stack.start())
            self._append_log("MuJoCo viewer is launching. Wait for SONIC 'Init Done', then enable the controller.")
            self.refresh_status()
        except ConsoleError as error:
            self._show_error(error)

    def stop_stack(self) -> None:
        if QMessageBox.question(self, "Stop simulation", "Stop MuJoCo and the SONIC controller?") != QMessageBox.StandardButton.Yes:
            return
        self.cancel_queue()
        try:
            self._append_log(self.stack.stop())
            self.current_recording = None
            self.refresh_status()
        except ConsoleError as error:
            self._show_error(error)

    def arm_stream(self) -> None:
        try:
            self.stack.arm_stream()
            self._append_log(
                "Recorded stream armed. The robot remains on elastic support until a clip is published. "
                "Select a clip and press PLAY SELECTED + START CONTROLLER."
            )
            self.now_playing.setText("WAITING ON ELASTIC SUPPORT / SELECT A RECORDING")
            QTimer.singleShot(1200, self.refresh_status)
        except ConsoleError as error:
            self._show_error(error)

    def _live_status(self) -> dict:
        try:
            return json.loads(LIVE_STATUS.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {"state": "OFFLINE"}

    def _read_live_control(self) -> dict:
        try:
            return json.loads(LIVE_CONTROL.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {"calibration_generation": 0, "enabled": False}

    def calibrate_live(self) -> None:
        if QMessageBox.question(
            self,
            "Calibrate live Xsens",
            "Stand upright in the neutral pose. Capture this pose as live calibration?",
        ) != QMessageBox.StandardButton.Yes:
            return
        self.live_generation += 1
        write_live_control(self.live_generation, False)
        self._append_log(f"Live neutral calibration requested: generation {self.live_generation}.")

    def start_live_pipeline(self) -> None:
        try:
            self.publisher.close()
            write_live_control(self.live_generation, False)
            for path in (Path("/tmp/xsens-live-frame.json"), LIVE_STATUS):
                try:
                    path.unlink()
                except FileNotFoundError:
                    pass
            self._append_log(self.live_pipeline.start())
        except ConsoleError as error:
            self._show_error(error)

    def stop_live_pipeline(self) -> None:
        write_live_control(self.live_generation, False)
        try:
            self._append_log(self.live_pipeline.stop())
        except ConsoleError as error:
            self._show_error(error)

    def enable_live(self) -> None:
        state = self.stack.state()
        pipeline = self.live_pipeline.state()
        live = self._live_status()
        age_ms = live.get("age_ms")
        if not state.sonic_initialized:
            self._show_error(ConsoleError("SONIC is not initialized; complete Step 1 first"))
            return
        if not (pipeline.source_running and pipeline.capture_running and pipeline.retarget_running):
            self._show_error(ConsoleError("the complete live pipeline is not running; complete Step 2 first"))
            return
        if live.get("state") != "CALIBRATED_DISABLED" or not isinstance(age_ms, (int, float)) or age_ms > 150:
            self._show_error(
                ConsoleError("fresh calibrated Xsens data is required; restore MVN streaming and calibrate again")
            )
            return
        try:
            self.publisher.close()
            self.live_publish_baseline = int(live.get("published_windows", 0))
            self.live_enable_deadline = time.monotonic() + 5.0
            write_live_control(self.live_generation, True)
            self._append_log("Live enable requested; waiting for confirmed SONIC reference windows.")
            QTimer.singleShot(100, self._wait_for_live_output)
        except ConsoleError as error:
            self._show_error(error)

    def disable_live(self) -> None:
        write_live_control(self.live_generation, False)
        self._append_log("Live Xsens publication disabled.")

    def _start_live_control(self) -> None:
        try:
            self.stack.start_control()
            QTimer.singleShot(1500, self._release_support)
            self._append_log("Live SONIC control started; support releases after stabilization.")
        except ConsoleError as error:
            self._show_error(error)

    def _wait_for_live_output(self) -> None:
        live = self._live_status()
        if live.get("state") == "STALE":
            write_live_control(self.live_generation, False)
            self._show_error(ConsoleError("Xsens became stale before live control started; calibrate again"))
            return
        if int(live.get("published_windows", 0)) > self.live_publish_baseline:
            try:
                self.stack.arm_stream()
                self._append_log("Live Protocol v1 output confirmed; arming SONIC stream.")
                QTimer.singleShot(700, self._start_live_control)
            except ConsoleError as error:
                self._show_error(error)
            return
        if time.monotonic() >= self.live_enable_deadline:
            write_live_control(self.live_generation, False)
            self._show_error(
                ConsoleError("no live SONIC reference was produced within 5 seconds; check MVN streaming and recalibrate")
            )
            return
        QTimer.singleShot(100, self._wait_for_live_output)

    def play_selected(self) -> None:
        row = self.recording_table.currentRow()
        if row < 0:
            return
        self.cancel_queue()
        if self._arm_for_playback(row):
            self._publish(row)

    def _arm_for_playback(self, row: int) -> bool:
        self.refresh_status()
        if self.last_state and self.last_state.streaming_enabled:
            return True
        if not self.last_state or not self.last_state.sonic_initialized:
            self._show_error(ConsoleError("start MuJoCo + SONIC and wait for initialization before playback"))
            return False
        try:
            self.stack.arm_stream()
            self.now_playing.setText("ARMING STREAM / PLAYBACK STARTS AUTOMATICALLY")
            QTimer.singleShot(700, lambda: self._publish(row))
            return False
        except ConsoleError as error:
            self._show_error(error)
            return False

    def start_isaac(self) -> None:
        row = self.recording_table.currentRow()
        if row < 0:
            return
        recording = self.recordings[row]
        if QMessageBox.question(
            self,
            "Launch NVIDIA Isaac Sim",
            f"Open kinematic playback for {recording.name}?\n\n"
            "This is visual IK validation and does not use the SONIC controller.",
        ) != QMessageBox.StandardButton.Yes:
            return
        try:
            self._append_log(self.isaac.start(recording))
            self.isaac_running = True
            self._set_badge(self.isaac_badge, "ISAAC STARTING", True)
            self.isaac_stop_button.setEnabled(True)
        except ConsoleError as error:
            self._show_error(error)

    def stop_isaac(self) -> None:
        try:
            self._append_log(self.isaac.stop())
            self.isaac_running = False
            self._set_badge(self.isaac_badge, "ISAAC OFFLINE", False)
            self.isaac_stop_button.setEnabled(False)
        except ConsoleError as error:
            self._show_error(error)

    def play_all(self) -> None:
        self.refresh_status()
        start = max(0, self.recording_table.currentRow())
        if not self.last_state or not self.last_state.streaming_enabled:
            if not self._arm_for_playlist(start):
                return
        self.queue = list(range(start, len(self.recordings))) + list(range(0, start))
        self.cancel_button.setEnabled(True)
        self.all_button.setEnabled(False)
        self._play_next()

    def _arm_for_playlist(self, start: int) -> bool:
        self.refresh_status()
        if self.last_state and self.last_state.streaming_enabled:
            return True
        if not self.last_state or not self.last_state.sonic_initialized:
            self._show_error(ConsoleError("start MuJoCo + SONIC and wait for initialization before playback"))
            return False
        try:
            self.stack.arm_stream()
            self.now_playing.setText("ARMING STREAM / PLAYLIST STARTS AUTOMATICALLY")
            QTimer.singleShot(700, self.play_all)
        except ConsoleError as error:
            self._show_error(error)
        return False

    def _play_next(self) -> None:
        if not self.queue:
            self.cancel_button.setEnabled(False)
            self.all_button.setEnabled(bool(self.last_state and self.last_state.sonic_initialized))
            self._append_log("Recording queue complete.")
            return
        row = self.queue.pop(0)
        if self._publish(row):
            self.queue_timer.start(
                round((self.recordings[row].duration_seconds + GUI_PREROLL_SECONDS + 3.0) * 1000)
            )

    def cancel_queue(self) -> None:
        was_active = bool(self.queue or self.queue_timer.isActive())
        self.queue.clear()
        self.queue_timer.stop()
        self.cancel_button.setEnabled(False)
        self.all_button.setEnabled(bool(self.last_state and self.last_state.sonic_initialized))
        if was_active:
            self._append_log("Queue stopped; SONIC will finish the reference already published.")

    def _publish(self, row: int) -> bool:
        if not self._stream_ready():
            return False
        recording = self.recordings[row]
        try:
            message = pack_recording(recording)
            self.publisher.send(message)
        except ConsoleError as error:
            self.cancel_queue()
            self._show_error(error)
            return False
        self.recording_table.selectRow(row)
        self.current_recording = recording
        self.motion_started_at = time.monotonic()
        playback_seconds = recording.duration_seconds + GUI_PREROLL_SECONDS
        self.motion_ends_at = self.motion_started_at + playback_seconds
        self.now_playing.setText(recording.name.replace("-", " ").upper())
        self._append_log(
            f"Published {recording.name}: {GUI_PREROLL_SECONDS:.0f} s standing hold + "
            f"{recording.duration_seconds:.2f} s Xsens motion through SONIC Protocol v1."
        )
        if self.last_state and not self.last_state.control_started:
            QTimer.singleShot(500, self._start_control_after_publish)
        return True

    def _start_control_after_publish(self) -> None:
        try:
            self.stack.start_control()
            self.control_start_deadline = time.monotonic() + 5.0
            self._append_log(
                "Selected reference loaded; waiting for confirmed SONIC control under elastic support."
            )
            QTimer.singleShot(100, self._wait_for_recorded_control)
        except ConsoleError as error:
            self._show_error(error)

    def _wait_for_recorded_control(self) -> None:
        try:
            state = self.stack.state()
        except ConsoleError as error:
            self._show_error(error)
            return
        if state.control_started:
            self._append_log(
                f"SONIC control confirmed; retaining support for {GUI_SUPPORTED_CONTROL_SECONDS:.1f} seconds."
            )
            QTimer.singleShot(round(GUI_SUPPORTED_CONTROL_SECONDS * 1000), self._release_support)
            QTimer.singleShot(1000, self.refresh_status)
            return
        if time.monotonic() >= self.control_start_deadline:
            self._show_error(ConsoleError("SONIC did not confirm CONTROL within 5 seconds; support remains enabled"))
            return
        QTimer.singleShot(100, self._wait_for_recorded_control)

    def _release_support(self) -> None:
        try:
            self.stack.release_support()
            self._append_log("MuJoCo elastic support released after controller stabilization.")
        except ConsoleError as error:
            self._show_error(error)

    def _stream_ready(self) -> bool:
        self.refresh_status()
        if not self.last_state or not self.last_state.streaming_enabled:
            self._show_error(ConsoleError("start the stack and enable controller streaming before playback"))
            return False
        return True

    def _refresh_progress(self) -> None:
        recording = self.current_recording
        if recording is None:
            return
        elapsed = max(0.0, time.monotonic() - self.motion_started_at)
        playback_seconds = recording.duration_seconds + GUI_PREROLL_SECONDS
        fraction = min(1.0, elapsed / max(playback_seconds, 0.001))
        self.progress.setValue(round(fraction * 1000))
        self.time_label.setText(
            f"{_format_time(min(elapsed, playback_seconds))} / {_format_time(playback_seconds)}"
        )
        if time.monotonic() >= self.motion_ends_at and not self.queue_timer.isActive():
            self.now_playing.setText(f"{recording.name.replace('-', ' ').upper()}  /  COMPLETE")

    def emergency_stop(self) -> None:
        if QMessageBox.warning(
            self,
            "Emergency stop",
            "Stop the SONIC controller now? MuJoCo will remain open, but controller output will cease.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        ) != QMessageBox.StandardButton.Yes:
            return
        self.cancel_queue()
        try:
            self.stack.emergency_stop()
            self._append_log("Emergency stop sent to SONIC.")
        except ConsoleError as error:
            self._show_error(error)

    def _append_log(self, message: str) -> None:
        if hasattr(self, "log_view"):
            self.log_view.appendPlainText(message.rstrip())

    def _show_error(self, error: Exception) -> None:
        self._append_log(f"ERROR: {error}")
        QMessageBox.critical(self, "SONIC simulation console", str(error))

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API
        self.queue_timer.stop()
        self.publisher.close()
        event.accept()


def _format_time(seconds: float) -> str:
    total = max(0, round(seconds))
    return f"{total // 60:02d}:{total % 60:02d}"


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--recordings",
        type=Path,
        default=Path("outputs/xsens-g1-batch"),
        help="batch output root (default: outputs/xsens-g1-batch)",
    )
    args = parser.parse_args(argv)
    app = QApplication([sys.argv[0]])
    app.setApplicationName("Xsens SONIC Simulation Console")
    try:
        window = MainWindow(args.recordings)
    except ConsoleError as error:
        QMessageBox.critical(None, "Cannot start simulation console", str(error))
        return 1
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
