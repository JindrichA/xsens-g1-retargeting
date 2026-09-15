#!/usr/bin/env python3
"""Start and inspect the local retargeting experiment stacks."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from typing import Any


ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = ROOT / "config" / "stacks.json"
SUPPORTED_PROFILES = {"simulation", "simulation-adapter"}


class ConfigError(ValueError):
    pass


def variables() -> dict[str, str]:
    projects = Path(os.environ.get("RETARGET_PROJECTS_ROOT", ROOT.parent)).expanduser()
    return {"home": str(Path.home()), "projects": str(projects), "root": str(ROOT)}


def expand(value: str) -> str:
    try:
        return value.format_map(variables())
    except KeyError as exc:
        raise ConfigError(f"unknown template variable: {exc.args[0]}") from exc


def load_stacks(path: Path) -> dict[str, dict[str, Any]]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ConfigError(f"config not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ConfigError(f"invalid JSON in {path}: {exc}") from exc

    stacks = data.get("stacks")
    if not isinstance(stacks, dict) or not stacks:
        raise ConfigError("config must contain a non-empty 'stacks' object")
    for name, stack in stacks.items():
        for field in ("profile", "maturity"):
            if field in stack and not isinstance(stack[field], str):
                raise ConfigError(f"stack '{name}' field '{field}' must be a string")
        dependencies = stack.get("dependencies", [])
        if not isinstance(dependencies, list) or any(
            not isinstance(item, dict)
            or not isinstance(item.get("name"), str)
            or ("location" in item and not isinstance(item["location"], str))
            for item in dependencies
        ):
            raise ConfigError(f"stack '{name}' dependencies must be named objects")
        requirements = stack.get("requirements", {})
        files = requirements.get("files", []) if isinstance(requirements, dict) else None
        if files is None or not isinstance(files, list) or any(
            not isinstance(item, dict)
            or not isinstance(item.get("path"), str)
            or not isinstance(item.get("contains", []), list)
            or any(not isinstance(marker, str) for marker in item.get("contains", []))
            for item in files
        ):
            raise ConfigError(f"stack '{name}' requirements.files must contain path objects")
        if not isinstance(stack.get("components"), list) or not stack["components"]:
            raise ConfigError(f"stack '{name}' must contain components")
        component_names = [component.get("name") for component in stack["components"]]
        if any(not name for name in component_names) or len(component_names) != len(set(component_names)):
            raise ConfigError(f"stack '{name}' has missing or duplicate component names")
        for component in stack["components"]:
            if not all(isinstance(component.get(field), str) for field in ("name", "cwd", "command")):
                raise ConfigError(f"stack '{name}' components require string name, cwd, and command fields")
            for field in ("inputs", "outputs"):
                if field in component and (
                    not isinstance(component[field], list)
                    or any(not isinstance(item, str) for item in component[field])
                ):
                    raise ConfigError(f"stack '{name}' component '{component['name']}' has invalid {field}")
    return stacks


def select_stack(stacks: dict[str, dict[str, Any]], name: str) -> dict[str, Any]:
    try:
        return stacks[name]
    except KeyError as exc:
        choices = ", ".join(sorted(stacks))
        raise ConfigError(f"unknown stack '{name}'; choose one of: {choices}") from exc


def check_stack(name: str, stack: dict[str, Any]) -> list[str]:
    missing = []
    requirements = stack.get("requirements", {})
    for executable in requirements.get("executables", []):
        if shutil.which(executable) is None:
            missing.append(f"executable: {executable}")
    for raw_path in requirements.get("paths", []):
        path = Path(expand(raw_path))
        if not path.exists():
            missing.append(f"path: {path}")
    for file_requirement in requirements.get("files", []):
        path = Path(expand(file_requirement["path"]))
        try:
            content = path.read_text(encoding="utf-8")
        except OSError:
            missing.append(f"file: {path}")
            continue
        for marker in file_requirement.get("contains", []):
            if marker not in content:
                missing.append(f"content {marker!r} in: {path}")
    for component in stack["components"]:
        cwd = Path(expand(component["cwd"]))
        if not cwd.is_dir():
            missing.append(f"component {component['name']} cwd: {cwd}")
    if missing:
        print(f"{name}: NOT READY")
        for item in missing:
            print(f"  missing {item}")
    else:
        print(f"{name}: ready")
    return missing


def describe_stack(name: str, stack: dict[str, Any]) -> None:
    print(name)
    print(f"  description: {stack.get('description', 'not documented')}")
    print(f"  profile: {stack.get('profile', 'unspecified')}")
    print(f"  maturity: {stack.get('maturity', 'unspecified')}")
    print(f"  tmux session: {stack.get('session', f'retarget-{name}')}")

    dependencies = stack.get("dependencies", [])
    print("  dependencies:")
    if not dependencies:
        print("    none documented")
    for dependency in dependencies:
        location = expand(dependency["location"]) if dependency.get("location") else "system"
        print(f"    - {dependency['name']} [{dependency.get('kind', 'external')}] @ {location}")
        if dependency.get("purpose"):
            print(f"      {dependency['purpose']}")

    requirements = stack.get("requirements", {})
    print("  prerequisite checks:")
    for executable in requirements.get("executables", []):
        print(f"    - executable: {executable}")
    for path in requirements.get("paths", []):
        print(f"    - path: {expand(path)}")
    for file_requirement in requirements.get("files", []):
        markers = ", ".join(file_requirement.get("contains", []))
        suffix = f" containing: {markers}" if markers else ""
        print(f"    - file: {expand(file_requirement['path'])}{suffix}")
    if not any(requirements.get(field) for field in ("executables", "paths", "files")):
        print("    none documented")

    print("  components:")
    for component in stack["components"]:
        print(f"    - {component['name']}")
        print(f"      runtime: {component.get('runtime', 'not documented')}")
        print(f"      cwd: {expand(component['cwd'])}")
        if component.get("role"):
            print(f"      role: {component['role']}")
        for field in ("inputs", "outputs"):
            values = component.get(field, [])
            print(f"      {field}: {', '.join(values) if values else 'none'}")


def tmux(*args: str, capture: bool = False, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["tmux", *args],
        check=check,
        text=True,
        capture_output=capture,
    )


def session_exists(session: str) -> bool:
    return tmux("has-session", "-t", session, capture=True, check=False).returncode == 0


def pane_command(component: dict[str, Any]) -> str:
    cwd = Path(expand(component["cwd"]))
    command = expand(component["command"])
    # Keep a failed pane open so its error remains inspectable with `logs`.
    return (
        f"cd {shlex_quote(str(cwd))} && {command}; "
        "rc=$?; printf '\\n[retargetctl] process exited with status %s\\n' \"$rc\"; exec bash"
    )


def shlex_quote(value: str) -> str:
    import shlex

    return shlex.quote(value)


def run_foreground(name: str, stack: dict[str, Any]) -> int:
    processes: list[subprocess.Popen[bytes]] = []
    try:
        for component in stack["components"]:
            delay = float(component.get("delay", 0))
            if delay > 0:
                time.sleep(delay)
            process = subprocess.Popen(
                ["bash", "-lc", expand(component["command"])],
                cwd=Path(expand(component["cwd"])),
                start_new_session=True,
            )
            processes.append(process)
            print(f"started {component['name']} (pid {process.pid})")

        print(f"{name} is running; press Ctrl-C to stop the complete stack")
        while True:
            for component, process in zip(stack["components"], processes):
                return_code = process.poll()
                if return_code is not None:
                    print(f"{component['name']} exited with status {return_code}", file=sys.stderr)
                    return return_code or 1
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nstopping stack")
        return 0
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        deadline = time.monotonic() + 5
        for process in processes:
            if process.poll() is None:
                try:
                    process.wait(timeout=max(0, deadline - time.monotonic()))
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)


def start_stack(
    name: str,
    stack: dict[str, Any],
    attach: bool,
    dry_run: bool,
    backend: str = "auto",
) -> int:
    profile = stack.get("profile")
    if profile not in SUPPORTED_PROFILES:
        declared = profile if profile is not None else "missing"
        raise ConfigError(f"stack '{name}' uses unsupported profile '{declared}'")
    session = stack.get("session", f"retarget-{name}")
    commands = [(component["name"], pane_command(component)) for component in stack["components"]]
    if dry_run:
        print(f"backend: {backend}")
        print(f"tmux session: {session}")
        for component_name, command in commands:
            print(f"[{component_name}] {command}")
        return 0

    if check_stack(name, stack):
        return 1
    if backend == "auto":
        backend = "tmux" if shutil.which("tmux") else "foreground"
    if backend == "foreground":
        return run_foreground(name, stack)
    if shutil.which("tmux") is None:
        raise ConfigError("tmux backend requested, but tmux is not installed")
    if session_exists(session):
        print(f"session already exists: {session}", file=sys.stderr)
        print(f"attach with: tmux attach -t {session}", file=sys.stderr)
        return 1

    first_name, first_command = commands[0]
    tmux("new-session", "-d", "-s", session, "-n", first_name, "bash", "-lc", first_command)
    try:
        for component, (component_name, command) in zip(stack["components"][1:], commands[1:]):
            delay = float(component.get("delay", 0))
            if delay > 0:
                time.sleep(delay)
            tmux("new-window", "-t", session, "-n", component_name, "bash", "-lc", command)
        tmux("set-option", "-t", session, "remain-on-exit", "on")
    except Exception:
        tmux("kill-session", "-t", session, check=False)
        raise

    print(f"started {name} in tmux session {session}")
    print(f"attach: tmux attach -t {session}")
    if attach:
        os.execvp("tmux", ["tmux", "attach", "-t", session])
    return 0


def show_status(name: str, stack: dict[str, Any]) -> int:
    session = stack.get("session", f"retarget-{name}")
    if not session_exists(session):
        print(f"{name}: stopped ({session})")
        return 1
    result = tmux(
        "list-panes",
        "-s",
        "-t",
        session,
        "-F",
        "#{window_name}: #{pane_current_command} dead=#{pane_dead}",
        capture=True,
    )
    pane_lines = result.stdout.splitlines()
    failed = []
    for component in stack["components"]:
        pane = next(
            (line for line in pane_lines if line.startswith(f"{component['name']}: ")),
            None,
        )
        if pane is None:
            failed.append(f"{component['name']} (missing pane)")
            continue
        if pane.endswith("dead=1"):
            failed.append(f"{component['name']} (dead pane)")
            continue
        log = tmux(
            "capture-pane",
            "-p",
            "-t",
            f"{session}:{component['name']}",
            "-S",
            "-100",
            capture=True,
            check=False,
        ).stdout
        if "[retargetctl] process exited with status" in log:
            failed.append(f"{component['name']} (process exited)")
    label = "degraded" if failed else "running"
    print(f"{name}: {label} ({session})")
    print(result.stdout.rstrip())
    if failed:
        print(f"failed components: {', '.join(failed)}")
        return 1
    return 0


def show_logs(name: str, stack: dict[str, Any], component: str, lines: int) -> int:
    session = stack.get("session", f"retarget-{name}")
    names = {item["name"] for item in stack["components"]}
    if component not in names:
        raise ConfigError(f"unknown component '{component}'; choose one of: {', '.join(sorted(names))}")
    if not session_exists(session):
        print(f"stack is not running: {name}", file=sys.stderr)
        return 1
    result = tmux("capture-pane", "-p", "-t", f"{session}:{component}", "-S", f"-{lines}", capture=True)
    print(result.stdout.rstrip())
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    subparsers = parser.add_subparsers(dest="action", required=True)
    subparsers.add_parser("list", help="list configured stacks")

    describe = subparsers.add_parser("describe", help="show dependencies and process boundaries")
    describe.add_argument("stack")

    check = subparsers.add_parser("check", help="check stack prerequisites")
    check.add_argument("stack", nargs="?")

    start = subparsers.add_parser("start", help="start a stack")
    start.add_argument("stack")
    start.add_argument("--attach", action="store_true")
    start.add_argument("--dry-run", action="store_true")
    start.add_argument("--backend", choices=("auto", "tmux", "foreground"), default="auto")

    status = subparsers.add_parser("status", help="show stack process status")
    status.add_argument("stack")

    logs = subparsers.add_parser("logs", help="print recent component output")
    logs.add_argument("stack")
    logs.add_argument("component")
    logs.add_argument("--lines", type=int, default=200)

    stop = subparsers.add_parser("stop", help="stop all processes in a stack")
    stop.add_argument("stack")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        stacks = load_stacks(args.config)
        if args.action == "list":
            for name, stack in stacks.items():
                profile = stack.get("profile", "unspecified")
                maturity = stack.get("maturity", "unspecified")
                print(f"{name:24} [{profile} / {maturity}] {stack.get('description', '')}")
            return 0
        if args.action == "check" and args.stack is None:
            results = [check_stack(name, stack) for name, stack in stacks.items()]
            return int(any(results))

        stack = select_stack(stacks, args.stack)
        if args.action == "describe":
            describe_stack(args.stack, stack)
            return 0
        if args.action == "check":
            return int(bool(check_stack(args.stack, stack)))
        if args.action == "start":
            return start_stack(args.stack, stack, args.attach, args.dry_run, args.backend)
        if shutil.which("tmux") is None:
            raise ConfigError(f"'{args.action}' is available only for stacks started with the tmux backend")
        if args.action == "status":
            return show_status(args.stack, stack)
        if args.action == "logs":
            return show_logs(args.stack, stack, args.component, args.lines)
        if args.action == "stop":
            session = stack.get("session", f"retarget-{args.stack}")
            if not session_exists(session):
                print(f"{args.stack}: already stopped")
                return 0
            tmux("kill-session", "-t", session)
            print(f"stopped {args.stack}")
            return 0
    except ConfigError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
