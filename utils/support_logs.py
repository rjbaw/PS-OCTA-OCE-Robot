#!/usr/bin/env python3
"""Summarize one ROS 2 incident and build a support bundle.

The ROS log directory contains one file per process and one launch directory per
driver restart.  Looking at the newest file (or its last few lines) is therefore
usually misleading after a clean shutdown.  This tool associates those files
with a coordinator session, selects an operational incident, and preserves the
raw evidence alongside a short human-readable report.  Terminal output is
either a concise single-session summary or one contiguous, verbatim clip from
the selected coordinator log; it never merges lines from several processes.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import platform
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterable


ROS_HEADER_RE = re.compile(
    r"^\[(?P<severity>DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]\s+"
    r"\[(?P<timestamp>\d+(?:\.\d+)?)\]\s*(?P<body>.*)$"
)
LAUNCH_HEADER_RE = re.compile(r"^(?P<timestamp>\d{10}(?:\.\d+)?)\s+(?P<body>.*)$")
SEVERITY_RE = re.compile(r"\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]")
PROCESS_FILE_RE = re.compile(r"^(?P<node>.+)_(?P<pid>\d+)_(?P<timestamp>\d{13,})\.log$")
FULLSCAN_VALUE_RE = re.compile(
    r"\bfull[\s_-]*scan\s*:\s*(?P<value>1|0|true|false)\b", re.IGNORECASE
)
FULLSCAN_REQUEST_RE = re.compile(r"\[Fullscan\]\s+REQUEST_ACCEPTED\b", re.IGNORECASE)
FULLSCAN_TERMINAL_RE = re.compile(
    r"\[Fullscan\]\s+(?P<outcome>CANCEL|ABORT|COMPLETE)\b", re.IGNORECASE
)
ACTIVITY_RE = re.compile(
    r"(?:"
    r"\bStep\s*\[\d+[/\d]*\]|"
    r"\[(?:SUBSCRIBING|Action)\]|"
    r"\b(?:Focus|Move|Reset|Scanning)\s+Action\b|"
    r"\b(?:goal accepted|SUCCEEDED|FAILED|Scan Complete)\b|"
    r"\[Fullscan\].*(?:REQUEST_ACCEPTED|START|CANCEL|ABORT|COMPLETE)|"
    r"\b(?:scan_trigger|scan_3d|apply_octa|octa_mode)\s*:\s*(?:1|true)\b"
    r")",
    re.IGNORECASE,
)
FAILURE_RE = re.compile(
    r"\b(?:failed|failure|aborted|cancelled|canceled|timed out|timeout|"
    r"exception|traceback|segmentation fault|core dumped|unexpectedly)\b",
    re.IGNORECASE,
)
COORDINATOR_CRASH_RE = re.compile(
    r"coordinator_node-\d+.*process has died.*exit code\s+(?P<code>-?\d+)",
    re.IGNORECASE,
)
PROCESS_STARTED_RE = re.compile(r"process started with pid\s*\[(?P<pid>\d+)\]")
REQUIRED_SHUTDOWN_RE = re.compile(
    r"process\[(?P<process>[^]]+)\] was required: shutting down", re.IGNORECASE
)

IGNORED_FAILURE_PATTERNS = (
    "no 3d sensor plugin(s) defined for octomap updates",
    "resolution not specified for octomap",
    "publisher already registered for node name",
)
EXPECTED_CANCELLATION_PATTERNS = (
    "goal canceled by the user",
    "action cancelled normally",
    "request canceled before acceptance",
    "ignoring focus result after its goal was canceled",
    "ignoring move result after its goal was canceled",
    "ignoring freedrive result after its goal was canceled",
    "ignoring reset result after its goal was canceled",
)
SHUTDOWN_PATTERNS = (
    "user interrupted with ctrl-c",
    "keyboardinterrupt received",
    "sending signal 'sigint'",
    "sending signal 'sigterm'",
    "shutting down launched system",
    "failed to terminate '5' seconds after receiving 'sigint'",
)


@dataclass(frozen=True)
class Event:
    path: Path
    line: int
    timestamp: float
    severity: str
    text: str

    @property
    def flat(self) -> str:
        return " | ".join(
            part.strip() for part in self.text.splitlines() if part.strip()
        )


@dataclass
class ParsedLog:
    path: Path
    events: list[Event]
    start_time: float
    end_time: float


@dataclass
class Session:
    coordinator: ParsedLog
    pid: int | None
    launch: ParsedLog | None = None
    fullscan_starts: list[Event] = field(default_factory=list)
    failures: list[Event] = field(default_factory=list)
    crashes: list[Event] = field(default_factory=list)
    launch_failures: list[Event] = field(default_factory=list)
    activity: list[Event] = field(default_factory=list)
    shutdown_time: float | None = None

    @property
    def start_time(self) -> float:
        if self.launch is not None:
            return min(self.launch.start_time, self.coordinator.start_time)
        return self.coordinator.start_time

    @property
    def end_time(self) -> float:
        if self.launch is not None:
            return max(self.launch.end_time, self.coordinator.end_time)
        return self.coordinator.end_time


@dataclass(frozen=True)
class Selection:
    session: Session
    event: Event
    anchor: Event
    reason: str


def _timestamp_from_filename(path: Path) -> tuple[int | None, float | None]:
    match = PROCESS_FILE_RE.match(path.name)
    if not match:
        return None, None
    value = float(match.group("timestamp"))
    while value > 4_000_000_000:
        value /= 1000.0
    return int(match.group("pid")), value


def parse_log(path: Path) -> ParsedLog:
    """Parse ROS and launch logs while retaining multiline ROS messages."""

    events: list[Event] = []
    current_line = 0
    current_timestamp: float | None = None
    current_severity = "INFO"
    current_parts: list[str] = []

    def flush() -> None:
        nonlocal current_parts
        if current_timestamp is not None and current_parts:
            events.append(
                Event(
                    path=path,
                    line=current_line,
                    timestamp=current_timestamp,
                    severity=current_severity,
                    text="\n".join(current_parts),
                )
            )
        current_parts = []

    try:
        with path.open("r", encoding="utf-8", errors="replace") as handle:
            for line_number, raw_line in enumerate(handle, start=1):
                line = raw_line.rstrip("\n")
                ros_match = ROS_HEADER_RE.match(line)
                if ros_match:
                    flush()
                    current_line = line_number
                    current_timestamp = float(ros_match.group("timestamp"))
                    current_severity = ros_match.group("severity").replace(
                        "WARNING", "WARN"
                    )
                    current_parts = [ros_match.group("body")]
                    continue

                launch_match = LAUNCH_HEADER_RE.match(line)
                if launch_match:
                    flush()
                    current_line = line_number
                    current_timestamp = float(launch_match.group("timestamp"))
                    body = launch_match.group("body")
                    severity_match = SEVERITY_RE.search(body)
                    current_severity = (
                        severity_match.group(1).replace("WARNING", "WARN")
                        if severity_match
                        else "INFO"
                    )
                    current_parts = [body]
                    continue

                if current_timestamp is not None:
                    current_parts.append(line)
    except OSError:
        events = []

    flush()
    _, filename_start = _timestamp_from_filename(path)
    try:
        stat_time = path.stat().st_mtime
    except OSError:
        stat_time = 0.0
    start_time = events[0].timestamp if events else (filename_start or stat_time)
    end_time = events[-1].timestamp if events else stat_time
    return ParsedLog(path=path, events=events, start_time=start_time, end_time=end_time)


def _contains_ignored_failure(event: Event) -> bool:
    lowered = event.flat.lower()
    return any(pattern in lowered for pattern in IGNORED_FAILURE_PATTERNS)


def _is_expected_cancellation(event: Event) -> bool:
    lowered = event.flat.lower()
    return any(pattern in lowered for pattern in EXPECTED_CANCELLATION_PATTERNS)


def _is_shutdown_event(event: Event) -> bool:
    lowered = event.flat.lower()
    return any(pattern in lowered for pattern in SHUTDOWN_PATTERNS)


def _is_failure(event: Event) -> bool:
    if _contains_ignored_failure(event) or _is_shutdown_event(event):
        return False
    if event.severity in {"ERROR", "FATAL"}:
        return True
    if _is_expected_cancellation(event):
        return False
    return bool(FAILURE_RE.search(event.flat))


def _find_fullscan_starts(events: Iterable[Event]) -> list[Event]:
    events = list(events)
    accepted = [event for event in events if FULLSCAN_REQUEST_RE.search(event.flat)]
    if accepted:
        return accepted

    # Older coordinator logs predate the explicit REQUEST_ACCEPTED marker.  For
    # those only, recover rising edges from the published/subscribed field.
    starts: list[Event] = []
    state: bool | None = None
    for event in events:
        values = list(FULLSCAN_VALUE_RE.finditer(event.flat))
        for match in values:
            value = match.group("value").lower() in {"1", "true"}
            if value and state is not True:
                starts.append(event)
            state = value
    return starts


def _launch_shutdown_time(events: Iterable[Event]) -> float | None:
    for event in events:
        lowered = event.flat.lower()
        if any(pattern in lowered for pattern in SHUTDOWN_PATTERNS[:5]):
            return event.timestamp
    return None


def _build_sessions(log_dir: Path) -> tuple[list[Session], list[ParsedLog]]:
    launch_logs = [parse_log(path) for path in sorted(log_dir.glob("*/launch.log"))]
    launch_by_coordinator_pid: dict[int, list[tuple[float, ParsedLog]]] = {}
    for launch in launch_logs:
        for event in launch.events:
            if (
                "coordinator_node" not in event.flat
                or "process started with pid" not in event.flat
            ):
                continue
            match = PROCESS_STARTED_RE.search(event.flat)
            if match:
                pid = int(match.group("pid"))
                launch_by_coordinator_pid.setdefault(pid, []).append(
                    (event.timestamp, launch)
                )

    sessions: list[Session] = []
    for path in sorted(log_dir.glob("coordinator_node_*.log")):
        coordinator = parse_log(path)
        pid, filename_start = _timestamp_from_filename(path)
        reference = filename_start or coordinator.start_time
        launch = None
        matching_launches = (
            launch_by_coordinator_pid.get(pid, []) if pid is not None else []
        )
        if matching_launches:
            process_start, candidate = min(
                matching_launches,
                key=lambda item: abs(item[0] - reference),
            )
            if abs(process_start - reference) <= 15.0:
                launch = candidate
        if launch is None and launch_logs:
            closest = min(
                launch_logs, key=lambda item: abs(item.start_time - reference)
            )
            if abs(closest.start_time - reference) <= 15.0:
                launch = closest

        shutdown_time = _launch_shutdown_time(launch.events) if launch else None
        crashes: list[Event] = []
        launch_failures: list[Event] = []
        if launch:
            for event in launch.events:
                if not COORDINATOR_CRASH_RE.search(event.flat):
                    continue
                if shutdown_time is None or event.timestamp < shutdown_time:
                    crashes.append(event)
            for index, event in enumerate(launch.events):
                required_match = REQUIRED_SHUTDOWN_RE.search(event.flat)
                if not required_match:
                    continue
                # A required process can also die while an intentional
                # Ctrl-C/SIGTERM shutdown is already in progress. That is
                # shutdown fallout, not the incident that selected this run.
                if shutdown_time is not None and event.timestamp > shutdown_time:
                    continue
                process_name = required_match.group("process")
                for prior in reversed(launch.events[:index]):
                    if event.timestamp - prior.timestamp > 2.0:
                        break
                    if (
                        f"[{process_name}]" in prior.flat
                        and "process has died" in prior.flat.lower()
                    ):
                        launch_failures.append(prior)
                        break

        failures = [
            event
            for event in coordinator.events
            if _is_failure(event)
            and (shutdown_time is None or event.timestamp < shutdown_time)
        ]
        activity = [
            event for event in coordinator.events if ACTIVITY_RE.search(event.flat)
        ]
        sessions.append(
            Session(
                coordinator=coordinator,
                pid=pid,
                launch=launch,
                fullscan_starts=_find_fullscan_starts(coordinator.events),
                failures=failures,
                crashes=crashes,
                launch_failures=launch_failures,
                activity=activity,
                shutdown_time=shutdown_time,
            )
        )
    return sessions, launch_logs


def _failure_reason(session: Session, event: Event) -> str:
    if event in session.crashes:
        return "coordinator crash"
    if event in session.launch_failures:
        process_match = re.search(
            r"\[(?:DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]\s+"
            r"\[([^]]+)\]:\s*process has died",
            event.flat,
        )
        process = process_match.group(1) if process_match else "required process"
        return f"required process crash ({process})"
    return "coordinator failure"


def _selection_for_session(
    session: Session, incident: str, custom_match: re.Pattern[str] | None
) -> Selection | None:
    combined_events = list(session.coordinator.events)
    if session.launch:
        combined_events.extend(session.launch.events)

    if custom_match is not None:
        matches = [
            event for event in combined_events if custom_match.search(event.flat)
        ]
        if not matches:
            return None
        event = max(matches, key=lambda item: item.timestamp)
        return Selection(session, event, event, f"custom match: {custom_match.pattern}")

    if incident == "fullscan":
        if not session.fullscan_starts:
            return None
        event = session.fullscan_starts[-1]
        return Selection(session, event, event, "fullscan request")

    if incident == "error":
        failures = session.failures + session.crashes + session.launch_failures
        if not failures:
            return None
        event = max(failures, key=lambda item: item.timestamp)
        prior_fullscans = [
            item
            for item in session.fullscan_starts
            if item.timestamp <= event.timestamp
        ]
        anchor = prior_fullscans[-1] if prior_fullscans else event
        reason = _failure_reason(session, event)
        if prior_fullscans:
            reason += " after fullscan request"
        return Selection(session, event, anchor, reason)

    if incident == "latest":
        events = session.activity or session.coordinator.events
        if not events:
            return None
        event = max(events, key=lambda item: item.timestamp)
        return Selection(session, event, event, "latest coordinator activity")

    notable = (
        session.failures
        + session.crashes
        + session.launch_failures
        + session.fullscan_starts
    )
    if notable:
        event = max(notable, key=lambda item: item.timestamp)
        prior_fullscans = [
            item
            for item in session.fullscan_starts
            if item.timestamp <= event.timestamp
        ]
        if event in session.crashes:
            anchor = prior_fullscans[-1] if prior_fullscans else event
            reason = "coordinator crash"
            if prior_fullscans:
                reason += " after fullscan request"
        elif event in session.launch_failures:
            anchor = prior_fullscans[-1] if prior_fullscans else event
            reason = _failure_reason(session, event)
            if prior_fullscans:
                reason += " after fullscan request"
        elif event in session.failures:
            anchor = prior_fullscans[-1] if prior_fullscans else event
            reason = "coordinator failure"
            if prior_fullscans:
                reason += " after fullscan request"
        else:
            anchor = event
            reason = "fullscan request"
        return Selection(session, event, anchor, reason)
    return None


def select_sessions(
    sessions: list[Session], incident: str, match: str | None
) -> list[Selection]:
    try:
        custom_match = re.compile(match, re.IGNORECASE) if match else None
    except re.error as error:
        raise ValueError(f"invalid MATCH regular expression: {error}") from error

    selections = [
        selection
        for session in sessions
        if (selection := _selection_for_session(session, incident, custom_match))
        is not None
    ]

    # In auto mode, only fall back to routine activity when no incident-bearing
    # session exists.  This is what keeps a later clean shutdown from hiding the
    # preceding failure/fullscan session.
    if incident == "auto" and custom_match is None and not selections:
        selections = [
            selection
            for session in sessions
            if (selection := _selection_for_session(session, "latest", None))
            is not None
        ]
    # SESSION is a session offset, so order by the coordinator session rather
    # than by the timestamp of whichever event happened to match the filter.
    # Keep newest first internally so SESSION=0 remains the newest match.
    return sorted(
        selections,
        key=lambda item: (item.session.start_time, item.event.timestamp),
        reverse=True,
    )


def _format_time(timestamp: float) -> str:
    try:
        return (
            datetime.fromtimestamp(timestamp)
            .astimezone()
            .isoformat(timespec="milliseconds")
        )
    except (OSError, OverflowError, ValueError):
        return f"{timestamp:.3f}"


def _compact_event(event: Event, limit: int = 600) -> str:
    text = re.sub(r"\s+", " ", event.flat).strip()
    if len(text) > limit:
        text = text[: limit - 1] + "…"
    return f"{_format_time(event.timestamp)}  {event.severity:<5}  {text}"


def _find_manager_log(
    log_dir: Path, start_time: float, end_time: float
) -> ParsedLog | None:
    matches: list[ParsedLog] = []
    for path in log_dir.glob("python3_*.log"):
        parsed = parse_log(path)
        if not any("[driver_manager]" in event.flat for event in parsed.events):
            continue
        if parsed.start_time <= end_time + 60 and parsed.end_time >= start_time - 60:
            matches.append(parsed)
    if not matches:
        return None
    return max(matches, key=lambda item: item.end_time)


def _related_sessions(
    selection: Selection,
    sessions: Iterable[Session],
    before_seconds: float = 5 * 60,
    after_seconds: float = 2 * 60,
) -> list[Session]:
    """Return nearby restarts that may belong to the same user-visible incident."""

    start = min(selection.anchor.timestamp, selection.event.timestamp) - before_seconds
    end = max(selection.anchor.timestamp, selection.event.timestamp) + after_seconds
    return sorted(
        (
            session
            for session in sessions
            if session.end_time >= start and session.start_time <= end
        ),
        key=lambda session: session.start_time,
    )


def _session_files(log_dir: Path, session: Session) -> list[Path]:
    selected: set[Path] = {session.coordinator.path}
    process_starts: dict[int, list[float]] = {}
    if session.launch is not None:
        selected.add(session.launch.path)
        for event in session.launch.events:
            match = PROCESS_STARTED_RE.search(event.flat)
            if match:
                pid = int(match.group("pid"))
                process_starts.setdefault(pid, []).append(event.timestamp)

    for path in log_dir.glob("*.log"):
        pid, start_time = _timestamp_from_filename(path)
        matching_starts = process_starts.get(pid, []) if pid is not None else []
        if (
            start_time is not None
            and matching_starts
            and min(abs(start_time - value) for value in matching_starts) <= 15.0
        ):
            selected.add(path)
            continue
        if (
            start_time is not None
            and abs(start_time - session.coordinator.start_time) <= 5.0
        ):
            selected.add(path)

    return sorted(selected)


def _command_output(
    command: list[str], timeout: float = 5.0, cwd: Path | None = None
) -> str:
    rendered = " ".join(command)
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
            cwd=cwd,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return f"$ {rendered}\n(unavailable: {error})\n"
    output = completed.stdout
    if completed.stderr:
        output += completed.stderr
    return f"$ {rendered}\n{output.rstrip()}\n(exit {completed.returncode})\n"


def _runtime_context(
    repo_root: Path,
    container: str,
    include_docker: bool,
    since: float | None = None,
    until: float | None = None,
) -> tuple[str, str]:
    context = [
        f"captured_at={datetime.now().astimezone().isoformat(timespec='seconds')}",
        f"platform={platform.platform()}",
        f"python={sys.version.split()[0]}",
        f"repository={repo_root.name}",
        "",
        _command_output(["git", "rev-parse", "HEAD"], cwd=repo_root),
        _command_output(["git", "status", "--short"], cwd=repo_root),
    ]
    docker_log = "Docker capture disabled.\n"
    if include_docker and shutil.which("docker"):
        context.append(
            _command_output(
                [
                    "docker",
                    "ps",
                    "-a",
                    "--filter",
                    f"name=^/{container}$",
                    "--format",
                    "table {{.Names}}\t{{.Status}}\t{{.Image}}",
                ]
            )
        )
        docker_command = ["docker", "logs", "--timestamps"]
        if since is not None:
            docker_command.extend(["--since", f"{max(0.0, since):.3f}"])
        if until is not None:
            docker_command.extend(["--until", f"{max(0.0, until):.3f}"])
        docker_command.extend(["--tail", "5000", container])
        docker_log = _command_output(docker_command, timeout=10.0)
    elif include_docker:
        context.append("docker command not found\n")
        docker_log = "docker command not found\n"
    return "\n".join(context).rstrip() + "\n", docker_log


def _display_path(path: Path | None, log_dir: Path) -> str:
    if path is None:
        return "(not found)"
    try:
        return str(Path("logs") / path.resolve().relative_to(log_dir.resolve()))
    except (OSError, ValueError):
        return str(path)


def _fullscan_window(selection: Selection) -> tuple[float, Event | None]:
    session = selection.session
    if selection.anchor not in session.fullscan_starts:
        return selection.event.timestamp, None

    next_request_time = min(
        (
            event.timestamp
            for event in session.fullscan_starts
            if event.timestamp > selection.anchor.timestamp
        ),
        default=session.end_time,
    )
    terminal = next(
        (
            event
            for event in session.coordinator.events
            if selection.anchor.timestamp <= event.timestamp <= next_request_time
            and FULLSCAN_TERMINAL_RE.search(event.flat)
        ),
        None,
    )
    return (terminal.timestamp if terminal else next_request_time), terminal


def _last_coordinator_activity(selection: Selection) -> Event | None:
    end_time, _ = _fullscan_window(selection)
    activity = [
        event
        for event in selection.session.activity
        if selection.anchor.timestamp <= event.timestamp <= end_time
    ]
    if activity:
        return max(activity, key=lambda event: event.timestamp)
    return None


def _session_outcome(selection: Selection) -> str:
    session = selection.session
    end_time, terminal = _fullscan_window(selection)
    failures = [
        event
        for event in session.failures + session.crashes + session.launch_failures
        if selection.anchor.timestamp <= event.timestamp <= end_time
    ]
    if failures:
        event = max(failures, key=lambda item: item.timestamp)
        outcome = f"{_failure_reason(session, event)}: {_compact_event(event)}"
        if terminal is not None and terminal is not event:
            match = FULLSCAN_TERMINAL_RE.search(terminal.flat)
            outcome += (
                f"; Fullscan {match.group('outcome').upper()} at "
                f"{_format_time(terminal.timestamp)}"
            )
        return outcome
    if terminal is not None:
        match = FULLSCAN_TERMINAL_RE.search(terminal.flat)
        return (
            f"Fullscan {match.group('outcome').upper()} at "
            f"{_format_time(terminal.timestamp)}"
        )
    if session.shutdown_time is not None:
        return f"clean shutdown at {_format_time(session.shutdown_time)}"
    return f"no crash or clean shutdown recorded; log ends {_format_time(session.end_time)}"


def _session_selection_lines(
    selection: Selection,
    candidates: list[Selection],
    log_dir: Path,
) -> list[str]:
    """Describe the selected match and the chronological matching range."""

    selected_index = candidates.index(selection)
    oldest_index = len(candidates) - 1
    oldest = candidates[oldest_index].session
    newest = candidates[0].session
    return [
        (
            f"Selected: SESSION={selected_index} of {len(candidates)} matching "
            f"session(s), {_format_time(selection.session.start_time)} to "
            f"{_format_time(selection.session.end_time)}"
        ),
        "Matching range (chronological, oldest -> newest):",
        (
            f"  Oldest: SESSION={oldest_index}, started "
            f"{_format_time(oldest.start_time)}, "
            f"{_display_path(oldest.coordinator.path, log_dir)}"
        ),
        (
            f"  Newest: SESSION=0 (default), started "
            f"{_format_time(newest.start_time)}, "
            f"{_display_path(newest.coordinator.path, log_dir)}"
        ),
    ]


def _render_report(
    selection: Selection | None,
    candidates: list[Selection],
    log_dir: Path,
) -> str:
    lines = [
        "PS-OCTA/OCE incident summary",
        f"Created: {datetime.now().astimezone().isoformat(timespec='seconds')}",
        "",
    ]
    if selection is None:
        lines.extend(
            [
                "No coordinator session was found.",
                "Runtime context and the newest available launch log are still included in the bundle.",
                "",
            ]
        )
        return "\n".join(lines)

    session = selection.session
    fullscan_requests = [
        event
        for event in session.fullscan_starts
        if event.timestamp <= selection.event.timestamp
    ]
    last_activity = _last_coordinator_activity(selection)
    lines.extend(
        [
            f"Reason: {selection.reason}",
            *_session_selection_lines(selection, candidates, log_dir),
            f"Primary INFO log: {_display_path(session.coordinator.path, log_dir)}",
            f"Launch log: {_display_path(session.launch.path if session.launch else None, log_dir)}",
            "",
            "Observed:",
        ]
    )
    if fullscan_requests:
        request = fullscan_requests[-1]
        lines.append(f"  Fullscan request: {_compact_event(request)}")
    if last_activity is not None:
        lines.append(f"  Last coordinator activity: {_compact_event(last_activity)}")
    lines.extend(
        [
            f"  Outcome: {_session_outcome(selection)}",
            f"  Selected source: {_display_path(selection.event.path, log_dir)}:{selection.event.line}",
            "",
            "For unfiltered original lines from this coordinator only:",
            "  make logs VIEW=raw",
            "The support bundle retains the related raw logs and runtime context.",
        ]
    )
    return "\n".join(lines) + "\n"


def _render_raw_view(
    selection: Selection | None,
    candidates: list[Selection],
    log_dir: Path,
    max_lines: int,
) -> str:
    if selection is None:
        return (
            "PS-OCTA/OCE raw primary log\n\n"
            "No coordinator session was found, so there is no primary INFO log to show.\n"
        )

    source = selection.session.coordinator.path
    try:
        original_lines = source.read_text(
            encoding="utf-8", errors="replace"
        ).splitlines(keepends=True)
    except OSError as error:
        return (
            "PS-OCTA/OCE raw primary log\n"
            f"Source: {source.resolve()}\n"
            f"Unable to read source: {error}\n"
        )

    total_lines = len(original_lines)
    focus_event: Event | None = None
    if selection.anchor.path == source:
        focus_event = selection.anchor
    elif selection.event.path == source:
        focus_event = selection.event

    focus_line = focus_event.line if focus_event is not None else total_lines
    if total_lines <= max_lines:
        start_line = 1
        end_line = total_lines
    else:
        context_before = min(100, max(1, max_lines // 5))
        start_line = max(1, focus_line - context_before)
        end_line = min(total_lines, start_line + max_lines - 1)
        if end_line == total_lines:
            start_line = max(1, end_line - max_lines + 1)

    leading_omitted = max(0, start_line - 1)
    trailing_omitted = max(0, total_lines - end_line)
    completeness = (
        "complete file" if not (leading_omitted or trailing_omitted) else "clipped"
    )
    focus_description = (
        f"selection anchor is physical line {focus_line}"
        if focus_event is not None
        else "selection event is in the launch log; showing the coordinator tail"
    )
    header = [
        "PS-OCTA/OCE raw primary coordinator log",
        f"Reason: {selection.reason}",
        *_session_selection_lines(selection, candidates, log_dir),
        f"Source: {source.resolve()}",
        (
            f"Clip: physical lines {start_line}-{end_line} of {total_lines} "
            f"({completeness}; {leading_omitted} before and {trailing_omitted} after omitted)"
        ),
        f"Basis: {focus_description}",
        f"Limit: {max_lines} lines (override with LOG_RAW_LINES=<count>)",
        "----- BEGIN ORIGINAL LOG -----",
    ]
    content = "".join(original_lines[start_line - 1 : end_line])
    if content and not content.endswith("\n"):
        content += "\n"
    return "\n".join(header) + "\n" + content + "----- END ORIGINAL LOG -----\n"


def _copy_log_snapshot(
    source: Path,
    destination: Path,
    anchor_time: float,
    max_file_bytes: int,
) -> str:
    destination.parent.mkdir(parents=True, exist_ok=True)
    size = source.stat().st_size
    if size <= max_file_bytes:
        with source.open("rb") as source_handle:
            snapshot = source_handle.read(max_file_bytes + 1)
        if len(snapshot) <= max_file_bytes:
            destination.write_bytes(snapshot)
            try:
                shutil.copystat(source, destination)
            except OSError:
                pass
            return "full"
        size = max(size, len(snapshot))

    excerpt_destination = destination.with_name(destination.name + ".incident-excerpt")
    window_start = anchor_time - 10 * 60
    window_end = anchor_time + 20 * 60
    current_timestamp: float | None = None
    header = (
        f"# Source was {size} bytes; this is the incident window "
        f"{_format_time(window_start)} to {_format_time(window_end)}.\n"
    ).encode()
    limit_marker = b"# Excerpt reached the per-file size limit.\n"
    copied = min(len(header), max_file_bytes)
    with source.open("r", encoding="utf-8", errors="replace") as source_handle:
        with excerpt_destination.open("wb") as output:
            output.write(header[:max_file_bytes])
            for line in source_handle:
                ros_match = ROS_HEADER_RE.match(line)
                launch_match = LAUNCH_HEADER_RE.match(line)
                if ros_match:
                    current_timestamp = float(ros_match.group("timestamp"))
                elif launch_match:
                    current_timestamp = float(launch_match.group("timestamp"))
                if (
                    current_timestamp is not None
                    and window_start <= current_timestamp <= window_end
                ):
                    encoded_line = line.encode("utf-8", errors="replace")
                    remaining = max_file_bytes - copied
                    if len(encoded_line) > remaining:
                        if remaining >= len(limit_marker):
                            output.write(limit_marker)
                            copied += len(limit_marker)
                        break
                    output.write(encoded_line)
                    copied += len(encoded_line)
    return f"excerpt:{excerpt_destination.name}"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _unique_bundle_path(output_dir: Path, suffix: str) -> Path:
    timestamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")
    base = output_dir / f"support-{timestamp}-{suffix}.tar.gz"
    if not base.exists():
        return base
    counter = 2
    while True:
        candidate = output_dir / f"support-{timestamp}-{suffix}-{counter}.tar.gz"
        if not candidate.exists():
            return candidate
        counter += 1


def create_bundle(args: argparse.Namespace) -> tuple[Path, str]:
    log_dir = args.log_dir.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    sessions, launch_logs = _build_sessions(log_dir) if log_dir.is_dir() else ([], [])
    candidates = select_sessions(sessions, args.incident, args.match)
    if candidates and args.session >= len(candidates):
        raise ValueError(
            f"SESSION={args.session} is out of range; only {len(candidates)} matching session(s) found"
        )
    selection = candidates[args.session] if candidates else None
    if sessions and selection is None and (args.match or args.incident != "auto"):
        requested = (
            f"MATCH={args.match!r}" if args.match else f"INCIDENT={args.incident}"
        )
        raise ValueError(f"no coordinator session matched {requested}")

    manager = None
    related_sessions: list[Session] = []
    raw_files: list[Path] = []
    if selection is not None:
        context_start = (
            min(selection.anchor.timestamp, selection.event.timestamp) - 5 * 60
        )
        context_end = (
            max(selection.anchor.timestamp, selection.event.timestamp) + 2 * 60
        )
        related_sessions = _related_sessions(selection, sessions)
        manager = _find_manager_log(log_dir, context_start, context_end)
        related_files: list[Path] = _session_files(log_dir, selection.session)
        for related_session in related_sessions:
            if related_session is selection.session:
                continue
            related_files.append(related_session.coordinator.path)
            if related_session.launch is not None:
                related_files.append(related_session.launch.path)
        if manager is not None:
            related_files.append(manager.path)
        raw_files = list(dict.fromkeys(related_files))
    elif launch_logs:
        latest_launch = max(launch_logs, key=lambda item: item.end_time)
        raw_files = [latest_launch.path]

    report = _render_report(selection, candidates, log_dir)
    terminal_output = (
        report
        if args.view == "summary"
        else _render_raw_view(selection, candidates, log_dir, args.raw_lines)
    )
    suffix = "no-session"
    anchor_time = datetime.now().timestamp()
    if selection is not None:
        suffix = f"{args.incident}-pid{selection.session.pid or 'unknown'}"
        anchor_time = selection.anchor.timestamp
    archive_path = _unique_bundle_path(output_dir, suffix)

    repo_root = Path(__file__).resolve().parents[1]
    docker_since = (
        min(selection.anchor.timestamp, selection.event.timestamp) - 5 * 60
        if selection
        else None
    )
    docker_until = (
        max(selection.anchor.timestamp, selection.event.timestamp) + 2 * 60
        if selection
        else None
    )
    runtime_context, docker_log = _runtime_context(
        repo_root,
        args.container,
        include_docker=not args.no_runtime,
        since=docker_since,
        until=docker_until,
    )

    with tempfile.TemporaryDirectory(prefix=".support-", dir=output_dir) as temp_name:
        staging = Path(temp_name) / "support"
        staging.mkdir()
        (staging / "report.txt").write_text(report, encoding="utf-8")
        runtime_dir = staging / "runtime"
        runtime_dir.mkdir()
        (runtime_dir / "context.txt").write_text(runtime_context, encoding="utf-8")
        (runtime_dir / "docker.log").write_text(docker_log, encoding="utf-8")

        captured_logs: list[tuple[Path, Path, str]] = []
        skipped_manifest_lines: list[str] = []
        if selection is not None:
            raw_files.sort(key=lambda path: path != selection.session.coordinator.path)
        captured_log_bytes = 0
        for source in raw_files:
            try:
                relative = source.resolve().relative_to(log_dir)
            except (OSError, ValueError):
                continue
            remaining_bytes = args.max_bundle_bytes - captured_log_bytes
            if remaining_bytes < 1024:
                skipped_manifest_lines.append(
                    f"-  0  skipped:staging-limit  {relative}"
                )
                continue
            destination = staging / "logs" / relative
            try:
                capture = _copy_log_snapshot(
                    source,
                    destination,
                    anchor_time,
                    min(args.max_file_bytes, remaining_bytes),
                )
            except OSError as error:
                skipped_manifest_lines.append(
                    f"-  0  skipped:unavailable  {relative} ({error})"
                )
                continue
            captured_path = destination
            if capture.startswith("excerpt:"):
                captured_path = destination.with_name(
                    destination.name + ".incident-excerpt"
                )
            captured_logs.append((relative, captured_path, capture))
            captured_log_bytes += captured_path.stat().st_size

        temporary_archive = archive_path.with_name(archive_path.name + ".tmp")
        primary_relative = None
        if selection is not None:
            try:
                primary_relative = (
                    selection.session.coordinator.path.resolve().relative_to(log_dir)
                )
            except (OSError, ValueError):
                pass
        while True:
            manifest_lines = ["sha256  bytes  capture  source"]
            for relative, captured_path, capture in captured_logs:
                manifest_lines.append(
                    f"{_sha256(captured_path)}  {captured_path.stat().st_size}  "
                    f"{capture}  {relative}"
                )
            manifest_lines.extend(skipped_manifest_lines)
            (staging / "manifest.txt").write_text(
                "\n".join(manifest_lines) + "\n", encoding="utf-8"
            )

            with tarfile.open(temporary_archive, "w:gz") as archive:
                archive.add(staging, arcname="support")
            archive_size = temporary_archive.stat().st_size
            if archive_size <= args.max_bundle_bytes:
                os.replace(temporary_archive, archive_path)
                break

            if not captured_logs:
                temporary_archive.unlink(missing_ok=True)
                raise ValueError(
                    "--max-bundle-bytes (LOG_MAX_BUNDLE_BYTES) is too small for "
                    "the report and runtime context "
                    f"({archive_size} compressed bytes required without raw logs)"
                )

            remove_index = max(
                range(len(captured_logs)),
                key=lambda index: (
                    captured_logs[index][0] != primary_relative,
                    captured_logs[index][1].stat().st_size,
                ),
            )
            relative, captured_path, _ = captured_logs.pop(remove_index)
            captured_path.unlink()
            skipped_manifest_lines.append(
                f"-  0  skipped:compressed-bundle-limit  {relative}"
            )

    return archive_path, terminal_output


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Print one incident summary or one contiguous raw coordinator-log clip, "
            "and create a support bundle."
        )
    )
    parser.add_argument("--log-dir", type=Path, default=Path("logs"))
    parser.add_argument("--output-dir", type=Path, default=Path("logs/support-bundles"))
    parser.add_argument(
        "--incident",
        choices=("auto", "fullscan", "error", "latest"),
        default="auto",
        help="auto prefers incidents over later clean-shutdown sessions",
    )
    parser.add_argument(
        "--match", help="case-insensitive regular expression overriding --incident"
    )
    parser.add_argument(
        "--session",
        type=int,
        default=0,
        help="matching session offset (0 newest, 1 previous, ...)",
    )
    parser.add_argument(
        "--view",
        choices=("summary", "raw"),
        default="summary",
        help="summary is concise; raw prints one verbatim coordinator-log clip",
    )
    parser.add_argument(
        "--raw-lines",
        type=int,
        default=2000,
        help="maximum physical lines printed by --view raw",
    )
    parser.add_argument(
        "--max-file-bytes",
        type=int,
        default=25 * 1024 * 1024,
        help="maximum bytes copied from any one raw log before excerpting it",
    )
    parser.add_argument(
        "--max-bundle-bytes",
        type=int,
        default=100 * 1024 * 1024,
        help="hard maximum size of the final compressed support bundle",
    )
    parser.add_argument("--container", default="ps-oce-robot")
    parser.add_argument(
        "--no-runtime",
        action="store_true",
        help="skip Docker queries (primarily useful for tests)",
    )
    args = parser.parse_args(argv)
    if args.session < 0:
        parser.error("--session must be non-negative")
    if args.raw_lines < 1:
        parser.error("--raw-lines must be positive")
    if args.max_file_bytes < 1024:
        parser.error("--max-file-bytes must be at least 1024")
    if args.max_bundle_bytes < 1024:
        parser.error("--max-bundle-bytes must be at least 1024")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        archive_path, terminal_output = create_bundle(args)
    except ValueError as error:
        print(f"support_logs: {error}", file=sys.stderr)
        return 2
    print(terminal_output, end="")
    print(f"Support bundle: {archive_path}")
    print("Bundle report.txt contains the concise incident summary.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
