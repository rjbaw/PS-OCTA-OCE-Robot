import importlib.util
import os
import subprocess
import sys
import tarfile
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "utils" / "support_logs.py"


class SupportLogsTest(unittest.TestCase):
    def _write_session(
        self,
        log_dir: Path,
        *,
        start: float,
        pid: int,
        fullscan: bool,
        crash: bool,
        clean_shutdown: bool = False,
        required_crash: str | None = None,
        worker_pid: int | None = None,
        worker_start_offset: float = 0.6,
    ) -> tuple[Path, Path]:
        stamp_ms = int(start * 1000)
        coordinator = log_dir / f"coordinator_node_{pid}_{stamp_ms}.log"
        coordinator_lines = [
            f"[INFO] [{start + 1:.9f}] [coordinator_node]: Coordinator Node Initialized.",
            f"[INFO] [{start + 2:.9f}] [coordinator_node]: [SUBSCRIBING]: Changed fields",
            " scan_trigger: 1",
        ]
        if fullscan:
            coordinator_lines.extend([" scan_3d: 1", " full_scan: 1"])
        coordinator_lines.extend(
            [
                f"[INFO] [{start + 2.2:.9f}] [coordinator_node]: [Action] Focusing",
                f"[INFO] [{start + 2.5:.9f}] [coordinator_node]: Focus goal accepted; waiting for result",
            ]
        )
        coordinator.write_text("\n".join(coordinator_lines) + "\n", encoding="utf-8")

        launch_dir = log_dir / f"session-{pid}-{stamp_ms}"
        launch_dir.mkdir()
        launch = launch_dir / "launch.log"
        launch_lines = [
            f"{start:.6f} [INFO] [launch]: Default logging verbosity is set to INFO",
            f"{start + 0.5:.6f} [INFO] [coordinator_node-14]: process started with pid [{pid}]",
        ]
        if worker_pid is not None:
            launch_lines.append(
                f"{start + worker_start_offset:.6f} [INFO] [freedrive_node-15]: "
                f"process started with pid [{worker_pid}]"
            )
        if crash:
            launch_lines.append(
                f"{start + 8:.6f} [ERROR] [coordinator_node-14]: process has died "
                f"[pid {pid}, exit code -6, cmd '/app/coordinator_node']."
            )
            launch_lines.append(
                f"{start + 8.1:.6f} [INFO] [launch]: process[coordinator_node-14] "
                "was required: shutting down launched system"
            )
        elif required_crash:
            launch_lines.extend(
                [
                    f"{start + 8:.6f} [ERROR] [{required_crash}]: process has died "
                    "[pid 777, exit code -6, cmd '/app/required_node'].",
                    f"{start + 8.1:.6f} [INFO] [launch]: process[{required_crash}] "
                    "was required: shutting down launched system",
                ]
            )
        elif clean_shutdown:
            launch_lines.extend(
                [
                    f"{start + 20:.6f} [WARNING] [launch]: user interrupted with ctrl-c (SIGINT)",
                    f"{start + 20.1:.6f} [ERROR] [move_group-12]: process has died "
                    "[pid 999, exit code -11, cmd '/app/move_group'].",
                    f"{start + 20.2:.6f} [INFO] [coordinator_node-14]: process has finished cleanly "
                    f"[pid {pid}]",
                ]
            )
        launch.write_text("\n".join(launch_lines) + "\n", encoding="utf-8")
        return coordinator, launch

    def _run(
        self, log_dir: Path, output_dir: Path, *extra: str
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "--log-dir",
                str(log_dir),
                "--output-dir",
                str(output_dir),
                "--no-runtime",
                *extra,
            ],
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
        )

    def _run_prune(self, log_dir: Path) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                "make",
                "-s",
                "prune-logs",
                f"LOG_DIR={log_dir}",
                "PRUNE_LOGS_DAYS=0",
            ],
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
        )

    def test_prune_logs_requires_a_dedicated_directory(self):
        with tempfile.TemporaryDirectory() as temp_name:
            broad_directory = Path(temp_name)
            sentinel = broad_directory / "keep.log"
            sentinel.write_text("keep\n", encoding="utf-8")

            completed = self._run_prune(broad_directory)

            self.assertEqual(completed.returncode, 2)
            self.assertIn("Refusing unsafe LOG_DIR", completed.stderr)
            self.assertTrue(sentinel.exists())

    def test_prune_logs_removes_old_entries_from_a_dedicated_directory(self):
        with tempfile.TemporaryDirectory() as temp_name:
            log_dir = Path(temp_name) / "robot" / "logs"
            log_dir.mkdir(parents=True)
            old_log = log_dir / "old.log"
            old_log.write_text("old\n", encoding="utf-8")
            os.utime(old_log, (1, 1))

            completed = self._run_prune(log_dir)

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertFalse(old_log.exists())

    def test_prune_logs_keeps_recent_files_in_an_old_session_directory(self):
        with tempfile.TemporaryDirectory() as temp_name:
            log_dir = Path(temp_name) / "robot" / "logs"
            session_dir = log_dir / "long-running-session"
            session_dir.mkdir(parents=True)
            current_log = session_dir / "coordinator.log"
            current_log.write_text("still running\n", encoding="utf-8")
            os.utime(session_dir, (1, 1))

            completed = self._run_prune(log_dir)

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertTrue(current_log.exists())

    def test_auto_prefers_incident_over_newer_clean_shutdown(self):
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            log_dir = temp / "logs"
            output_dir = temp / "bundles"
            log_dir.mkdir()
            related_coordinator, related_launch = self._write_session(
                log_dir,
                start=1_699_999_850.0,
                pid=77,
                fullscan=False,
                crash=True,
            )
            incident_coordinator, incident_launch = self._write_session(
                log_dir,
                start=1_700_000_000.0,
                pid=101,
                fullscan=True,
                crash=True,
            )
            newer_coordinator, newer_launch = self._write_session(
                log_dir,
                start=1_700_001_000.0,
                pid=202,
                fullscan=False,
                crash=False,
                clean_shutdown=True,
            )

            completed = self._run(log_dir, output_dir)

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn(
                "Reason: coordinator crash after fullscan request", completed.stdout
            )
            self.assertIn(incident_coordinator.name, completed.stdout)
            self.assertNotIn(newer_coordinator.name, completed.stdout)
            self.assertIn(
                "Oldest: SESSION=1, started ",
                completed.stdout,
            )
            self.assertIn(related_coordinator.name, completed.stdout)
            self.assertNotIn("move_group-12", completed.stdout)
            self.assertNotIn("RELATED RESTART WINDOW", completed.stdout)
            self.assertIn("make logs VIEW=raw", completed.stdout)

            archives = list(output_dir.glob("*.tar.gz"))
            self.assertEqual(len(archives), 1)
            with tarfile.open(archives[0], "r:gz") as archive:
                names = set(archive.getnames())
                report = archive.extractfile("support/report.txt").read().decode()
            self.assertIn(f"support/logs/{incident_coordinator.name}", names)
            self.assertIn(
                f"support/logs/{incident_launch.parent.name}/launch.log", names
            )
            self.assertIn(f"support/logs/{related_coordinator.name}", names)
            self.assertIn(
                f"support/logs/{related_launch.parent.name}/launch.log", names
            )
            self.assertNotIn(
                f"support/logs/{newer_launch.parent.name}/launch.log", names
            )
            self.assertIn("full_scan: 1", report)

    def test_raw_view_is_one_contiguous_verbatim_coordinator_clip(self):
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            log_dir = temp / "logs"
            output_dir = temp / "bundles"
            log_dir.mkdir()
            coordinator, _ = self._write_session(
                log_dir,
                start=1_700_000_000.0,
                pid=101,
                fullscan=True,
                crash=True,
            )

            completed = self._run(
                log_dir,
                output_dir,
                "--incident",
                "latest",
                "--view",
                "raw",
                "--raw-lines",
                "4",
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("Selected: SESSION=0 of 1", completed.stdout)
            self.assertIn(
                "Matching range (chronological, oldest -> newest):",
                completed.stdout,
            )
            self.assertIn("Oldest: SESSION=0, started ", completed.stdout)
            self.assertIn("Newest: SESSION=0 (default), started ", completed.stdout)
            self.assertIn(f"Source: {coordinator.resolve()}", completed.stdout)
            self.assertIn(
                "Clip: physical lines 4-7 of 7 (clipped; 3 before and 0 after omitted)",
                completed.stdout,
            )
            expected_clip = "".join(
                coordinator.read_text(encoding="utf-8").splitlines(keepends=True)[3:7]
            )
            self.assertIn(
                "----- BEGIN ORIGINAL LOG -----\n"
                + expected_clip
                + "----- END ORIGINAL LOG -----",
                completed.stdout,
            )
            self.assertNotIn("process started with pid", completed.stdout)
            archives = list(output_dir.glob("*.tar.gz"))
            self.assertEqual(len(archives), 1)
            with tarfile.open(archives[0], "r:gz") as archive:
                report = archive.extractfile("support/report.txt").read().decode()
            self.assertIn("PS-OCTA/OCE incident summary", report)
            self.assertNotIn("BEGIN ORIGINAL LOG", report)

    def test_latest_can_intentionally_select_clean_shutdown_session(self):
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            log_dir = temp / "logs"
            output_dir = temp / "bundles"
            log_dir.mkdir()
            self._write_session(
                log_dir,
                start=1_700_000_000.0,
                pid=101,
                fullscan=True,
                crash=True,
            )
            newer_coordinator, _ = self._write_session(
                log_dir,
                start=1_700_001_000.0,
                pid=202,
                fullscan=False,
                crash=False,
                clean_shutdown=True,
            )

            completed = self._run(log_dir, output_dir, "--incident", "latest")

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("Reason: latest coordinator activity", completed.stdout)
            self.assertIn(newer_coordinator.name, completed.stdout)

    def test_session_range_is_chronological_while_zero_selects_newest(self):
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            log_dir = temp / "logs"
            output_dir = temp / "bundles"
            log_dir.mkdir()
            oldest, _ = self._write_session(
                log_dir,
                start=1_700_000_000.0,
                pid=101,
                fullscan=True,
                crash=False,
            )
            selected, _ = self._write_session(
                log_dir,
                start=1_700_001_000.0,
                pid=202,
                fullscan=True,
                crash=False,
            )
            newest, _ = self._write_session(
                log_dir,
                start=1_700_002_000.0,
                pid=303,
                fullscan=True,
                crash=False,
            )

            previous = self._run(
                log_dir,
                output_dir,
                "--incident",
                "fullscan",
                "--session",
                "1",
            )
            default = self._run(log_dir, output_dir, "--incident", "fullscan")

            self.assertEqual(previous.returncode, 0, previous.stderr)
            self.assertIn("Selected: SESSION=1 of 3", previous.stdout)
            self.assertIn(selected.name, previous.stdout)
            oldest_text = next(
                line
                for line in previous.stdout.splitlines()
                if line.startswith("  Oldest:")
            )
            newest_text = next(
                line
                for line in previous.stdout.splitlines()
                if line.startswith("  Newest:")
            )
            self.assertIn("Oldest: SESSION=2, started ", oldest_text)
            self.assertIn(f"logs/{oldest.name}", oldest_text)
            self.assertIn("Newest: SESSION=0 (default), started ", newest_text)
            self.assertIn(f"logs/{newest.name}", newest_text)
            oldest_line = previous.stdout.index(oldest_text)
            newest_line = previous.stdout.index(newest_text)
            self.assertLess(oldest_line, newest_line)

            self.assertEqual(default.returncode, 0, default.stderr)
            self.assertIn("Selected: SESSION=0 of 3", default.stdout)
            self.assertIn(
                f"Primary INFO log: logs/{newest.name}",
                default.stdout,
            )

    def test_auto_selects_required_non_coordinator_crash(self):
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            log_dir = temp / "logs"
            output_dir = temp / "bundles"
            log_dir.mkdir()
            self._write_session(
                log_dir,
                start=1_700_000_000.0,
                pid=101,
                fullscan=True,
                crash=True,
            )
            newer_coordinator, _ = self._write_session(
                log_dir,
                start=1_700_001_000.0,
                pid=202,
                fullscan=False,
                crash=False,
                required_crash="freedrive_node-15",
            )

            completed = self._run(log_dir, output_dir)

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn(
                "Reason: required process crash (freedrive_node-15)",
                completed.stdout,
            )
            self.assertIn(newer_coordinator.name, completed.stdout)

    def test_required_process_death_after_ctrl_c_is_shutdown_noise(self):
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            log_dir = temp / "logs"
            output_dir = temp / "bundles"
            log_dir.mkdir()
            coordinator, launch = self._write_session(
                log_dir,
                start=1_700_001_000.0,
                pid=202,
                fullscan=False,
                crash=False,
                clean_shutdown=True,
            )
            with launch.open("a", encoding="utf-8") as handle:
                handle.write(
                    "1700001021.000000 [ERROR] [freedrive_node-15]: "
                    "process has died [pid 777, exit code -6, cmd '/app/freedrive'].\n"
                    "1700001021.100000 [INFO] [launch]: "
                    "process[freedrive_node-15] was required: "
                    "shutting down launched system\n"
                )

            completed = self._run(log_dir, output_dir)

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("Reason: latest coordinator activity", completed.stdout)
            self.assertIn(coordinator.name, completed.stdout)
            self.assertNotIn("required process crash", completed.stdout)

    def test_session_end_uses_coordinator_when_launch_log_is_truncated(self):
        with tempfile.TemporaryDirectory() as temp_name:
            log_dir = Path(temp_name) / "logs"
            log_dir.mkdir()
            coordinator, _ = self._write_session(
                log_dir,
                start=1_700_000_000.0,
                pid=101,
                fullscan=True,
                crash=False,
            )
            with coordinator.open("a", encoding="utf-8") as handle:
                handle.write(
                    "[INFO] [1700000100.000000000] [coordinator_node]: "
                    "[Fullscan] COMPLETE: completed_steps=41\n"
                )

            spec = importlib.util.spec_from_file_location(
                "support_logs_for_test", SCRIPT
            )
            module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = module
            try:
                spec.loader.exec_module(module)
                sessions, _ = module._build_sessions(log_dir)
            finally:
                sys.modules.pop(spec.name, None)

            self.assertEqual(len(sessions), 1)
            self.assertEqual(sessions[0].end_time, 1_700_000_100.0)

    def test_reused_pids_match_the_launch_and_process_logs_by_start_time(self):
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            log_dir = temp / "logs"
            output_dir = temp / "bundles"
            log_dir.mkdir()
            old_start = 1_700_000_000.0
            new_start = 1_700_001_000.0
            coordinator_pid = 101
            worker_pid = 404
            old_coordinator, old_launch = self._write_session(
                log_dir,
                start=old_start,
                pid=coordinator_pid,
                fullscan=True,
                crash=False,
                worker_pid=worker_pid,
                worker_start_offset=12.0,
            )
            with old_coordinator.open("a", encoding="utf-8") as handle:
                handle.write(
                    f"[ERROR] [{old_start + 20:.9f}] [coordinator_node]: "
                    "trajectory failed\n"
                )
            old_worker = (
                log_dir
                / f"freedrive_node_{worker_pid}_{int((old_start + 12) * 1000)}.log"
            )
            old_worker.write_text(
                f"[INFO] [{old_start + 12:.9f}] [freedrive_node]: old run\n",
                encoding="utf-8",
            )

            _, new_launch = self._write_session(
                log_dir,
                start=new_start,
                pid=coordinator_pid,
                fullscan=False,
                crash=False,
                clean_shutdown=True,
                worker_pid=worker_pid,
                worker_start_offset=12.0,
            )
            new_worker = (
                log_dir
                / f"freedrive_node_{worker_pid}_{int((new_start + 12) * 1000)}.log"
            )
            new_worker.write_text(
                f"[INFO] [{new_start + 12:.9f}] [freedrive_node]: new run\n",
                encoding="utf-8",
            )

            completed = self._run(log_dir, output_dir, "--incident", "error")

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn(
                f"Launch log: logs/{old_launch.parent.name}/launch.log",
                completed.stdout,
            )
            self.assertNotIn(new_launch.parent.name, completed.stdout)
            archive_path = next(output_dir.glob("*.tar.gz"))
            with tarfile.open(archive_path, "r:gz") as archive:
                names = set(archive.getnames())
            self.assertIn(f"support/logs/{old_launch.parent.name}/launch.log", names)
            self.assertIn(f"support/logs/{old_worker.name}", names)
            self.assertNotIn(f"support/logs/{new_launch.parent.name}/launch.log", names)
            self.assertNotIn(f"support/logs/{new_worker.name}", names)

    def test_explicit_fullscan_marker_beats_later_blocked_high_reassertion(self):
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            log_dir = temp / "logs"
            output_dir = temp / "bundles"
            log_dir.mkdir()
            start = 1_700_000_000.0
            coordinator, _ = self._write_session(
                log_dir,
                start=start,
                pid=101,
                fullscan=False,
                crash=False,
            )
            with coordinator.open("a", encoding="utf-8") as handle:
                handle.write(
                    f"[INFO] [{start + 3:.9f}] [coordinator_node]: "
                    "[Fullscan] REQUEST_ACCEPTED: LabVIEW Fullscan ON\n"
                    f"[INFO] [{start + 3.1:.9f}] [coordinator_node]: "
                    "[Fullscan] START: total_steps=41\n"
                    f"[ERROR] [{start + 4:.9f}] [coordinator_node]: "
                    "Fullscan canceled after terminal Focus failure\n"
                    f"[INFO] [{start + 4.1:.9f}] [coordinator_node]: "
                    "[Fullscan] CANCEL: completed_steps=0 total_steps=41\n"
                    f"[WARN] [{start + 10:.9f}] [coordinator_node]: "
                    "[Blocked] Full Scan because Cancel is returning the robot to Ready.\n"
                    f"[INFO] [{start + 10.1:.9f}] [coordinator_node]: "
                    "[SUBSCRIBING]: Changed fields\n full_scan: 1\n"
                )

            completed = self._run(log_dir, output_dir, "--incident", "fullscan")

            self.assertEqual(completed.returncode, 0, completed.stderr)
            request_line = next(
                line
                for line in completed.stdout.splitlines()
                if line.strip().startswith("Fullscan request:")
            )
            self.assertIn("REQUEST_ACCEPTED", request_line)
            self.assertNotIn("[Blocked]", request_line)
            self.assertNotIn("[Blocked]", completed.stdout)
            self.assertIn("Fullscan CANCEL", completed.stdout)
            self.assertIn("terminal Focus failure", completed.stdout)

    def test_expected_cancellation_is_not_an_error_but_error_severity_is(self):
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            log_dir = temp / "logs"
            output_dir = temp / "bundles"
            log_dir.mkdir()
            start = 1_700_000_000.0
            coordinator, _ = self._write_session(
                log_dir,
                start=start,
                pid=101,
                fullscan=False,
                crash=False,
            )
            with coordinator.open("a", encoding="utf-8") as handle:
                handle.write(
                    f"[WARN] [{start + 3:.9f}] [coordinator_node]: "
                    "Goal canceled by the user\n"
                    f"[INFO] [{start + 4:.9f}] [coordinator_node]: "
                    "Action cancelled normally\n"
                )

            expected_cancel = self._run(log_dir, output_dir, "--incident", "error")
            self.assertEqual(expected_cancel.returncode, 2)
            self.assertIn("no coordinator session matched", expected_cancel.stderr)

            with coordinator.open("a", encoding="utf-8") as handle:
                handle.write(
                    f"[WARN] [{start + 4.5:.9f}] [coordinator_node]: Move CANCELED\n"
                )
            unexpected_cancel = self._run(log_dir, output_dir, "--incident", "error")
            self.assertEqual(unexpected_cancel.returncode, 0, unexpected_cancel.stderr)
            self.assertIn("Reason: coordinator failure", unexpected_cancel.stdout)

            with coordinator.open("a", encoding="utf-8") as handle:
                handle.write(
                    f"[ERROR] [{start + 5:.9f}] [coordinator_node]: "
                    "Goal canceled because the action server disconnected\n"
                )
            failed_cancel = self._run(log_dir, output_dir, "--incident", "error")
            self.assertEqual(failed_cancel.returncode, 0, failed_cancel.stderr)
            self.assertIn("Reason: coordinator failure", failed_cancel.stdout)

    def test_max_bundle_bytes_limits_the_final_compressed_archive(self):
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            log_dir = temp / "logs"
            output_dir = temp / "bundles"
            log_dir.mkdir()
            coordinator, _ = self._write_session(
                log_dir,
                start=1_700_000_000.0,
                pid=101,
                fullscan=False,
                crash=False,
            )
            with coordinator.open("ab") as handle:
                handle.write(os.urandom(256 * 1024))

            max_bundle_bytes = 8 * 1024
            completed = self._run(
                log_dir,
                output_dir,
                "--incident",
                "latest",
                "--max-bundle-bytes",
                str(max_bundle_bytes),
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            archive_path = next(output_dir.glob("*.tar.gz"))
            self.assertLessEqual(archive_path.stat().st_size, max_bundle_bytes)
            with tarfile.open(archive_path, "r:gz") as archive:
                manifest = archive.extractfile("support/manifest.txt").read().decode()
            self.assertRegex(
                manifest,
                r"skipped:(?:staging|compressed-bundle)-limit",
            )

    def test_custom_match_and_no_match_error_are_clear(self):
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            log_dir = temp / "logs"
            output_dir = temp / "bundles"
            log_dir.mkdir()
            coordinator, _ = self._write_session(
                log_dir,
                start=1_700_000_000.0,
                pid=101,
                fullscan=True,
                crash=False,
            )

            matched = self._run(log_dir, output_dir, "--match", "Focus goal accepted")
            missing = self._run(
                log_dir, output_dir, "--match", "definitely-not-present"
            )

            self.assertEqual(matched.returncode, 0, matched.stderr)
            self.assertIn(coordinator.name, matched.stdout)
            self.assertIn("custom match: Focus goal accepted", matched.stdout)
            self.assertEqual(missing.returncode, 2)
            self.assertIn("no coordinator session matched", missing.stderr)

    def test_empty_log_directory_still_creates_context_bundle(self):
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            log_dir = temp / "logs"
            output_dir = temp / "bundles"
            log_dir.mkdir()

            completed = self._run(log_dir, output_dir)

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("No coordinator session was found", completed.stdout)
            archives = list(output_dir.glob("*.tar.gz"))
            self.assertEqual(len(archives), 1)
            with tarfile.open(archives[0], "r:gz") as archive:
                self.assertIn("support/runtime/context.txt", archive.getnames())


if __name__ == "__main__":
    unittest.main()
