#!/usr/bin/env python3
"""Own the mapping launch process and finish saving before releasing its lock."""

import argparse
import os
import signal
import subprocess
import sys

import yaml

from map_manifest import record_map


def report(message, stream=None):
    try:
        print(message, file=stream, flush=True)
    except (OSError, ValueError):
        pass


def run_session(command, manifest_path, yaml_path, pcd_path):
    process = None
    stop_requested = False
    signal_sent = False
    previous_handlers = {}

    def forward_stop():
        nonlocal signal_sent
        if process is not None and process.poll() is None and not signal_sent:
            signal_sent = True
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            report("Stopping mapping; waiting for PGM/YAML and PCD to finish saving.")

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True
        forward_stop()

    try:
        handled_signals = [signal.SIGINT, signal.SIGTERM]
        if hasattr(signal, "SIGHUP"):
            handled_signals.append(signal.SIGHUP)
        for signum in handled_signals:
            previous_handlers[signum] = signal.signal(signum, request_stop)
        process = subprocess.Popen(command, start_new_session=True)
        if stop_requested:
            forward_stop()
        status = process.wait()
        if status not in (0, 130, 143, -signal.SIGINT, -signal.SIGTERM) and not stop_requested:
            report(f"Mapping exited abnormally ({status}); previous map remains selected.", sys.stderr)
            return 1
        try:
            record_map(manifest_path, yaml_path, pcd_path)
        except (OSError, ValueError, KeyError, IndexError, TypeError, yaml.YAMLError) as exc:
            report(f"Map files are incomplete; previous map remains selected: {exc}", sys.stderr)
            return 1
        report(f"Map saved: {yaml_path}\nPoint cloud saved: {pcd_path}")
        report("The next navigation or simulation start will use this validated map.")
        return 0
    finally:
        if process is not None and process.poll() is None:
            forward_stop()
            process.wait()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--map", required=True)
    parser.add_argument("--pcd", required=True)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    options = parser.parse_args()
    command = options.command[1:] if options.command[:1] == ["--"] else options.command
    if not command:
        parser.error("mapping launch command is required after --")
    try:
        return run_session(command, options.manifest, options.map, options.pcd)
    except OSError as exc:
        report(f"Cannot run mapping: {exc}", sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
