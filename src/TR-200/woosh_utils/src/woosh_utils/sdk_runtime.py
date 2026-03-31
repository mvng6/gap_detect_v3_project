#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Woosh SDK runtime ownership helpers."""

import os
import re
import subprocess
import sys


SDK_OWNER_PARAM = "/woosh/sdk_owner"


def get_process_name():
    """Return a short process name for log tagging."""
    for path in ("/proc/self/comm", "/proc/self/cmdline"):
        try:
            with open(path, "rb") as handle:
                raw = handle.read().strip()
            if not raw:
                continue
            if path.endswith("cmdline"):
                raw = raw.split(b"\0", 1)[0]
            name = os.path.basename(raw.decode("utf-8", errors="ignore"))
            if name:
                return name
        except Exception:
            pass

    if sys.argv and sys.argv[0]:
        return os.path.basename(sys.argv[0])
    return "unknown"


def build_sdk_owner_record(owner, identity, robot_ip, robot_port, caller):
    return {
        "pid": int(os.getpid()),
        "proc": get_process_name(),
        "identity": identity,
        "owner": owner,
        "caller": caller,
        "target": f"{robot_ip}:{robot_port}",
    }


def log_sdk_owner(log_fn, event, owner, identity, robot_ip, robot_port, caller, note=None):
    message = (
        f"[SDK_OWNER] event={event} pid={os.getpid()} proc={get_process_name()} "
        f"identity={identity} owner={owner} caller={caller} target={robot_ip}:{robot_port}"
    )
    if note:
        message += f" note={note}"
    log_fn(message)
    return message


def register_sdk_owner(rospy, owner, identity, robot_ip, robot_port, caller):
    record = build_sdk_owner_record(owner, identity, robot_ip, robot_port, caller)
    rospy.set_param(SDK_OWNER_PARAM, record)
    return record


def get_registered_sdk_owner(rospy):
    return rospy.get_param(SDK_OWNER_PARAM, None)


def clear_registered_sdk_owner(rospy):
    try:
        current = get_registered_sdk_owner(rospy)
        if current and int(current.get("pid", -1)) == os.getpid():
            rospy.delete_param(SDK_OWNER_PARAM)
    except Exception:
        pass


def current_process_is_registered_owner(rospy):
    record = get_registered_sdk_owner(rospy)
    if not isinstance(record, dict):
        return False, record
    return int(record.get("pid", -1)) == os.getpid(), record


def inspect_tcp_connections(port, target_ip=None):
    """Inspect TCP connections with `ss -tnpH`.

    Returns:
        list[str]: raw matching lines.
    Raises:
        RuntimeError: when `ss` fails unexpectedly.
    """
    result = subprocess.run(
        ["ss", "-tnpH"],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode not in (0, 1):
        raise RuntimeError(result.stderr.strip() or f"`ss` exited with {result.returncode}")

    matches = []
    for line in result.stdout.splitlines():
        stripped = line.strip()
        if not stripped or f":{port}" not in stripped:
            continue
        if target_ip and target_ip not in stripped:
            continue
        matches.append(stripped)
    return matches


def parse_connection_owners(lines):
    """Extract process / pid tuples from `ss -tnp` lines."""
    owners = []
    pattern = re.compile(r'users:\(\("([^"]+)",pid=(\d+)')
    for line in lines:
        match = pattern.search(line)
        if match:
            owners.append({
                "proc": match.group(1),
                "pid": int(match.group(2)),
                "line": line,
            })
        else:
            owners.append({
                "proc": "unknown",
                "pid": -1,
                "line": line,
            })
    return owners


def find_foreign_sdk_owners(port, target_ip=None, ignore_pids=None):
    """Return active SDK connection owners excluding ignored PIDs."""
    ignore = {int(pid) for pid in (ignore_pids or []) if pid is not None}
    owners = parse_connection_owners(inspect_tcp_connections(port, target_ip=target_ip))
    return [owner for owner in owners if owner["pid"] not in ignore]
