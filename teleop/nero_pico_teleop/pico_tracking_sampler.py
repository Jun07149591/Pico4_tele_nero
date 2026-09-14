#!/usr/bin/env python3
"""Read-only PICO Tracking sampler.

This program subscribes to XRoboToolkit's PC SDK and reports the health of
tracking input. It deliberately does not call any device-control API and does
not open or configure the Nero CAN interface.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import queue
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .paths import pico_sdk_path


PXREA_SERVER_CONNECT = 1 << 2
PXREA_SERVER_DISCONNECT = 1 << 3
PXREA_DEVICE_FIND = 1 << 4
PXREA_DEVICE_MISSING = 1 << 5
PXREA_DEVICE_CONNECT = 1 << 9
PXREA_DEVICE_STATE_JSON = 1 << 25
PXREA_FULL_MASK = 0xFFFFFFFF


class PXREADevStateJson(ctypes.Structure):
    _fields_ = [
        ("devID", ctypes.c_char * 32),
        ("stateJson", ctypes.c_char * 16352),
    ]


Callback = ctypes.CFUNCTYPE(
    None,
    ctypes.c_void_p,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_void_p,
)


@dataclass
class Event:
    kind: str
    dev_id: str = ""
    status: int = 0
    payload: Any = None


def decode_c_string(value: bytes) -> str:
    return value.split(b"\0", 1)[0].decode("utf-8", errors="replace")


def parse_tracking(raw: str) -> dict[str, Any] | None:
    """Parse the SDK envelope and its value field without assuming its type."""
    try:
        envelope = json.loads(raw)
    except json.JSONDecodeError:
        return None
    if not isinstance(envelope, dict) or envelope.get("functionName") != "Tracking":
        return None
    value = envelope.get("value")
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except json.JSONDecodeError:
            # Some SDK versions return an escaped JSON string.
            try:
                value = json.loads(value.replace("\\", ""))
            except json.JSONDecodeError:
                return None
    return value if isinstance(value, dict) else None


def pose_is_default(pose: Any) -> bool:
    if isinstance(pose, str):
        try:
            values = [float(x.strip()) for x in pose.split(",")]
        except (TypeError, ValueError):
            return False
        return len(values) == 7 and all(abs(x) < 1e-7 for x in values[:6]) and abs(values[6] - 1.0) < 1e-7
    if isinstance(pose, (list, tuple)) and len(pose) == 7:
        try:
            values = [float(x) for x in pose]
        except (TypeError, ValueError):
            return False
        return all(abs(x) < 1e-7 for x in values[:6]) and abs(values[6] - 1.0) < 1e-7
    return False


def find_pose_values(value: Any) -> list[Any]:
    found: list[Any] = []
    if isinstance(value, dict):
        for key, item in value.items():
            if key.lower() == "pose":
                found.append(item)
            found.extend(find_pose_values(item))
    elif isinstance(value, list):
        for item in value:
            found.extend(find_pose_values(item))
    return found


def compact_tracking(value: dict[str, Any]) -> dict[str, Any]:
    timestamp = value.get("timeStampNs")
    focus = value.get("appState", {}).get("focus") if isinstance(value.get("appState"), dict) else None
    poses = find_pose_values(value)
    controller = value.get("Controller") if isinstance(value.get("Controller"), dict) else {}
    right = controller.get("right") if isinstance(controller.get("right"), dict) else {}
    return {
        "timestamp_ns": timestamp,
        "focus": focus,
        "top_level_keys": sorted(value.keys()),
        "pose_count": len(poses),
        "default_pose_count": sum(pose_is_default(pose) for pose in poses),
        "right_controller": {
            "pose": right.get("pose"),
            "trigger": right.get("trigger"),
            "grip": right.get("grip"),
            "primaryButton": right.get("primaryButton"),
            "secondaryButton": right.get("secondaryButton"),
            "menuButton": right.get("menuButton"),
            "axisX": right.get("axisX"),
            "axisY": right.get("axisY"),
            "axisClick": right.get("axisClick"),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sdk",
        default=str(pico_sdk_path()),
        help="path to libPXREARobotSDK.so",
    )
    parser.add_argument("--duration", type=float, default=30.0, help="采样秒数，0 表示持续运行")
    parser.add_argument("--stale-ms", type=float, default=250.0, help="超过此时间没有 Tracking 即视为 STALE")
    parser.add_argument("--print-every-ms", type=float, default=500.0, help="READY 状态的周期输出间隔")
    args = parser.parse_args()

    try:
        sdk = ctypes.CDLL(args.sdk)
    except OSError as exc:
        print(f"ERROR cannot load SDK: {exc}", file=sys.stderr)
        return 2

    sdk.PXREAInit.argtypes = [ctypes.c_void_p, Callback, ctypes.c_uint]
    sdk.PXREAInit.restype = ctypes.c_int
    sdk.PXREADeinit.argtypes = []
    sdk.PXREADeinit.restype = ctypes.c_int

    events: queue.Queue[Event] = queue.Queue()

    @Callback
    def callback(_context: int, event_type: int, status: int, user_data: int) -> None:
        try:
            if event_type == PXREA_SERVER_CONNECT:
                events.put(Event("server_connect", status=status))
            elif event_type == PXREA_SERVER_DISCONNECT:
                events.put(Event("server_disconnect", status=status))
            elif event_type in (PXREA_DEVICE_FIND, PXREA_DEVICE_MISSING, PXREA_DEVICE_CONNECT):
                dev_id = decode_c_string(ctypes.cast(user_data, ctypes.c_char_p).value or b"")
                kind = {
                    PXREA_DEVICE_FIND: "device_find",
                    PXREA_DEVICE_MISSING: "device_missing",
                    PXREA_DEVICE_CONNECT: "device_connect",
                }[event_type]
                events.put(Event(kind, dev_id=dev_id, status=status))
            elif event_type == PXREA_DEVICE_STATE_JSON and user_data:
                state = ctypes.cast(user_data, ctypes.POINTER(PXREADevStateJson)).contents
                raw = decode_c_string(bytes(state.stateJson))
                value = parse_tracking(raw)
                if value is not None:
                    events.put(Event("tracking", dev_id=decode_c_string(bytes(state.devID)), payload=value))
        except Exception as exc:  # Never allow an exception to escape the C callback.
            events.put(Event("callback_error", payload=str(exc)))

    init_result = sdk.PXREAInit(None, callback, PXREA_FULL_MASK)
    if init_result != 0:
        print(json.dumps({"state": "ERROR", "reason": "PXREAInit", "code": init_result}, ensure_ascii=False))
        return 3

    stop = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    server_online = False
    device_online = False
    last_tracking_monotonic: float | None = None
    last_tracking: dict[str, Any] | None = None
    last_print = 0.0
    last_state = "DISCONNECTED"
    start = time.monotonic()

    def report(state: str, reason: str, summary: dict[str, Any] | None = None) -> None:
        nonlocal last_state, last_print
        now = time.monotonic()
        if state != last_state or now - last_print >= args.print_every_ms / 1000.0:
            record: dict[str, Any] = {"state": state, "reason": reason}
            if summary is not None:
                record["tracking"] = summary
            print(json.dumps(record, ensure_ascii=False), flush=True)
            last_state = state
            last_print = now

    try:
        while not stop and (args.duration <= 0 or time.monotonic() - start < args.duration):
            try:
                event = events.get(timeout=0.05)
            except queue.Empty:
                event = None
            if event is not None:
                if event.kind == "server_connect":
                    server_online = True
                    report("DISCONNECTED", "server_connected_waiting_for_tracking")
                elif event.kind in ("server_disconnect", "device_missing"):
                    server_online = event.kind != "server_disconnect"
                    device_online = False
                    report("DISCONNECTED", event.kind)
                elif event.kind in ("device_find", "device_connect"):
                    device_online = True
                    report("DISCONNECTED", f"{event.kind}:{event.dev_id}")
                elif event.kind == "tracking":
                    device_online = True
                    last_tracking_monotonic = time.monotonic()
                    last_tracking = event.payload
                elif event.kind == "callback_error":
                    report("INVALID", f"callback_error:{event.payload}")

            if last_tracking_monotonic is None:
                report("DISCONNECTED" if not server_online else "DISCONNECTED", "waiting_for_tracking")
                continue

            age_ms = (time.monotonic() - last_tracking_monotonic) * 1000.0
            if age_ms > args.stale_ms:
                report("STALE", f"tracking_age_ms={age_ms:.1f}")
                continue

            summary = compact_tracking(last_tracking or {})
            focus = summary["focus"]
            defaults = summary["default_pose_count"]
            if focus is False:
                report("INVALID", "app_focus_false", summary)
            elif summary["pose_count"] and defaults == summary["pose_count"]:
                report("INVALID", "all_poses_default", summary)
            else:
                report("READY", "fresh_tracking", summary)
    finally:
        sdk.PXREADeinit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
