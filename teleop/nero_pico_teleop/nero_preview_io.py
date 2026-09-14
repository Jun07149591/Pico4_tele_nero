#!/usr/bin/env python3
"""Live input adapters for read-only Nero target preview. No motion sender."""

from __future__ import annotations

import ctypes
import math
import queue
import threading
import time
from pathlib import Path

from .paths import use_agx_sdk

from .pico_tracking_sampler import (
    Callback, PXREADevStateJson, PXREA_DEVICE_MISSING, PXREA_DEVICE_STATE_JSON,
    PXREA_SERVER_DISCONNECT, PXREA_FULL_MASK, parse_tracking,
)


# Nero API: 0 = disabled, 6 = terminate execution. Active/paused states are excluded.
INACTIVE_TEACH_STATUSES = (0, 6)


class IncoherentJointFeedback(ValueError):
    """Fresh joint groups do not yet form a consistent snapshot."""


def _joint_snapshot_once(arm, stale_s):
    # SDK get_joint_angles() timestamps only the last group. Check all four.
    groups = (("joint_12", ("joint_1", "joint_2")),
              ("joint_34", ("joint_3", "joint_4")),
              ("joint_56", ("joint_5", "joint_6")), ("joint_7", ("joint_7",)))
    def read_groups():
        stamps, values = [], []
        for name, fields in groups:
            group = getattr(arm._parser, name, None)
            if group is None:
                raise ValueError(f"waiting for Nero feedback group {name}")
            stamps.append(float(group.timestamp))
            values.extend(float(getattr(group.msg, field)) for field in fields)
        return stamps, values

    # SDK caches are updated in place; discard reads that visibly changed.
    stamps, values = read_groups()
    checked_stamps, checked_values = read_groups()
    now = time.time()
    if not all(math.isfinite(x) for x in stamps + values + checked_stamps + checked_values):
        raise ValueError("nonfinite Nero feedback")
    if any(not 0 <= now - stamp <= stale_s for stamp in stamps + checked_stamps):
        ages_ms = [round((now - stamp) * 1000, 3) for stamp in stamps]
        checked_ages_ms = [round((now - stamp) * 1000, 3) for stamp in checked_stamps]
        raise ValueError(f"stale Nero joint feedback: group_ages_ms={ages_ms}, "
                         f"checked_group_ages_ms={checked_ages_ms}, limit_ms={stale_s * 1000:.0f}")
    if stamps != checked_stamps or values != checked_values:
        raise IncoherentJointFeedback("Nero joint feedback changed during snapshot")
    span = max(stamps) - min(stamps)
    if span > 0.025:
        raise IncoherentJointFeedback(
            f"Nero joint feedback groups differ by over 25 ms (span={span * 1000:.3f} ms, "
            f"oldest_age={(now - min(stamps)) * 1000:.3f} ms)")
    return {"joints_rad": values, "group_timestamps_s": stamps}


def joint_snapshot(arm, stale_s=0.25, *, coherence_wait_s=0.):
    if not math.isfinite(coherence_wait_s) or not 0 <= coherence_wait_s <= .02:
        raise ValueError("feedback coherence wait must be between 0 and 20 ms")
    deadline = time.monotonic() + coherence_wait_s
    while True:
        try:
            return _joint_snapshot_once(arm, stale_s)
        except IncoherentJointFeedback as exc:
            if not coherence_wait_s:
                raise
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(min(.001, remaining))
            if time.monotonic() >= deadline:
                raise IncoherentJointFeedback(
                    f"{exc}; coherent feedback unavailable within {coherence_wait_s * 1000:.0f} ms") from exc


def robot_snapshot(arm, stale_s=0.25, *, coherence_wait_s=0.):
    result = joint_snapshot(arm, stale_s, coherence_wait_s=coherence_wait_s)

    def fresh(message, name):
        if message is None:
            raise ValueError(f"waiting for Nero {name}")
        age = time.time() - float(message.timestamp)
        if not math.isfinite(age) or not 0 <= age <= stale_s:
            raise ValueError(f"stale Nero {name}")
        return message.msg

    status_feedback = arm.get_arm_status()
    message = fresh(status_feedback, "arm status")
    status_timestamp = float(status_feedback.timestamp)
    status = {name: int(getattr(message, name)) for name in (
        "ctrl_mode", "arm_status", "mode_feedback", "teach_status", "motion_status", "err_code")}
    status_names = {name: getattr(getattr(message, name), "name", str(status[name]))
                    for name in ("ctrl_mode", "arm_status", "mode_feedback", "teach_status", "motion_status")}
    enabled, faults = [], []
    for index in range(1, 8):
        driver = fresh(arm.get_driver_states(index), f"joint {index} driver status")
        flags = driver.foc_status
        enabled.append(bool(flags.driver_enable_status))
        faults.append([name for name in ("voltage_too_low", "motor_overheating", "driver_overcurrent",
                       "driver_overheating", "collision_status", "driver_error_status", "stall_status")
                       if getattr(flags, name)])
    issues = []
    if status["ctrl_mode"] != 1:
        issues.append("not_in_can_control")
    if status["mode_feedback"] != 1:
        issues.append("not_in_joint_position_mode")
    if status["arm_status"] != 0 or status["err_code"] != 0:
        issues.append("arm_status_not_normal")
    if status["teach_status"] not in INACTIVE_TEACH_STATUSES:
        issues.append("teaching_not_inactive")
    if not all(enabled):
        issues.append("not_all_seven_joints_enabled")
    if any(faults):
        issues.append("joint_driver_fault")
    return {**result, "status": status, "status_names": status_names,
            "status_timestamp_s": status_timestamp,
            "joints_enabled": enabled, "joint_faults": faults,
            "joint_output_blockers": issues}


class PicoEvents:
    def __init__(self, sdk_path):
        self.sdk_path = sdk_path
        self.events = queue.Queue(maxsize=128)
        self.overflow = threading.Event()

    def __enter__(self):
        self.sdk = ctypes.CDLL(str(self.sdk_path))
        self.sdk.PXREAInit.argtypes = [ctypes.c_void_p, Callback, ctypes.c_uint]
        self.sdk.PXREAInit.restype = ctypes.c_int
        self.sdk.PXREADeinit.argtypes = []
        self.sdk.PXREADeinit.restype = ctypes.c_int

        @Callback
        def callback(_context, event_type, status, user_data):
            event = {"received_monotonic": time.monotonic(), "sdk_status": status}
            try:
                if event_type in (PXREA_SERVER_DISCONNECT, PXREA_DEVICE_MISSING):
                    event["kind"] = "disconnect"
                elif event_type == PXREA_DEVICE_STATE_JSON and user_data:
                    state = ctypes.cast(user_data, ctypes.POINTER(PXREADevStateJson)).contents
                    raw = bytes(state.stateJson).split(b"\0", 1)[0].decode("utf-8")
                    tracking = parse_tracking(raw)
                    if tracking is None:
                        return
                    event.update(kind="tracking", tracking=tracking,
                                 device_id=bytes(state.devID).split(b"\0", 1)[0].decode("utf-8"))
                else:
                    return
            except Exception as exc:
                event.update(kind="callback_error", detail=str(exc))
            try:
                self.events.put_nowait(event)
            except queue.Full:
                self.overflow.set()

        self.callback = callback
        result = self.sdk.PXREAInit(None, self.callback, PXREA_FULL_MASK)
        if result != 0:
            raise RuntimeError(f"PICO SDK initialization failed: {result}")
        return self

    def next(self):
        if self.overflow.is_set():
            self.overflow.clear()
            while True:
                try:
                    self.events.get_nowait()
                except queue.Empty:
                    break
            event = {"kind": "overflow", "received_monotonic": time.monotonic()}
        else:
            try:
                event = self.events.get(timeout=0.02)
            except queue.Empty:
                event = {"kind": "tick", "received_monotonic": time.monotonic()}
        event["processed_monotonic"] = time.monotonic()
        return event

    def __exit__(self, *_args):
        self.sdk.PXREADeinit()


class NeroFeedback:
    """Passive SDK connection, blocking every transport send including startup."""

    def __init__(self, arm_config, stale_s=0.25):
        self.config = arm_config
        self.stale_s = stale_s
        self.blocked_sends = []
        self.arm = None

    def __enter__(self):
        use_agx_sdk()
        from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config
        from pyAgxArm.protocols.can_protocol.comms.can_comm import CanCommImpl
        self.comm = CanCommImpl
        self.original_send = CanCommImpl.send

        def blocked_send(_comm, message, timeout=None):
            self.blocked_sends.append(hex(message.arbitration_id))
            raise RuntimeError("READ_ONLY_SEND_BLOCKED")

        CanCommImpl.send = blocked_send
        try:
            if self.config["firmware"] != "NeroFW.V120":
                raise ValueError("only the selected Nero V120 firmware profile is supported")
            config = create_agx_arm_config(
                robot=ArmModel.NERO, firmeware_version=NeroFW.V120,
                interface="socketcan", channel=self.config["can_channel"],
                bitrate=self.config["bitrate"], auto_connect=False)
            self.arm = AgxArmFactory.create_arm(config)
            self.arm.connect(start_read_thread=True)
            return self
        except Exception:
            self.__exit__()
            raise

    def snapshot(self):
        if self.blocked_sends:
            raise RuntimeError(f"SDK attempted forbidden CAN sends: {self.blocked_sends}")
        return {**joint_snapshot(self.arm, self.stale_s),
                "read_only": True, "blocked_send_count": len(self.blocked_sends)}

    def robot_snapshot(self):
        if self.blocked_sends:
            raise RuntimeError(f"SDK attempted forbidden CAN sends: {self.blocked_sends}")
        return {**robot_snapshot(self.arm, self.stale_s),
                "read_only": True, "blocked_send_count": 0}

    def __exit__(self, *_args):
        try:
            if self.arm is not None:
                self.arm.disconnect()
        finally:
            self.comm.send = self.original_send
