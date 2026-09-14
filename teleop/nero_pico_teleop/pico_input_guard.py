#!/usr/bin/env python3
"""Read-only PICO input guard. No state authorizes robot motion."""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import queue
import signal
import sys
import threading
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .paths import PROJECT_ROOT, pico_sdk_path

from .pico_tracking_sampler import (
    Callback, PXREADevStateJson, PXREA_DEVICE_MISSING, PXREA_DEVICE_STATE_JSON,
    PXREA_SERVER_DISCONNECT, PXREA_FULL_MASK, parse_tracking,
)


def finite_number(value: Any) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def valid_pose(value: Any) -> list[float] | None:
    if isinstance(value, str):
        try:
            value = [float(v.strip()) for v in value.split(",")]
        except ValueError:
            return None
    if not isinstance(value, (list, tuple)) or len(value) != 7:
        return None
    if not all(finite_number(v) for v in value):
        return None
    pose = [float(v) for v in value]
    norm = math.hypot(*pose[3:])
    if not 0.9 <= norm <= 1.1:
        return None
    if all(abs(v) < 1e-7 for v in pose[:6]) and abs(abs(pose[6]) - 1) < 1e-7:
        return None
    return pose


def valid_head_pose(tracking: Any) -> list[float] | None:
    head = tracking.get("Head") if isinstance(tracking, dict) else None
    if not isinstance(head, dict) or type(head.get("status")) is not int or head["status"] != 3:
        return None
    return valid_pose(head.get("pose"))


class RelativePreview:
    """Bounded displacement in PICO axes only; never a robot TCP target."""

    def __init__(self, scale: float = 0.1, max_displacement_m: float = 0.02,
                 max_speed_m_s: float = 0.01, max_gap_s: float = 0.25,
                 mapping: list[list[float]] | None = None,
                 reference_tcp_m: list[float] | None = None,
                 source_basis: list[list[float]] | None = None):
        if not all(finite_number(v) and v > 0 for v in
                   (scale, max_displacement_m, max_speed_m_s, max_gap_s)):
            raise ValueError("preview limits must be finite and positive")
        self.scale = scale
        self.max_displacement = max_displacement_m
        self.max_speed = max_speed_m_s
        self.max_gap = max_gap_s
        self.position = [0.0, 0.0, 0.0]
        self.controller_anchor: list[float] | None = None
        self.preview_anchor = self.position.copy()
        self.last_received: float | None = None
        self.last_processed: float | None = None
        self.generation = 0
        self.mapping = mapping
        self.reference_tcp_m = reference_tcp_m
        self.source_basis = source_basis
        if mapping is not None:
            if (len(mapping) != 3 or any(len(row) != 3 for row in mapping)
                    or reference_tcp_m is None or len(reference_tcp_m) != 3):
                raise ValueError("candidate mapping requires a 3x3 matrix and reference TCP")
            if (source_basis is None or len(source_basis) != 3
                    or any(len(row) != 3 for row in source_basis)):
                raise ValueError("candidate mapping requires a 3x3 source basis")

    def mapped_snapshot(self, result: dict[str, Any]) -> dict[str, Any]:
        if self.mapping is None:
            return result
        import numpy as np
        mapping = np.asarray(self.mapping, dtype=float)
        raw = np.asarray(self.position, dtype=float)
        source_basis = np.asarray(self.source_basis, dtype=float)
        reference = np.asarray(self.reference_tcp_m, dtype=float)
        delta = mapping @ raw
        result.update({
            "coordinate_frame": "right_arm_base_offline_candidate",
            "robot_delta_m": delta.tolist(),
            "robot_target_m": (reference + delta).tolist(),
            "robot_orientation_target": "held_at_reference",
            "motion_target_generated": False,
        })
        return result

    def snapshot(self, state: str, reason: str, **extra: Any) -> dict[str, Any]:
        result = {
            "state": state, "reason": reason,
            "coordinate_frame": "pico_tracking_axes_unmapped_to_robot",
            "displacement_m": self.position.copy(),
            "anchor_generation": self.generation,
            "translation_scale": self.scale,
            "max_displacement_m": self.max_displacement,
            "max_speed_m_s": self.max_speed,
            "robot_target": None, "orientation_target": None, "gripper_target": None,
            "read_only": True, "real_motion_allowed": False,
            **extra,
        }
        return self.mapped_snapshot(result)

    def hold(self, reason: str) -> dict[str, Any]:
        self.controller_anchor = None
        self.last_received = self.last_processed = None
        return self.snapshot("HOLD", reason)

    def step(self, event: dict[str, Any]) -> dict[str, Any]:
        guard = event.get("guard", {})
        if (guard.get("input_held") is not True or guard.get("state") != "INPUT_HELD"
                or guard.get("paused") is not False or guard.get("needs_release") is not False):
            return self.hold("guard_not_held")
        if event.get("kind") != "tracking":
            return self.snapshot("HOLD", "no_new_tracking_sample")
        tracking = event.get("tracking", {})
        controllers = tracking.get("Controller") if isinstance(tracking, dict) else None
        right = controllers.get("right") if isinstance(controllers, dict) else None
        pose = valid_pose(right.get("pose")) if isinstance(right, dict) else None
        received, processed = event.get("received_monotonic"), event.get("processed_monotonic")
        if pose is None or not all(finite_number(t) for t in (received, processed)):
            return self.hold("invalid_preview_sample")
        if not 0 <= processed - received <= self.max_gap:
            return self.hold("invalid_preview_sample_age")
        if self.controller_anchor is None:
            self.controller_anchor = pose[:3]
            self.preview_anchor = self.position.copy()
            self.last_received, self.last_processed = received, processed
            self.generation += 1
            return self.snapshot("ANCHOR", "new_relative_grip_anchor")
        received_dt = received - self.last_received
        processed_dt = processed - self.last_processed
        if not (0 < received_dt <= self.max_gap and 0 < processed_dt <= self.max_gap):
            return self.hold("invalid_preview_interval")
        self.last_received, self.last_processed = received, processed
        requested = [self.preview_anchor[i] + self.scale * (pose[i] - self.controller_anchor[i])
                     for i in range(3)]
        if math.hypot(*requested) > self.max_displacement:
            return self.snapshot("LIMIT_HOLD", "requested_displacement_exceeds_limit",
                                 requested_displacement_m=requested)
        delta = [requested[i] - self.position[i] for i in range(3)]
        distance = math.hypot(*delta)
        # Native SDK bursts must not turn old callback intervals into fast updates.
        dt = min(received_dt, processed_dt)
        fraction = min(1.0, self.max_speed * dt / distance) if distance else 0.0
        self.position = [self.position[i] + fraction * delta[i] for i in range(3)]
        return self.snapshot("FOLLOW", "bounded_relative_displacement",
                             requested_displacement_m=requested,
                             update_speed_m_s=distance * fraction / dt)


class InputGuard:
    """A diagnostic state machine, independent of SDK callbacks and wall time."""

    def __init__(self, stale_s: float = 0.25, neutral_s: float = 0.15, *, require_head_pose: bool = False,
                 hand: str = "right", secondary_action: str = "pause"):
        if not finite_number(stale_s) or stale_s <= 0:
            raise ValueError("stale_s must be finite and positive")
        if not finite_number(neutral_s) or neutral_s < 0:
            raise ValueError("neutral_s must be finite and nonnegative")
        self.stale_s = stale_s
        self.require_head_pose = require_head_pose
        if hand not in ("left", "right"):
            raise ValueError("hand must be left or right")
        self.hand = hand
        if secondary_action not in ("pause", "home"):
            raise ValueError("unsupported secondary button action")
        self.secondary_action = secondary_action
        self.neutral_s = neutral_s
        self.paused = False
        self.needs_release = True
        self.neutral_since: float | None = None
        self.device_id: str | None = None
        self.last_source: int | None = None
        self.last_received: float | None = None
        self.last_valid_received: float | None = None
        self.last_pose: list[float] | None = None
        self.previous_a: bool | None = None
        self.previous_b: bool | None = None
        self.accepted = False
        self.reference_requests = 0
        self.home_requests = 0
        self.state = "DISCONNECTED"
        self.reason = "waiting_for_tracking"

    def snapshot(self, **extra: Any) -> dict[str, Any]:
        result = {
            "state": self.state, "reason": self.reason,
            "paused": self.paused, "needs_release": self.needs_release,
            "input_held": self.accepted, "device_id": self.device_id,
            "reference_requests": self.reference_requests,
            "home_requests": self.home_requests,
            "read_only": True, "real_motion_allowed": False,
        }
        result.update(extra)
        return result

    def invalidate(self, reason: str, disconnected: bool = False) -> dict[str, Any]:
        self.state = "DISCONNECTED" if disconnected else "INVALID"
        self.reason = reason
        self.accepted = False
        self.needs_release = True
        self.neutral_since = None
        self.previous_a = self.previous_b = None
        self.last_pose = None
        self.last_valid_received = None
        if disconnected:
            # A reconnect may restart the source clock, but never clears pause.
            self.last_source = self.last_received = None
        return self.snapshot()

    def tick(self, now: float) -> dict[str, Any]:
        if self.last_valid_received is not None:
            age = now - self.last_valid_received
            if age < 0 or age > self.stale_s:
                return self.invalidate("tracking_stale")
        return self.snapshot()

    def step(self, tracking: Any, received: float, now: float,
             device_id: str) -> dict[str, Any]:
        if not finite_number(received) or not finite_number(now):
            return self.invalidate("invalid_callback_time")
        if not 0 <= now - received <= self.stale_s:
            return self.invalidate("callback_too_old_or_future")
        if not isinstance(device_id, str) or not device_id:
            return self.invalidate("missing_device_id")
        if self.device_id is not None and device_id != self.device_id:
            return self.invalidate("device_identity_changed")
        if not isinstance(tracking, dict):
            return self.invalidate("invalid_tracking")
        app = tracking.get("appState")
        if not isinstance(app, dict) or app.get("focus") is not True:
            return self.invalidate("app_focus_invalid")
        # PXR ActiveInputDevice.ControllerActive = 1; other modes can retain poses.
        input_mode = tracking.get("Input")
        if type(input_mode) is not int or input_mode != 1:
            return self.invalidate("controller_mode_required")
        controllers = tracking.get("Controller")
        controller = controllers.get(self.hand) if isinstance(controllers, dict) else None
        if not isinstance(controller, dict):
            return self.invalidate(f"{self.hand}_controller_missing")
        pose = valid_pose(controller.get("pose"))
        if pose is None:
            return self.invalidate(f"{self.hand}_pose_invalid")
        if self.require_head_pose and valid_head_pose(tracking) is None:
            return self.invalidate("head_pose_invalid")
        grip, trigger = controller.get("grip"), controller.get("trigger")
        if not all(finite_number(v) and 0 <= v <= 1 for v in (grip, trigger)):
            return self.invalidate("invalid_analog_input")
        a, b, menu = (controller.get(k) for k in ("primaryButton", "secondaryButton", "menuButton"))
        if not all(type(v) is bool for v in (a, b, menu)):
            return self.invalidate("invalid_button_input")
        source = tracking.get("timeStampNs")
        if type(source) is not int or source <= 0:
            return self.invalidate("invalid_source_timestamp")
        if self.last_source is not None and source <= self.last_source:
            return self.invalidate("source_timestamp_not_increasing")
        if self.last_received is not None and received <= self.last_received:
            return self.invalidate("callback_timestamp_not_increasing")
        gap = ((self.last_source is not None and (source - self.last_source) / 1e9 > self.stale_s)
               or (self.last_received is not None and received - self.last_received > self.stale_s))
        self.device_id = device_id
        self.last_source, self.last_received = source, received
        if gap:
            return self.invalidate("tracking_gap_requires_release")
        if self.last_pose is not None and math.dist(pose[:3], self.last_pose[:3]) > 0.05:
            return self.invalidate("position_jump_requires_release")
        if menu:
            return self.invalidate("menu_pressed")
        self.last_pose = pose
        self.last_valid_received = received
        b_edge = b and self.previous_b is False
        a_edge = a and self.previous_a is False
        self.previous_a, self.previous_b = a, b
        action = None
        if b_edge:
            if self.secondary_action == "home":
                self.home_requests += 1
                action = "saved_home_requested"
            else:
                self.paused = not self.paused
                action = "pause_on" if self.paused else "pause_off_requires_release"
            self.needs_release = True
            self.neutral_since = None
        if a_edge:
            self.reference_requests += 1
            self.needs_release = True
            self.neutral_since = None
            action = "reference_request_only" if action is None else action
        if self.accepted and grip <= 0.5:
            self.needs_release = True
            self.neutral_since = None
        self.accepted = False
        if self.paused:
            self.neutral_since = None
            self.state, self.reason = "PAUSED", "software_pause"
        else:
            # A fresh neutral interval is required after startup or any interruption.
            neutral = grip <= 0.2 and trigger <= 0.1 and not a and not b
            if self.needs_release:
                if neutral:
                    if self.neutral_since is None:
                        self.neutral_since = received
                    if received - self.neutral_since >= self.neutral_s:
                        self.needs_release = False
                else:
                    self.neutral_since = None
            if self.needs_release:
                self.state, self.reason = "WAIT_RELEASE", "fresh_neutral_required"
            elif grip > 0.5:
                self.accepted = True
                self.state, self.reason = "INPUT_HELD", "diagnostic_grip_held"
            else:
                self.state, self.reason = "READY_IDLE", "grip_released"
        return self.snapshot(grip=grip, trigger=trigger, action=action,
                             source_timestamp_ns=source, callback_age_ms=(now - received) * 1000)


def load_preview_candidate(path: Path) -> tuple[list[list[float]], list[float], list[list[float]]]:
    candidate = json.loads(path.read_text(encoding="utf-8"))
    if (candidate.get("read_only") is not True
            or candidate.get("real_motion_allowed") is not False
            or candidate.get("active_configuration_updated") is not False
            or candidate.get("motion_targets_generated") is not False
            or candidate.get("mapping_status") != "offline_candidate_only"):
        raise ValueError("候选标定不是只读离线候选")
    mapping = candidate.get("mapping_matrix_source_to_right_arm_base")
    reference = candidate.get("model_tcp_position_at_reference_m")
    directions_path = Path(candidate.get("directions_report", ""))
    if not directions_path.is_absolute():
        directions_path = path.parent / directions_path
    directions = json.loads(directions_path.read_text(encoding="utf-8"))
    source_basis = directions.get("orthogonalized_source_basis_columns_up_outward_left")
    if (not isinstance(mapping, list) or len(mapping) != 3
            or any(not isinstance(row, list) or len(row) != 3 for row in mapping)
            or not isinstance(reference, list) or len(reference) != 3):
        raise ValueError("候选标定缺少有效的方向矩阵或 TCP 参考点")
    if (not isinstance(source_basis, list) or len(source_basis) != 3
            or any(not isinstance(row, list) or len(row) != 3 for row in source_basis)):
        raise ValueError("候选标定缺少有效的 PICO 源基底")
    return mapping, reference, source_basis


def chinese_console_message(guard: dict[str, Any], preview: dict[str, Any] | None) -> str:
    state = guard.get("state")
    if state == "WAIT_RELEASE":
        return "等待：请松开握把键、扳机键和 A，保持手柄静止。"
    if state == "READY_IDLE":
        return "已就绪：按住右手柄握把键开始只读预览；松开握把键会保持当前位置。"
    if state == "INPUT_HELD":
        if preview and preview.get("state") == "ANCHOR":
            return "已建立当前位置锚点：继续按住握把键，移动手柄即可预览。"
        if preview and preview.get("state") == "FOLLOW":
            delta = preview.get("robot_delta_m", [0.0, 0.0, 0.0])
            target = preview.get("robot_target_m", [0.0, 0.0, 0.0])
            return ("只读预览：右臂基座 ΔX={:.1f} mm，ΔY={:.1f} mm，ΔZ={:.1f} mm；"
                    "预计 TCP=[{:.1f}, {:.1f}, {:.1f}] mm。机械臂不会运动。"
                    .format(*(1000.0 * value for value in (*delta, *target))))
        if preview and preview.get("state") == "LIMIT_HOLD":
            return "预览限位保持：位移超过 20 mm，请松开握把键后回到小范围。"
        return "已握住握把键：等待有效的右手柄追踪数据。"
    if state == "PAUSED":
        return "软件暂停：按 B 解除后还需要松开并重新按住握把键。"
    if state in ("INVALID", "DISCONNECTED"):
        return "输入无效或设备断开：请保持机械臂不动，检查 PICO 前台和 USB 连接。"
    if state == "READY_IDLE":
        return "已就绪。"
    return "等待右手柄追踪数据。"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", default=str(pico_sdk_path()))
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--stale-ms", type=float, default=250.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--candidate", type=Path,
                        help="只读 PICO4 方向候选报告；用于显示右臂基座预览坐标")
    parser.add_argument("--terminal-guidance", action="store_true",
                        help="使用中文终端提示")
    parser.add_argument("--preview", action="store_true",
                        help="also compute bounded relative displacement in PICO axes; no robot targets")
    args = parser.parse_args()
    if not finite_number(args.duration) or args.duration < 0:
        parser.error("duration must be finite and nonnegative")
    try:
        guard = InputGuard(stale_s=args.stale_ms / 1000)
    except ValueError as exc:
        parser.error(str(exc))
    mapping = reference_tcp = source_basis = None
    if args.candidate:
        try:
            mapping, reference_tcp, source_basis = load_preview_candidate(args.candidate)
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
            parser.error(str(exc))
        args.preview = True
    preview = (RelativePreview(max_gap_s=guard.stale_s, mapping=mapping,
                               reference_tcp_m=reference_tcp, source_basis=source_basis)
               if args.preview else None)
    output = args.output or PROJECT_ROOT / "logs" / datetime.now(timezone.utc).strftime("guard_%Y%m%dT%H%M%S_%fZ.jsonl")
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        sdk = ctypes.CDLL(args.sdk)
    except OSError as exc:
        print(f"ERROR cannot load SDK: {exc}", file=sys.stderr)
        return 2
    sdk.PXREAInit.argtypes = [ctypes.c_void_p, Callback, ctypes.c_uint]
    sdk.PXREAInit.restype = ctypes.c_int
    sdk.PXREADeinit.argtypes = []
    sdk.PXREADeinit.restype = ctypes.c_int
    events: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=256)
    overflow = threading.Event()

    @Callback
    def callback(_context: int, event_type: int, status: int, user_data: int) -> None:
        received = time.monotonic()
        event: dict[str, Any] = {"received_monotonic": received, "sdk_status": status}
        try:
            if event_type in (PXREA_SERVER_DISCONNECT, PXREA_DEVICE_MISSING):
                event.update(kind="disconnect", event_type=event_type)
            elif event_type == PXREA_DEVICE_STATE_JSON and user_data:
                state = ctypes.cast(user_data, ctypes.POINTER(PXREADevStateJson)).contents
                raw = bytes(state.stateJson).split(b"\0", 1)[0].decode("utf-8", errors="replace")
                tracking = parse_tracking(raw)
                if tracking is None:
                    return
                event.update(kind="tracking", tracking=tracking,
                             device_id=bytes(state.devID).split(b"\0", 1)[0].decode("utf-8", errors="replace"))
            else:
                return
        except Exception as exc:
            event.update(kind="invalid", error=str(exc))
        try:
            events.put_nowait(event)
        except queue.Full:
            overflow.set()

    stop = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    counts: Counter[str] = Counter()
    frames = 0
    last_print = 0.0
    previous_signature = None
    with output.open("x", encoding="utf-8") as stream:
        initialized = False
        try:
            result = sdk.PXREAInit(None, callback, PXREA_FULL_MASK)
            if result != 0:
                print(f"ERROR PXREAInit returned {result}", file=sys.stderr)
                return 3
            initialized = True
            started = time.monotonic()
            if args.terminal_guidance:
                print("中文只读预览已启动：机械臂不会运动，也不会发送 CAN 指令。", flush=True)
                print("站位：照片左侧的右臂；站在自由空间一侧，正对右臂安装面，肩膀与安装面平行。", flush=True)
                print("方向：向上=朝天花板；远离立柱=朝身体；向左=面对安装面时你的左侧。", flush=True)
                print("映射：向上→右臂基座 +X，远离立柱→+Z，面对安装面时你的左侧→+Y。", flush=True)
            else:
                print(json.dumps({"event": "capture_start", "output": str(output), "duration_s": args.duration,
                                  "preview_enabled": preview is not None,
                                  "read_only": True, "real_motion_allowed": False}), flush=True)
            while not stop and (args.duration == 0 or time.monotonic() - started < args.duration):
                if overflow.is_set():
                    overflow.clear()
                    while True:
                        try:
                            events.get_nowait()
                        except queue.Empty:
                            break
                    event = {"kind": "overflow", "received_monotonic": time.monotonic()}
                else:
                    try:
                        event = events.get(timeout=0.02)
                    except queue.Empty:
                        event = {"kind": "tick", "received_monotonic": time.monotonic()}
                now = time.monotonic()
                kind = event["kind"]
                if kind == "tracking":
                    frames += 1
                    result = guard.step(event["tracking"], event["received_monotonic"], now, event["device_id"])
                elif kind == "disconnect":
                    result = guard.invalidate("device_or_server_disconnect", disconnected=True)
                elif kind in ("overflow", "invalid"):
                    result = guard.invalidate("event_queue_overflow" if kind == "overflow" else "callback_parse_error")
                else:
                    result = guard.tick(now)
                counts[result["state"]] += 1
                event.update(processed_monotonic=now, guard=result)
                if preview is not None:
                    event["preview"] = preview.step(event)
                stream.write(json.dumps(event, ensure_ascii=False, separators=(",", ":")) + "\n")
                signature = (result["state"], result["reason"], result["paused"])
                if signature != previous_signature or result.get("action") or now - last_print >= 1:
                    console_result = dict(result)
                    if preview is not None:
                        console_result["preview"] = event["preview"]
                    if args.terminal_guidance:
                        print(chinese_console_message(result, event.get("preview")), flush=True)
                    else:
                        print(json.dumps(console_result, ensure_ascii=False), flush=True)
                    stream.flush()
                    last_print, previous_signature = now, signature
        finally:
            if initialized:
                sdk.PXREADeinit()
            end = {"kind": "capture_end", "processed_monotonic": time.monotonic(),
                   "guard": guard.invalidate("capture_stopped")}
            if preview is not None:
                end["preview"] = preview.step(end)
            stream.write(json.dumps(end, separators=(",", ":")) + "\n")
    if args.terminal_guidance:
        print(f"中文只读预览结束：收到 {frames} 帧 Tracking；机械臂未运动。输出文件：{output}", flush=True)
    else:
        print(json.dumps({"event": "capture_complete", "output": str(output), "tracking_frames": frames,
                          "state_event_counts": dict(counts), "real_motion_allowed": False}), flush=True)
    return 0 if frames else 4


if __name__ == "__main__":
    raise SystemExit(main())
