#!/usr/bin/env python3
"""PICO/Nero previews, bounded commissioning and continuous relative pose teleop."""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import ExitStack
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import time

import numpy as np

from .nero_relative_core import NeroIK, RelativeTeleop, load_config
from .paths import PROJECT_ROOT, DEFAULT_CONFIG, pico_sdk_path




def historical_seed(config):
    report = json.loads(Path(config["replay_seed_report"]).read_text())
    return np.deg2rad(report["last_pair"]["joint_angles_deg"]), {
        "kind": "historical_snapshot_for_virtual_arm_only",
        "path": config["replay_seed_report"], "current_robot_feedback": False}


def replay_seed(config, path):
    with path.open() as stream:
        first_line = stream.readline()
    if not first_line:
        raise ValueError("recording is empty")
    first = json.loads(first_line)
    if first.get("kind") != "session_start":
        return historical_seed(config)
    source_seed = first.get("seed", {})
    values = first.get("seed_joints_rad", source_seed.get("joints_rad"))
    if values is None:
        raise ValueError("recording has no joint seed; cannot reproduce its virtual arm")
    return np.asarray(values, dtype=float), {
        "kind": "recorded_session_joint_snapshot_for_virtual_arm_only",
        "path": str(path), "current_robot_feedback": False,
        "original_seed_kind": source_seed.get("kind")}


def demo_events(config):
    """Synthetic held/released input; neither a physical calibration nor a capture."""
    index = 0
    mapping = np.asarray(config["controller_reference_to_base"])
    pose_basis = np.asarray(config["pose_basis_change"])
    inverse_mapping = (np.asarray(config["tracking_to_base"]).T
                       if config.get("mapping_mode") == "fixed_tracking_to_base" else pose_basis.T @ mapping.T)
    if config.get("mapping_mode") == "head_relative_tcp":
        # With an identity headset orientation, demo displacements are TCP-local.
        inverse_mapping = np.asarray(config["head_to_tcp_axes"]).T
    elif config.get("mapping_mode") == "head_relative_body":
        from .humanoid_frames import HEADSET_TO_BODY
        inverse_mapping = HEADSET_TO_BODY.T
    base = np.array([0.1, 0.2, -0.3])

    def frame(displacement, grip=0.):
        nonlocal index
        index += 1
        t = index * 0.04
        p = base + inverse_mapping @ displacement / config["translation_scale"]
        tracking = {"timeStampNs": 1_000_000_000 + index * 40_000_000,
                    "Input": 1,
                    "appState": {"focus": True},
                    "Head": {"pose": [0., 1.6, 0., 0., 0., 0., 1.], "status": 3},
                    "Controller": {"right": {
                        "pose": [*p.tolist(), 0., 0., 0., 1.], "grip": grip,
                        "trigger": 0., "primaryButton": False,
                        "secondaryButton": False, "menuButton": False}}}
        return {"kind": "tracking", "tracking": tracking, "device_id": "synthetic_demo",
                "received_monotonic": t, "processed_monotonic": t + 0.001}

    for _ in range(10):
        yield frame(np.zeros(3))
    yield frame(np.zeros(3), 1.)
    for axis in range(3):
        for sign in (1., -1.):
            for fraction in np.r_[np.linspace(0, 1, 26)[1:], np.linspace(1, 0, 26)[1:]]:
                displacement = np.zeros(3)
                displacement[axis] = sign * 0.002 * fraction
                yield frame(displacement, 1.)
    for _ in range(10):
        yield frame(np.zeros(3))


def recorded_events(path):
    with path.open() as stream:
        for line_number, line in enumerate(stream, 1):
            event = json.loads(line)
            if event.get("kind") == "session_start" or event.get("event") == "complete":
                continue
            if "input" in event and "teleop" in event:
                event = event["input"]
            if "kind" not in event or "processed_monotonic" not in event:
                raise ValueError(f"{path}:{line_number}: requires an InputGuard or Nero preview recording")
            # Recompute guard decisions from raw input; recorded guards are not authority.
            event.pop("guard", None)
            event.pop("preview", None)
            yield event


def inspect_robot(config, output, duration, stopping):
    """Inspect the real arm passively, independently of PICO availability."""
    from .nero_preview_io import NeroFeedback
    samples, last, first, error = 0, None, None, None
    max_change = 0.
    from .nero_relative_core import NeroKinematics
    ik = NeroKinematics(config)
    with output.open("x", encoding="utf-8") as stream, NeroFeedback(
            config["arm"], config["max_gap_s"]) as feedback:
        print(json.dumps({"event": "inspecting", "output": str(output),
                          "read_only": True, "real_motion_allowed": False}), flush=True)
        deadline = time.monotonic() + duration
        while not stopping() and time.monotonic() < deadline:
            try:
                last = feedback.robot_snapshot()
                q = ik.validate_joints(last["joints_rad"])
                if first is None:
                    first = q.copy()
                max_change = max(max_change, float(np.max(np.abs(np.rad2deg(q - first)))))
                last.update(joints_deg=np.rad2deg(q).tolist(), tcp_base_m=ik.pose(q).translation.tolist())
                samples += 1
                error = None
                record = {"feedback": last}
            except ValueError as exc:
                error = str(exc)
                record = {"feedback_error": error}
            stream.write(json.dumps({"received_monotonic": time.monotonic(), **record}) + "\n")
            time.sleep(.02)
        result = {"event": "complete", "mode": "inspect", "output": str(output),
                  "arm_id": config["arm_id"], "can_channel": config["arm"]["can_channel"],
                  "fresh_complete_samples": samples, "last_feedback": last,
                  "feedback_error": error, "max_joint_change_deg": max_change,
                  "last_feedback_is_current": last is not None and error is None,
                  "mapping_status": config["mapping_status"],
                  "read_only": True, "real_motion_allowed": False,
                  "can_commands_sent": 0, "blocked_send_count": len(feedback.blocked_sends)}
        stream.write(json.dumps(result) + "\n")
        print(json.dumps(result, indent=2), flush=True)
    return 0 if samples and error is None else 4


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("demo", "replay", "preview", "inspect", "pilot", "teleop"))
    parser.add_argument("--input", type=Path, help="InputGuard or Nero preview JSONL recording for replay")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--duration", type=float, default=30.)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--telemetry-socket", type=Path, help="teleop: publish local data collection telemetry")
    parser.add_argument("--confirm-clearance", action="store_true",
                        help="physical modes: confirm the arm and empty tool are clear of obstacles and supervised")
    parser.add_argument("--enable-joints", action="store_true",
                        help="physical modes: explicitly enable individual arm joints during CAN takeover")
    parser.add_argument("--pilot-profile", choices=("initial", "axis-check", "yz-check", "translation-check", "teleop"),
                        help="pilot only: initial/axis-check (+X), yz-check (+Y/+Z), translation-check (XYZ, 2 mm), or teleop (XYZ, 10 mm, 5 mm/s)")
    parser.add_argument("--continuous-grip", action="store_true",
                        help="pilot: allow regrip after release and settled feedback; teleop always supports this")
    parser.add_argument("--mapping-candidate", type=Path,
                        help="pilot: load a read-only direction candidate into the transform chain; does not authorize CAN output")
    parser.add_argument("--demo-seed", choices=("model", "recorded"), default="model",
                        help="demo model seed is virtual only; recorded uses the old arm snapshot")
    parser.add_argument("--translation-only", action="store_true", help="teleop: keep the initial tool orientation")
    parser.add_argument("--gripper", action="store_true", help="teleop: joystick gripper control of an already enabled AGX gripper")
    parser.add_argument("--scale", type=float, help="teleop robot/controller translation ratio, 0.05-2 (default 1; 2 doubles hand displacement)")
    parser.add_argument("--radius-mm", type=float, help="teleop radius from session start, 2-150 mm (default 50)")
    parser.add_argument("--no-session-limits", action="store_true",
                        help="teleop: disable TCP radius and joint travel limits relative to session start; retain model joint limits and speed/input checks")
    parser.add_argument("--tcp-speed-mm-s", type=float, help="teleop target speed ceiling, 1-200 mm/s (default 50)")
    parser.add_argument("--joint-speed-deg-s", type=float, help="teleop joint target speed ceiling, 1-60 deg/s (default 20)")
    parser.add_argument("--angular-speed-deg-s", type=float, help="teleop angular target speed, 1-30 deg/s (default 30)")
    parser.add_argument("--speed-percent", type=int, help="teleop Nero position-mode speed setting, 1-100 (default 100)")
    arm_modes = parser.add_mutually_exclusive_group()
    arm_modes.add_argument("--single", action="store_true", help="teleop right_arm only (can0, right controller)")
    arm_modes.add_argument("--dual", action="store_true", help="teleop both configured arms (can0 right, can1 left)")
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0:
        parser.error("duration must be finite and positive")
    if args.mode == "replay" and args.input is None:
        parser.error("replay requires --input")
    if args.mode != "replay" and args.input is not None:
        parser.error("--input is only used with replay")
    if args.mode in ("pilot", "teleop"):
        if not args.confirm_clearance:
            parser.error("physical CAN output requires --confirm-clearance")
        if args.mode == "pilot" and args.duration > 30:
            parser.error("first-motion pilot duration must be at most 30 seconds")
    elif args.confirm_clearance or args.enable_joints or args.pilot_profile is not None or args.continuous_grip or args.dual or args.single:
        parser.error("physical control options require pilot or teleop")
    if args.mode != "pilot" and args.pilot_profile is not None:
        parser.error("--pilot-profile is only used with pilot; teleop is a separate continuous mode")
    if args.mapping_candidate is not None and args.mode != "pilot":
        parser.error("--mapping-candidate is currently only supported with pilot")
    stream_options = {name: getattr(args, name) for name in
                      ("scale", "radius_mm", "tcp_speed_mm_s", "joint_speed_deg_s", "angular_speed_deg_s", "speed_percent")
                      if getattr(args, name) is not None}
    if args.mode != "teleop" and (stream_options or args.translation_only or args.gripper or args.no_session_limits):
        parser.error("stream settings, --translation-only, --gripper and --no-session-limits require teleop mode")
    if (args.dual or args.single) and args.mode != "teleop":
        parser.error("--single and --dual require teleop mode")
    if args.telemetry_socket is not None and args.mode != "teleop":
        parser.error("--telemetry-socket requires teleop mode")
    config = load_config(args.config, arm_id="right_arm" if args.single else None)
    if args.mode in ("pilot", "teleop"):
        from .deploy import require_site_ready
        try:
            require_site_ready(config)
        except ValueError as exc:
            parser.error(str(exc))
    if args.mode == "teleop":
        from .nero_stream import stream_configs
        stream_options.update(translation_only=args.translation_only, gripper=args.gripper,
                              session_limits=not args.no_session_limits)
        try:
            stream_configs(config, **stream_options)
        except ValueError as exc:
            parser.error(str(exc))
    output = args.output or PROJECT_ROOT / "logs" / datetime.now(timezone.utc).strftime(
        f"nero_{args.mode}_%Y%m%dT%H%M%S_%fZ.jsonl")
    output.parent.mkdir(parents=True, exist_ok=True)
    stop = False

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    if args.mode == "teleop":
        from .nero_stream import run_physical_stream
        if args.telemetry_socket is not None:
            stream_options["telemetry_socket"] = args.telemetry_socket
        if args.dual:
            from .nero_stream import run_physical_dual_stream
            robot_config = json.loads((Path(args.config).resolve().parent / config["robot_config"]).read_text())
            enabled = [arm_id for arm_id in ("right_arm", "left_arm")
                       if robot_config["arms"].get(arm_id, {}).get("enabled") is True]
            if set(enabled) != {"right_arm", "left_arm"}:
                parser.error("--dual requires enabled right_arm and left_arm configurations")
            dual_configs = {arm_id: load_config(args.config, arm_id=arm_id) for arm_id in enabled}
            try:
                for dual_config in dual_configs.values():
                    require_site_ready(dual_config)
            except ValueError as exc:
                parser.error(str(exc))
            return run_physical_dual_stream(dual_configs, output, args.duration, lambda: stop,
                                            clearance_confirmed=args.confirm_clearance,
                                            enable_joints=args.enable_joints, **stream_options)
        return run_physical_stream(config, output, args.duration, lambda: stop,
                                   clearance_confirmed=args.confirm_clearance, enable_joints=args.enable_joints,
                                   **stream_options)
    if args.mode == "pilot":
        from .nero_pilot import run_physical_pilot
        return run_physical_pilot(config, output, args.duration, lambda: stop,
                                   clearance_confirmed=args.confirm_clearance, enable_joints=args.enable_joints,
                                   profile=args.pilot_profile or "initial", continuous_grip=args.continuous_grip,
                                   mapping_candidate=args.mapping_candidate)
    if args.mode == "inspect":
        return inspect_robot(config, output, args.duration, lambda: stop)
    counts = Counter()
    tracking_frames = 0
    last_right_grip = last_right_trigger = None
    max_right_grip = None
    solve_times, pos_errors, ori_errors = [], [], []
    solver_attempts = 0
    max_displacement = 0.
    max_joint_displacement = 0.
    previous_ik = None
    feedback = None
    with ExitStack() as stack:
        stream = stack.enter_context(output.open("x", encoding="utf-8"))
        if args.mode == "preview":
            from .nero_preview_io import NeroFeedback, PicoEvents
            feedback = stack.enter_context(NeroFeedback(config["arm"], config["max_gap_s"]))
            deadline = time.monotonic() + 3
            while True:
                try:
                    seed_info = feedback.snapshot()
                    seed = np.asarray(seed_info["joints_rad"])
                    seed_info.update(kind="live_can_snapshot_for_virtual_arm", current_robot_feedback=True)
                    break
                except ValueError:
                    if stop or time.monotonic() > deadline:
                        raise RuntimeError("no fresh complete Nero feedback; preview was not started")
                    time.sleep(0.02)
        elif args.mode == "demo" and args.demo_seed == "model":
            seed = np.deg2rad([0, 45, -60, 60, 0, 20, 0])
            seed_info = {"kind": "synthetic_model_pose_not_a_robot_move_target",
                         "current_robot_feedback": False}
        elif args.mode == "replay":
            seed, seed_info = replay_seed(config, args.input)
        else:
            seed, seed_info = historical_seed(config)
        teleop = RelativeTeleop(config, seed)
        if args.mode == "preview":
            pico = stack.enter_context(PicoEvents(pico_sdk_path()))
            events = iter(pico.next, None)
        else:
            events = demo_events(config) if args.mode == "demo" else recorded_events(args.input)
        started = time.monotonic()
        metadata = {"kind": "session_start", "mode": args.mode, "seed": seed_info,
                    "seed_joints_rad": seed.tolist(),
                    "config": config, "output": str(output), "read_only": True,
                    "real_motion_allowed": False, "motion_sender_used": False}
        stream.write(json.dumps(metadata) + "\n")
        print(json.dumps({"event": "initialized", "mode": args.mode, "output": str(output),
                          "input_ready": False, "tracking_frames": 0,
                          "seed_kind": seed_info["kind"], "mapping_status": config["mapping_status"],
                          "real_motion_allowed": False}), flush=True)
        last_print = float("-inf")
        previous_state = None
        last_time = 0.
        try:
            for event in events:
                if stop or (args.mode == "preview" and time.monotonic() - started >= args.duration):
                    break
                last_time = event["processed_monotonic"]
                if event["kind"] == "tracking":
                    tracking_frames += 1
                    tracking = event.get("tracking")
                    controllers = tracking.get("Controller") if isinstance(tracking, dict) else None
                    right = controllers.get("right") if isinstance(controllers, dict) else None
                    last_right_grip = right.get("grip") if isinstance(right, dict) else None
                    last_right_trigger = right.get("trigger") if isinstance(right, dict) else None
                    if type(last_right_grip) in (int, float) and np.isfinite(last_right_grip):
                        max_right_grip = max(last_right_grip, max_right_grip or 0.)
                if feedback is not None:
                    try:
                        actual = feedback.snapshot()
                        event["nero_feedback"] = actual
                        if np.max(np.abs(np.asarray(actual["joints_rad"]) - seed)) > np.deg2rad(1):
                            raise RuntimeError("actual arm moved over 1 degree during read-only preview; restart from fresh feedback")
                    except ValueError as exc:
                        event = {"kind": "robot_feedback_invalid", "detail": str(exc),
                                 "pico_event": event,
                                 "processed_monotonic": time.monotonic()}
                result = teleop.process(event)
                counts[result["state"]] += 1
                counts["reason:" + result["reason"]] += 1
                max_displacement = max(max_displacement, float(np.linalg.norm(teleop.target - teleop.origin)))
                max_joint_displacement = max(max_joint_displacement, float(np.max(np.abs(teleop.q - seed))))
                if result["ik"] is not None and result["ik"] is not previous_ik:
                    previous_ik = result["ik"]
                    solve_times.append(previous_ik.get("solve_ms", 0.))
                    solver_attempts += len(previous_ik.get("attempts", [previous_ik]))
                    if previous_ik["ok"]:
                        pos_errors.append(previous_ik["position_error_m"])
                        ori_errors.append(previous_ik["orientation_error_deg"])
                stream.write(json.dumps({"input": event, "teleop": result}, separators=(",", ":")) + "\n")
                signature = (result["state"], result["reason"])
                if last_time - last_print >= 1 or (signature != previous_state and result["state"] in ("HOLD", "ANCHOR", "FOLLOW", "LIMITED")):
                    console = {k: result[k] for k in ("state", "reason", "tcp_displacement_base_mm", "joint_target_deg", "real_motion_allowed")}
                    console.update(input_state=result["guard"]["state"], tracking_frames=tracking_frames,
                                   last_right_grip=last_right_grip, last_right_trigger=last_right_trigger)
                    if result["reason"] in ("ik_rejected_requires_release", "position_target_limited"):
                        console["ik_rejection"] = result["ik"]
                    print(json.dumps(console), flush=True)
                    last_print = last_time
                    stream.flush()
                if result["state"] != "WAIT_SAMPLE":
                    previous_state = signature
        finally:
            end = {"kind": "session_end", "processed_monotonic": last_time}
            stream.write(json.dumps({"input": end, "teleop": teleop.process(end)}) + "\n")
        summary = {"event": "complete", "mode": args.mode, "output": str(output),
                   "tracking_frames": tracking_frames, "max_right_grip": max_right_grip,
                   "states_and_reasons": dict(counts), "ik_calls": len(solve_times),
                   "ik_solver_attempts": solver_attempts,
                   "ik_solve_p95_ms": float(np.percentile(solve_times, 95)) if solve_times else None,
                   "max_accepted_position_error_mm": max(pos_errors) * 1000 if pos_errors else None,
                   "max_accepted_orientation_error_deg": max(ori_errors) if ori_errors else None,
                   "max_tcp_target_displacement_mm": max_displacement * 1000,
                   "max_joint_displacement_deg": float(np.rad2deg(max_joint_displacement)),
                   "read_only": True, "real_motion_allowed": False,
                   "can_commands_sent": 0,
                   "blocked_send_count": len(feedback.blocked_sends) if feedback else 0}
        stream.write(json.dumps(summary) + "\n")
        print(json.dumps(summary, indent=2), flush=True)
    if feedback and feedback.blocked_sends:
        return 2
    if args.mode == "demo":
        return 0 if len(pos_errors) >= 100 and not counts["reason:ik_rejected_requires_release"] else 3
    return 0 if counts["ANCHOR"] and pos_errors else 4


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, OSError, KeyError) as exc:
        print(f"ERROR: {exc}", flush=True)
        raise SystemExit(2)
