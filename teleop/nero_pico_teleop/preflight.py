"""Dependency and live reception checks. Never sends a CAN command."""

import argparse
import ctypes
from datetime import datetime, timezone
import importlib
import json
from pathlib import Path
import subprocess
import time

from .paths import DEFAULT_CONFIG, PROJECT_ROOT, pico_sdk_path, use_agx_sdk
from .humanoid_frames import requires_head


def can_interface(channel, bitrate, expected_serial=None):
    result = subprocess.run(["ip", "-j", "-details", "-statistics", "link", "show", channel],
                            capture_output=True, text=True, check=True, timeout=3)
    interface = json.loads(result.stdout)[0]
    info = interface.get("linkinfo", {})
    data = info.get("info_data", {})
    if (info.get("info_kind") != "can" or "UP" not in interface.get("flags", [])
            or data.get("bittiming", {}).get("bitrate") != bitrate
            or data.get("state") != "ERROR-ACTIVE"):
        raise RuntimeError(f"{channel} must be UP at {bitrate} bit/s in ERROR-ACTIVE state")
    device = Path(f"/sys/class/net/{channel}/device").resolve()
    serial = None
    for parent in (device, *device.parents):
        if (parent / "serial").is_file():
            serial = (parent / "serial").read_text().strip()
            break
    if expected_serial and serial != expected_serial:
        raise RuntimeError(f"CAN adapter differs: expected {expected_serial}, found {serial}")
    return {"interface": interface, "adapter_serial": serial}


def site_pending(config):
    arm = config["arm"]
    pending = []
    if arm.get("connection_verification", {}).get("physical_zero_alignment_verified") is not True:
        pending.append("physical_joint_zero_not_verified")
    if arm.get("controller_mounting_readback_verified") is not True:
        pending.append("controller_mounting_not_verified")
    if config.get("mapping_status") != "physically_verified_candidate":
        pending.append("tracking_to_base_mapping_not_physically_verified")
    return pending


def run_checks(config_path, *, live=False, duration=5., arm_id=None):
    report = {"checked_at_utc": datetime.now(timezone.utc).isoformat(),
              "config": str(config_path.resolve()), "live": live,
              "checks": {}, "errors": [], "can_commands_sent": 0,
              "real_motion_allowed": False}
    checks = report["checks"]
    try:
        checks["dependencies"] = {name: getattr(importlib.import_module(name), "__version__", "loaded")
                                  for name in ("numpy", "scipy", "pinocchio", "casadi", "can")}
        checks["nero_sdk"] = str(use_agx_sdk())
        from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config
        driver = AgxArmFactory.load_class(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V120))
        checks["driver"] = f"{driver.__module__}.{driver.__name__}"
        from .nero_relative_core import load_config, NeroKinematics
        import numpy as np
        config = load_config(config_path, arm_id=arm_id)
        model = NeroKinematics(config)
        pose = model.pose(np.deg2rad([0, 45, -60, 60, 0, 20, 0]))
        checks["model"] = {"joints": model.model.nq, "urdf": config["urdf"],
                           "synthetic_tcp_m": pose.translation.tolist()}
        if config.get("runtime_profile") == "moonbot_xnero":
            from .nero_placo_ik import ProcessPlacoIK
            from .nero_stream import stream_configs
            target_config, _ = stream_configs(config, session_limits=False)
            q = np.deg2rad([0, 45, -60, 60, 0, 20, 0])
            with ProcessPlacoIK(target_config) as solver:
                result = solver.solve(pose.translation, pose.rotation, q, q, 1 / 75)
                if not result["ok"]:
                    raise RuntimeError("Placo preflight solve failed")
                checks["placo"] = {"solver": result["solver"], "solve_ms": result["solve_ms"],
                                   "fk_crosscheck_passed": True, "hardware_access": False}
        native = ctypes.CDLL(str(pico_sdk_path()))
        for symbol in ("PXREAInit", "PXREADeinit"):
            getattr(native, symbol)
        checks["pico_sdk"] = str(pico_sdk_path())
        report["site_pending"] = site_pending(config)
        if live:
            arm = config["arm"]
            checks["can"] = can_interface(arm["can_channel"], arm["bitrate"],
                arm.get("connection_verification", {}).get("usb_adapter_serial"))
            from .nero_preview_io import NeroFeedback, PicoEvents
            from .pico_input_guard import InputGuard
            guard = InputGuard(stale_s=config["max_gap_s"],
                               require_head_pose=requires_head(config),
                               hand=config["arm"].get("controller_hand", "right"))
            frames, snapshots, last_feedback = 0, 0, None
            feedback_error = "no feedback"
            with NeroFeedback(arm, config["max_gap_s"]) as feedback, PicoEvents(pico_sdk_path()) as pico:
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline:
                    event = pico.next()
                    now = time.monotonic()
                    if event["kind"] == "tracking":
                        guard.step(event["tracking"], event["received_monotonic"], now, event["device_id"])
                        frames += 1
                    elif event["kind"] == "tick":
                        guard.tick(now)
                    else:
                        guard.invalidate(event["kind"], disconnected=event["kind"] == "disconnect")
                    try:
                        last_feedback = feedback.robot_snapshot()
                        model.command_from_feedback(last_feedback["joints_rad"])
                        snapshots += 1
                        feedback_error = None
                    except ValueError as exc:
                        feedback_error = str(exc)
                checks["blocked_can_sends"] = len(feedback.blocked_sends)
            checks["pico"] = {"frames": frames, "state": guard.state, "reason": guard.reason,
                              "device_id": guard.device_id}
            checks["nero_feedback"] = {"snapshots": snapshots, "last": last_feedback,
                                       "error": feedback_error}
            if feedback_error is not None:
                report["errors"].append(feedback_error)
            if not frames or guard.state != "READY_IDLE":
                report["errors"].append("PICO must send fresh controller input with grip/trigger released")
                if guard.reason == "head_pose_invalid":
                    report["errors"].append("Head-relative TCP mapping requires a valid tracked Head pose (status=3)")
            if last_feedback:
                if last_feedback["status"]["err_code"] or any(last_feedback["joint_faults"]):
                    report["errors"].append("Nero reports an arm or driver fault")
                checks["enabled_joints"] = last_feedback["joints_enabled"]
    except Exception as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
    report["technical_checks_passed"] = not report["errors"]
    report["commissioning_required"] = bool(report.get("site_pending", ["not_checked"]))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--duration", type=float, default=5.)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 0 < args.duration <= 60:
        parser.error("duration must be between 0 and 60 seconds")
    report = run_checks(args.config, live=args.live, duration=args.duration)
    output = args.output or PROJECT_ROOT / "logs" / datetime.now(timezone.utc).strftime("preflight_%Y%m%dT%H%M%S_%fZ.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    print(json.dumps({"report": str(output), **report}, indent=2))
    return 0 if report["technical_checks_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
