"""Start the configured robot workflow after a fresh reception check."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

from .paths import PROJECT_ROOT


def require_site_ready(config):
    arm = config["arm"]
    if arm.get("connection_verification", {}).get("physical_zero_alignment_verified") is not True:
        raise ValueError("Physical joint zero remains unverified in config/teleop_config.json. "
                         "Complete the manufacturer's zero verification before motion; setup does not set zeros.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=PROJECT_ROOT / "config/deployment.json")
    parser.add_argument("--confirm-clearance", action="store_true")
    parser.add_argument("--enable-joints", action="store_true")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--telemetry-socket", type=Path, help="optional local data collector socket")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--single", action="store_true", help="right arm only (can0); overrides profile arm selection")
    modes.add_argument("--dual", action="store_true", help="both right_arm/can0 and left_arm/can1")
    args = parser.parse_args()
    if not args.confirm_clearance:
        parser.error("--confirm-clearance is required for physical startup")
    profile = json.loads(args.profile.read_text())
    config_path = (args.profile.resolve().parent / profile["control_config"]).resolve()
    from .nero_relative_core import load_config
    enabled_arms = profile.get("enabled_arms", ["right_arm"])
    if args.single:
        enabled_arms = ["right_arm"]
    elif args.dual:
        enabled_arms = ["right_arm", "left_arm"]
    if enabled_arms == ["left_arm", "right_arm"]:
        enabled_arms = ["right_arm", "left_arm"]
    if enabled_arms not in (["right_arm"], ["right_arm", "left_arm"]):
        raise ValueError("deployment must select right_arm only or both right_arm and left_arm")
    mode = "dual" if len(enabled_arms) == 2 else "single"
    configs = {arm_id: load_config(config_path, arm_id=arm_id) for arm_id in enabled_arms}
    for config in configs.values():
        require_site_ready(config)
    print(json.dumps({"event": "startup_mode", "mode": mode, "arms": enabled_arms,
                      "can_channels": [config["arm"]["can_channel"] for config in configs.values()]}), flush=True)
    from .nero_stream import stream_configs
    stream_configs(config, session_limits=profile.get("session_limits", True),
                   joint_speed_deg_s=profile.get("joint_speed_deg_s", 20.),
                   **{name: profile[name] for name in ("scale", "radius_mm", "tcp_speed_mm_s",
                       "angular_speed_deg_s", "speed_percent", "translation_only", "gripper")})
    from .preflight import run_checks
    reports = {arm_id: run_checks(config_path, live=True, duration=5., arm_id=arm_id)
               for arm_id in enabled_arms}
    report = reports[enabled_arms[0]]
    if all(item["technical_checks_passed"] for item in reports.values()):
        from .firmware import query_firmware
        for arm_id, item in reports.items():
            firmware = query_firmware(configs[arm_id]["arm"]["can_channel"])
            item["firmware_query"] = firmware
            item["can_commands_sent"] = len(firmware["sent_frames"])
            if firmware.get("error") or firmware["blocked_frames"]:
                item["errors"].append(f"Firmware check failed: {firmware}")
                item["technical_checks_passed"] = False
    if len(reports) > 1:
        report = {"arms": reports, "technical_checks_passed": all(item["technical_checks_passed"] for item in reports.values()),
                  "errors": [f"{arm_id}: {error}" for arm_id, item in reports.items() for error in item["errors"]],
                  "real_motion_allowed": False, "can_commands_sent": sum(item.get("can_commands_sent", 0) for item in reports.values())}
    report.update(teleop_mode=mode, enabled_arms=enabled_arms)
    log = PROJECT_ROOT / "logs" / datetime.now(timezone.utc).strftime("startup_%Y%m%dT%H%M%S_%fZ.json")
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("x") as stream:
        json.dump(report, stream, indent=2)
    if not report["technical_checks_passed"]:
        raise ValueError(f"Preflight failed: {report['errors']}; report: {log}")
    argv = ["nero-pico-teleop", "teleop", "--config", str(config_path), "--confirm-clearance",
            "--duration", str(args.duration if args.duration is not None else profile["duration_s"])]
    for name in ("scale", "radius_mm", "tcp_speed_mm_s", "joint_speed_deg_s", "angular_speed_deg_s", "speed_percent"):
        if profile.get(name) is not None:
            argv.extend(["--" + name.replace("_", "-"), str(profile[name])])
    if not profile.get("session_limits", True):
        argv.append("--no-session-limits")
    for name in ("translation_only", "gripper"):
        if profile[name]:
            argv.append("--" + name.replace("_", "-"))
    if args.enable_joints:
        argv.append("--enable-joints")
    argv.append("--dual" if mode == "dual" else "--single")
    if args.telemetry_socket is not None:
        argv.extend(["--telemetry-socket", str(args.telemetry_socket)])
    from .cli import main as run
    sys.argv = argv
    return run()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, FileNotFoundError) as exc:
        print(f"Startup blocked: {exc}", file=sys.stderr)
        raise SystemExit(2)
