"""Read firmware using exactly one 0x4AF query; every other CAN write is blocked."""

import argparse
from datetime import datetime, timezone
import json
import time

from .paths import PROJECT_ROOT, use_agx_sdk


def query_firmware(channel):
    use_agx_sdk()
    from pyAgxArm import AgxArmFactory, create_agx_arm_config, resolve_firmware_profile
    from pyAgxArm.protocols.can_protocol.comms.can_comm import CanCommImpl
    report = {"channel": channel, "firmware": None, "resolved_profile": None,
              "sent_frames": [], "blocked_frames": [], "real_motion_allowed": False}
    original = CanCommImpl.send
    active, arm = False, None

    def query_only(comm, frame, timeout=None):
        detail = {"id": hex(frame.arbitration_id), "data": bytes(frame.data).hex()}
        allowed = (active and not report["sent_frames"] and frame.arbitration_id == 0x4AF
                   and frame.dlc == 1 and bytes(frame.data) == b"\x01"
                   and not any((frame.is_extended_id, frame.is_remote_frame, frame.is_error_frame,
                                frame.is_fd, frame.bitrate_switch, frame.error_state_indicator)))
        if not allowed:
            report["blocked_frames"].append(detail)
            raise RuntimeError("Unexpected CAN write during firmware query")
        report["sent_frames"].append(detail)
        original(comm, frame, timeout=timeout)
        if comm.last_error is not None:
            raise RuntimeError(str(comm.last_error))

    CanCommImpl.send = query_only
    try:
        arm = AgxArmFactory.create_arm(create_agx_arm_config(
            robot="nero", firmeware_version="v120", channel=channel, auto_connect=False))
        arm.connect()
        time.sleep(.1)
        active = True
        firmware = arm.get_firmware(timeout=2.)
        active = False
        if firmware is None:
            raise RuntimeError("No firmware response")
        report["firmware"] = firmware
        report["resolved_profile"] = resolve_firmware_profile("nero", firmware["software_version"])
        if report["resolved_profile"] != "v120":
            raise RuntimeError(f"This deployment requires firmware 1.20; received {firmware}")
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        active = False
        try:
            if arm is not None:
                arm.disconnect()
        finally:
            CanCommImpl.send = original
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", default="can0")
    args = parser.parse_args()
    report = query_firmware(args.channel)
    output = PROJECT_ROOT / "logs" / datetime.now(timezone.utc).strftime("firmware_%Y%m%dT%H%M%S_%fZ.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps({"report": str(output), **report}, indent=2))
    return 2 if report.get("error") or report["blocked_frames"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
