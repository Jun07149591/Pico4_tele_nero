"""Bring up only the configured USB adapter. An active bus is never reset."""

import argparse
import json
from pathlib import Path
import subprocess

from .paths import DEFAULT_CONFIG
from .preflight import can_interface


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--single', action='store_true')
    modes.add_argument('--dual', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    config = json.loads(DEFAULT_CONFIG.read_text())
    robots = json.loads((DEFAULT_CONFIG.parent / config["robot_config"]).read_text())
    planned = []
    for name in (['right_arm', 'left_arm'] if args.dual else ['right_arm']):
        arm = robots['arms'][name]
        channel, bitrate = arm['can_channel'], arm['bitrate']
        expected = arm.get('connection_verification', {}).get('usb_adapter_serial')
        device = Path(f'/sys/class/net/{channel}/device').resolve()
        serial = next(((p / 'serial').read_text().strip() for p in (device, *device.parents)
                       if (p / 'serial').is_file()), None)
        if expected and serial != expected:
            raise RuntimeError(f'Expected adapter {expected} on {channel}; found {serial}')
        info = json.loads(subprocess.check_output(['ip', '-j', 'link', 'show', channel], text=True))[0]
        command = ['sudo', 'ip', 'link', 'set', channel, 'up', 'type', 'can', 'bitrate', str(bitrate)]
        if 'UP' in info['flags']:
            can_interface(channel, bitrate, expected)
            command = None
        planned.append((channel, bitrate, expected, command))
    for channel, bitrate, expected, command in planned:
        if args.dry_run:
            print(json.dumps({'channel': channel, 'command': command, 'dry_run': True}))
            continue
        if command:
            subprocess.run(command, check=True)
        print(json.dumps(can_interface(channel, bitrate, expected), indent=2))


if __name__ == "__main__":
    main()
