"""Bind camera roles to serial numbers; never opens a recording pipeline."""

import argparse
import copy
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def assign_cameras(config, devices, assignments, automatic=False):
    result = copy.deepcopy(config)
    assignments = dict(assignments)
    if automatic:
        for role, model in (('front', 'D455'), ('right_wrist', 'D405')):
            if role not in assignments:
                candidates = [device['serial'] for device in devices if device['name'].endswith(model)]
                if len(candidates) != 1:
                    raise ValueError(f'{role}: found {len(candidates)} {model} cameras; specify serial explicitly')
                assignments[role] = candidates[0]
    serials = {device['serial'] for device in devices}
    for role, serial in assignments.items():
        if serial not in serials:
            raise ValueError(f'{role}: serial {serial} is not connected')
        result['cameras'][role] = {'backend': 'realsense', 'serial': serial, 'fps': 30}
    selected = [camera['serial'] for camera in result['cameras'].values()]
    if len(set(selected)) != len(selected):
        raise ValueError('one camera cannot fill multiple roles')
    if any(serial.startswith('SET_') for serial in selected):
        raise ValueError('configure both front and right_wrist camera serials')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--auto', action='store_true', help='one D455 as front and one D405 as right wrist')
    for role in ('front', 'right-wrist', 'left-wrist'):
        parser.add_argument('--' + role, metavar='SERIAL')
    parser.add_argument('--output', type=Path, default=ROOT / 'data_collection/config/capture.local.json')
    args = parser.parse_args()
    assignments = {role: getattr(args, role) for role in ('front', 'right_wrist', 'left_wrist') if getattr(args, role)}
    if not args.auto and not assignments:
        parser.error('choose --auto or specify camera roles with serial numbers')
    from nero_pico_data.cameras import discover
    from nero_pico_data.schema import load_config
    source = args.output if args.output.is_file() else ROOT / 'data_collection/config/capture.json'
    try:
        result = assign_cameras(json.loads(source.read_text()), discover(), assignments, args.auto)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(result, indent=2) + '\n')
        load_config(temporary)
        temporary.replace(args.output)
    except (ValueError, RuntimeError) as exc:
        parser.error(str(exc))
    print(json.dumps({'config': str(args.output.resolve()), 'cameras': result['cameras']}, indent=2))


if __name__ == '__main__':
    main()
