"""Fetch pinned XRoboToolkit assets without installing system services."""

import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ubuntu', choices=('22.04', '24.04'))
    parser.add_argument('--download-dir', type=Path, default=ROOT / '.downloads')
    args = parser.parse_args()
    version = args.ubuntu or platform.freedesktop_os_release().get('VERSION_ID')
    if version not in ('22.04', '24.04') or platform.machine() != 'x86_64':
        parser.error('supported XR binary platforms: Ubuntu 22.04/24.04 x86_64')
    manifest = json.loads((ROOT / 'vendor/xr-releases.json').read_text())
    cache = args.download_dir.resolve()
    cache.mkdir(parents=True, exist_ok=True)
    assets = {}
    for key in (f'pc_{version}', 'apk'):
        spec = manifest[key]
        target = cache / Path(urlparse(spec['url']).path).name
        if not target.exists():
            partial = target.with_suffix(target.suffix + '.partial')
            subprocess.run(['curl', '--fail', '--location', '--retry', '3', '--connect-timeout', '20',
                            '--output', str(partial), spec['url']], check=True)
            if sha256(partial) != spec['sha256']:
                raise RuntimeError(f'Checksum mismatch: {partial}')
            partial.replace(target)
        if sha256(target) != spec['sha256']:
            raise RuntimeError(f'Checksum mismatch: {target}; replace this download before retrying')
        assets[key] = target
    runtime = ROOT / '.runtime'
    runtime.mkdir(exist_ok=True)
    service = runtime / 'roboticsservice'
    if not service.exists():
        with tempfile.TemporaryDirectory(dir=runtime) as temporary:
            unpacked = Path(temporary)
            subprocess.run(['dpkg-deb', '--extract', str(assets[f'pc_{version}']), str(unpacked)], check=True)
            sources = list(unpacked.rglob('RoboticsServiceProcess'))
            if len(sources) != 1:
                raise RuntimeError('Unexpected XR service package layout')
            source = sources[0].parent
            prepared = unpacked / 'minimal-service'
            prepared.mkdir()
            for entry in source.iterdir():
                if entry.name in ('lib', 'plugins', 'SDK'):
                    shutil.copytree(entry, prepared / entry.name, symlinks=True)
                elif entry.name in ('RoboticsServiceProcess', 'setting.ini') or '.so' in entry.name:
                    shutil.copy2(entry, prepared / entry.name, follow_symlinks=False)
            if not (prepared / 'SDK/x64/libPXREARobotSDK.so').is_file():
                raise RuntimeError('The x64 XR SDK is missing from the service package')
            prepared.rename(service)
    sdk = service / 'SDK/x64/libPXREARobotSDK.so'
    if not sdk.is_file() or not (service / 'RoboticsServiceProcess').is_file():
        raise RuntimeError(f'Incomplete XR service directory: {service}')
    shutil.copy2(assets['apk'], runtime / assets['apk'].name)
    print(json.dumps({'service': str(service), 'sdk': str(sdk),
                      'apk': str(runtime / assets['apk'].name), 'ubuntu': version}, indent=2))


if __name__ == '__main__':
    main()
