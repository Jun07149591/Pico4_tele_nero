"""Build a clean source ZIP, excluding runtime data and local configuration."""

import argparse
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]
DIRECTORIES = {'teleop', 'data_collection', 'vendor', 'scripts', 'docs', 'tests', '.github'}
FILES = {'README.md', '.gitignore', 'environment-teleop.yml', 'LICENSE', 'NOTICE'}
SKIP = {'__pycache__', '.git', '.runtime', '.downloads', '.envs', 'artifacts', 'logs', 'data',
        'build', 'dist', '.pytest_cache', 'capture.local.json', '.env'}


def source_files(root):
    for path in sorted(root.rglob('*')):
        relative = path.relative_to(root)
        if any(part in SKIP or part.endswith('.egg-info') or part.startswith('.venv') for part in relative.parts):
            continue
        if relative.parts[0] not in DIRECTORIES and str(relative) not in FILES:
            continue
        if path.is_symlink():
            raise ValueError(f'External or symbolic source dependency: {relative}')
        if path.is_file() and path.suffix not in ('.pyc', '.pyo', '.h5', '.mp4', '.parquet', '.apk', '.deb'):
            yield path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'dist/pico4_tele_nero.zip')
    args = parser.parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED) as archive:
        for path in source_files(ROOT):
            archive.write(path, Path('pico4_tele_nero') / path.relative_to(ROOT))
            count += 1
    print(f'{output}: {count} files, {output.stat().st_size} bytes')


if __name__ == '__main__':
    main()
