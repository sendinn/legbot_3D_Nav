"""Reject wrong-architecture runtime libraries before invoking CMake."""
import argparse
from pathlib import Path
import struct


def check(path, architecture):
    with Path(path).open('rb') as stream:
        header = stream.read(20)
    if len(header) != 20 or header[:4] != b'\x7fELF' or header[4] != 2 or header[5] not in (1, 2):
        raise ValueError(f'{path}: 不是有效的 64 位 ELF 库')
    machine = struct.unpack('<H' if header[5] == 1 else '>H', header[18:20])[0]
    if machine != {'x86_64': 62, 'aarch64': 183}[architecture]:
        raise ValueError(f'{path}: ELF 架构 {machine} 与 {architecture} 不匹配，请勿复用其他机器的 third_party')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('architecture', choices=['x86_64', 'aarch64'])
    parser.add_argument('libraries', nargs='+')
    args = parser.parse_args()
    try:
        for path in args.libraries:
            check(path, args.architecture)
    except (ValueError, OSError) as error:
        parser.exit(1, str(error)+'\n')
