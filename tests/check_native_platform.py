"""Offline tests for native architecture selection and mismatched libraries."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from check_native_library import check


class NativePlatformTest(unittest.TestCase):
    def test_platform_defaults(self):
        for host, expected in [('aarch64', 'aarch64:1'), ('arm64', 'aarch64:1'), ('x86_64', 'x86_64:2')]:
            result = subprocess.run(['bash', '-c',
                'mock_arch=$1; uname() { echo "$mock_arch"; }; source "$2" || exit; '
                'printf "%s:%s" "$LEGBOT_ARCH" "$LEGBOT_DEFAULT_JOBS"',
                'test', host, str(ROOT/'tools/native_platform.sh')], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout, expected)

    def test_reject_arm32(self):
        result = subprocess.run(['bash', '-c', 'uname() { echo armv7l; }; source "$1"',
            'test', str(ROOT/'tools/native_platform.sh')], capture_output=True)
        self.assertNotEqual(result.returncode, 0)

    def test_elf_architecture(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)/'library.so'
            for arch, machine in [('aarch64', 183), ('x86_64', 62)]:
                header=bytearray(20);header[:6]=b'\x7fELF\x02\x01';header[18:20]=machine.to_bytes(2,'little')
                p.write_bytes(header)
                check(p,arch)
                with self.assertRaisesRegex(ValueError,'不匹配'):
                    check(p,'x86_64' if arch=='aarch64' else 'aarch64')
            p.write_bytes(b'not a native library')
            with self.assertRaisesRegex(ValueError,'ELF'):
                check(p,'aarch64')


if __name__ == '__main__':
    unittest.main()
