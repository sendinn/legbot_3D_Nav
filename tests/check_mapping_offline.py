#!/usr/bin/env python3
"""Offline behavioral tests; no ROS nodes or robot commands."""
import importlib.util
import sys
from pathlib import Path
import struct
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
spec = importlib.util.spec_from_file_location('mapping', ROOT / 'tools/mapping.py')
mapping = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mapping)


class MappingTests(unittest.TestCase):
    def test_keyboard_deadman_and_stop(self):
        command = mapping.KeyboardCommand(0.3, 0.4)
        command.press('w', 10.0)
        self.assertEqual(command.current(10.1), (0.3, 0., 0.))
        self.assertEqual(command.current(10.5), (0., 0., 0.))
        command.press('a', 11.0)
        self.assertEqual(command.current(11.1), (0., 0.25, 0.))
        command.press(' ', 11.2)
        self.assertEqual(command.current(11.21), (0., 0., 0.))
        command.press('E', 12.0)
        self.assertEqual(command.current(12.1), (0., 0., -0.4))
        command.press('x', 12.2)
        self.assertEqual(command.current(12.21), (0., 0., 0.))

    def test_unknown_key_stops(self):
        command = mapping.KeyboardCommand(0.25, 0.4)
        command.press('w', 0)
        command.press('?', 0.1)
        self.assertEqual(command.current(0.2), (0., 0., 0.))

    def test_pcd_requires_complete_nonempty_cloud(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.pcd'
            header = (b'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n'
                      b'SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
                      b'WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n')
            data = struct.pack('<ffff', 1., 2., 3., 4.)
            path.write_bytes(header + data)
            self.assertEqual(mapping.verify_pcd(path), 1)
            path.write_bytes(header + data[:-1])
            with self.assertRaises(RuntimeError):
                mapping.verify_pcd(path)
            path.write_bytes(header.replace(b'POINTS 1', b'POINTS 0'))
            with self.assertRaises(RuntimeError):
                mapping.verify_pcd(path)

    def test_limits_and_defaults(self):
        args = mapping.arguments([])
        self.assertEqual(args.speed, 0.25)
        for value in ('0', '-1', 'nan', 'inf'):
            with self.assertRaises(Exception):
                mapping.positive(value)

    def test_gpu_environment_is_per_process(self):
        import os
        from unittest.mock import patch
        with patch.dict(os.environ, {'LIBGL_ALWAYS_SOFTWARE': '1', 'GALLIUM_DRIVER': 'llvmpipe'}):
            self.assertNotIn('LIBGL_ALWAYS_SOFTWARE', mapping.rendering_environment('gpu'))
            self.assertEqual(mapping.rendering_environment('software')['GALLIUM_DRIVER'], 'llvmpipe')
            self.assertEqual(os.environ['LIBGL_ALWAYS_SOFTWARE'], '1')


if __name__ == '__main__':
    unittest.main()
