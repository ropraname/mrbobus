"""Latched stop state tests; no ROS, network, CAN or motors."""
import tempfile
import unittest
from pathlib import Path
from stop_panel import StopState

class StopTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name)/'enabled'
        self.state = StopState(self.path)
    def tearDown(self): self.temp.cleanup()
    def test_startup_stopped(self):
        self.assertFalse(self.state.status()['ready'])
        self.assertEqual(self.path.read_text(), '0')
    def test_stop_latches_until_explicit_reset(self):
        self.state.ready(); self.assertEqual(self.path.read_text(), '1')
        self.state.stop(); self.assertFalse(self.state.status()['ready'])
        self.assertEqual(self.path.read_text(), '0')
        self.state.ready(); self.assertEqual(self.path.read_text(), '1')
    def test_server_restart_latches_stop(self):
        self.state.ready()
        restarted = StopState(self.path)
        self.assertFalse(restarted.status()['ready'])
        self.assertEqual(self.path.read_text(), '0')

if __name__ == '__main__': unittest.main()
