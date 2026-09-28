import math
import unittest
from mrbobus_console.control import MotionGate

class MotionGateTest(unittest.TestCase):
    def setUp(self):
        self.gate = MotionGate()
        self.owner = 'test-console'
        self.gate.finish_arm(self.gate.begin_arm(self.owner))

    def test_stop_cancels_inflight_arm(self):
        epoch = self.gate.begin_arm(self.owner)
        self.gate.stop()
        with self.assertRaises(ValueError):
            self.gate.finish_arm(epoch)
        with self.assertRaises(ValueError):
            self.gate.command(self.owner, .03, 0, 1)

    def test_late_motion_cannot_override_release(self):
        self.gate.command(self.owner, .03, 0, 10)
        self.assertEqual(self.gate.command(self.owner, 0, 0, 12), (0, 0))
        with self.assertRaises(ValueError):
            self.gate.command(self.owner, .03, 0, 11)

    def test_other_tab_cannot_drive_or_take_over(self):
        with self.assertRaises(ValueError):
            self.gate.command('other-console', .03, 0, 1)
        with self.assertRaises(ValueError):
            self.gate.begin_arm('other-console')

    def test_combined_command_respects_both_wheel_limits(self):
        for seq, (v, w) in enumerate(((.06, .3), (-.06, .3), (.06, -.3), (-.06, -.3))):
            v, w = self.gate.command(self.owner, v, w, seq)
            for wheel in (v+w*.29174/2, v-w*.29174/2):
                self.assertLessEqual(abs(wheel)/.045, 1.5707963268)

    def test_invalid_velocity_cannot_poison_sequence(self):
        for v in (math.nan, math.inf, True):
            with self.assertRaises(ValueError):
                self.gate.command(self.owner, v, 0, 5)
        self.assertEqual(self.gate.command(self.owner, 0, 0, 5), (0, 0))

if __name__ == '__main__':
    unittest.main()
