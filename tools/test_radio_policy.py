import unittest
from radio_policy import RadioPolicy


def controls(manual=True, gear=191, stop=191, gas=225, turn=992):
    ch = [992] * 16
    ch[0], ch[2], ch[4], ch[5], ch[7] = turn, gas, 1792 if manual else 191, gear, stop
    return ch


class RadioTests(unittest.TestCase):
    def setUp(self):
        self.r = RadioPolicy()

    def update(self, **kw):
        self.r.link(100, 10.)
        self.r.update(controls(**kw), 10.)

    def test_startup_auto_but_no_arm_event(self):
        self.assertEqual(self.r.status(1)['mode'], 'auto')
        self.assertEqual(self.r.arm_token, 0)

    def test_manual_loss_auto_clears_command_and_stops_once(self):
        self.update(gas=1811, turn=1811)
        before = self.r.stop_token
        self.r.tick(10.31)
        self.assertEqual(self.r.mode, 'auto')
        self.assertEqual((self.r.throttle, self.r.steering, self.r.gear), (0, 0, 0))
        self.assertEqual(self.r.stop_token, before+1)
        self.assertEqual(self.r.arm_token, 0)
        self.r.tick(11)
        self.assertEqual(self.r.stop_token, before+1)

    def test_radio_stop_clears_on_disconnect_without_arm(self):
        self.update(stop=1792)
        self.r.tick(11)
        self.assertEqual(self.r.mode, 'auto')
        self.assertFalse(self.r.held)
        self.assertTrue(self.r.may_arm(11))
        self.assertEqual(self.r.arm_token, 0)
    def test_reconnect_pressed_stop_blocks_auto_again(self):
        self.update(manual=False, stop=1792)
        self.r.tick(11)
        self.assertFalse(self.r.held)
        self.r.link(100,12)
        self.r.update(controls(manual=False,stop=1792),12)
        self.assertTrue(self.r.held)
        self.assertFalse(self.r.may_arm(12))

    def test_auto_loss_does_not_interrupt_autonomy(self):
        self.update(manual=False)
        before = self.r.stop_token
        self.r.tick(11)
        self.assertEqual(self.r.stop_token, before)

    def test_auto_ignores_gears_and_sticks(self):
        self.update(manual=False)
        before = self.r.stop_token
        self.update(manual=False, gear=1792, gas=1811, turn=1811)
        self.assertEqual(self.r.stop_token, before)
        self.assertEqual(self.r.arm_token, 0)

    def test_reconnect_never_arms(self):
        self.update()
        self.r.tick(11)
        self.r.link(100, 12)
        self.r.update(controls(), 12)
        self.assertEqual(self.r.mode, 'manual')
        self.assertEqual(self.r.arm_token, 0)

    def test_gear_requires_zero_gas(self):
        self.update(gear=997)
        self.update(gear=191, gas=1811)
        self.assertEqual(self.r.arm_token, 0)
        self.update(gear=997)
        self.update(gear=191)
        self.assertEqual(self.r.arm_token, 1)

    def test_link_quality_zero_triggers_loss(self):
        self.update()
        self.r.link(0, 10.01)
        self.r.tick(10.01)
        self.assertEqual(self.r.mode, 'auto')
    def test_arming_requires_centered_steering(self):
        self.update(gear=997)
        self.update(gear=191, turn=1811)
        self.assertEqual(self.r.arm_token,0)
        self.assertFalse(self.r.may_arm(10))


if __name__ == '__main__': unittest.main()
