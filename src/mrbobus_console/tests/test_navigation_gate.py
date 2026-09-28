import unittest,time,threading
from types import SimpleNamespace as S
from rclpy.clock import Clock
from geometry_msgs.msg import TwistStamped
from mrbobus_console.navigation import Navigation

class NavigationGateTests(unittest.TestCase):
    def setUp(self):
        self.sent=[];self.zeros=[];self.clock=Clock();now=time.monotonic()
        self.n=S(gate=S(lock=threading.RLock(),active=True),lock=threading.RLock(),lio_time=now,cloud_time=now,axes={str(i):{'state':8,'error':0,'at':now} for i in range(4)},pub=S(publish=self.sent.append),zero=lambda:self.zeros.append(1),get_clock=lambda:self.clock)
        self.nav=Navigation.__new__(Navigation);self.nav.node=self.n;self.nav.active=True;self.nav.goal=None;self.nav.token=0
    def command(self):
        m=TwistStamped();m.header.stamp=self.clock.now().to_msg();m.twist.linear.x=3.;m.twist.angular.z=5.;return m
    def test_limits(self):
        self.nav.velocity(self.command());self.assertEqual(self.sent[0].twist.linear.x,.1);self.assertEqual(self.sent[0].twist.angular.z,.3)
    def test_no_command_without_active_gate(self):
        self.n.gate.active=False;self.nav.velocity(self.command());self.assertFalse(self.sent)
    def test_stale_sensor_cancels_navigation(self):
        self.n.cloud_time-=1;self.nav.velocity(self.command());self.assertFalse(self.sent);self.assertFalse(self.nav.active);self.assertTrue(self.zeros)
    def test_axis_error_cancels_navigation(self):
        self.n.axes['1']['error']=512;self.nav.velocity(self.command());self.assertFalse(self.sent);self.assertFalse(self.nav.active)
