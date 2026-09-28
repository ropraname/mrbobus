import math
import struct
import unittest
from robot_base.core import Base, SIGNS, wheels, frame, unpack, signed_delta, RADIUS

class CoreTests(unittest.TestCase):
    def setUp(self):
        self.t = 10.
        self.tx = []
        self.b = Base(self.tx.append,lambda:self.t,True)
        self.feedback()

    def feedback(self, state=1, error=0, counts=None):
        for n in SIGNS:
            self.b.receive(frame(n,1,struct.pack('<IBBBB',error,state,0,0,0)),self.t)
            self.b.receive(frame(n,10,struct.pack('<ii',(counts or {}).get(n,100),28)),self.t)

    def tick(self, state=1, error=0, command=False):
        self.t += .05
        self.feedback(state,error)
        if command:
            self.b.command('owner',round(self.t*100),.05,0,self.t)
        self.b.tick(self.t)

    def running(self):
        self.assertTrue(self.b.arm('owner',self.t)[0])
        for _ in range(6):
            self.tick(8,command=True)
        self.assertEqual(self.b.state,'RUNNING')

    def test_codec(self):
        raw=frame(3,13,struct.pack('<ff',.2,0))
        self.assertEqual(unpack(raw)[:2],(3,13))
        self.assertIsNone(unpack(frame(2,10,rtr=True)))
        self.assertIsNone(unpack(b'bad'))

    def test_signs_and_curvature(self):
        f=wheels(.1,0)
        self.assertEqual([math.copysign(1,f[n]) for n in SIGNS],[1,-1,-1,1])
        self.assertAlmostEqual(max(abs(v) for v in f.values()),.1/(2*math.pi*RADIUS))
        turn=wheels(0,1)
        self.assertTrue(all(v>0 for v in turn.values()))
        with self.assertRaises(ValueError):wheels(float('nan'),0)

    def test_no_session_or_rpm_cap(self):
        self.assertGreater(max(abs(v) for v in wheels(1.,0).values()), .2)
        self.running()
        for _ in range(400):
            self.tick(8,command=True)
        self.assertEqual(self.b.state,'RUNNING')

    def test_disabled_startup(self):
        self.b.hardware_enabled=False
        self.assertFalse(self.b.arm('owner',self.t)[0])
        self.b.tick(self.t)
        self.assertFalse(any(unpack(v) and unpack(v)[1]==7 for v in self.tx))

    def test_arm_sets_requested_current_limit(self):
        self.assertTrue(self.b.arm('owner', self.t)[0])
        limits = [struct.unpack('<ff', data) for _,cmd,data in self.b.pending if cmd == 0x0F]
        self.assertEqual(limits, [(2.0, 15.0)] * 4)

    def test_arm_setup_does_not_expire_before_first_command(self):
        self.assertTrue(self.b.arm('owner', self.t)[0])
        for _ in range(7):
            self.tick(8)
        self.assertEqual(self.b.state, 'RUNNING')
        self.tick(8)
        self.assertEqual(self.b.state, 'RUNNING')
        for _ in range(7):
            self.tick(8)
        self.assertIn(self.b.state, ('STOPPING', 'IDLE'))
        self.assertNotEqual(self.b.state, 'RUNNING')

    def test_owner_and_replay(self):
        self.running()
        self.assertFalse(self.b.command('other',999,.05,0,self.t))
        self.assertFalse(self.b.command('owner',1,.05,0,self.t))
        self.assertFalse(self.b.arm('other',self.t)[0])

    def test_ramp_and_command_timeout(self):
        self.running()
        old=dict(self.b.output)
        self.tick(8,command=True)
        self.assertLessEqual(max(abs(self.b.output[n]-old[n]) for n in SIGNS),.010001)
        for _ in range(7):self.tick(8)
        self.assertIn(self.b.state,('STOPPING','IDLE'))
        for _ in range(25):self.tick(8 if self.b.state=='STOPPING' else 1)
        self.assertEqual(self.b.state,'IDLE')
        self.assertIsNone(self.b.owner)

    def test_stale_feedback_no_queries_after_fault(self):
        self.running()
        self.t += .6
        self.b.tick(self.t)
        self.assertEqual(self.b.state,'FAULT')
        self.t += .5;self.tx.clear();self.b.tick(self.t)
        self.assertEqual(self.tx,[])

    def test_motor_fault_stops_all(self):
        self.running()
        self.b.receive(frame(2,1,struct.pack('<IBBBB',64,1,0,0,0)),self.t)
        self.tx.clear();self.b.tick(self.t)
        self.assertEqual(self.b.state,'FAULT')
        ids=[struct.unpack_from('=I',f)[0] for f in self.tx]
        self.assertEqual({i>>5 for i in ids},set(SIGNS))
        self.assertTrue(all(i & 31 == 7 for i in ids))

    def test_restart_no_odometry_jump(self):
        self.b.tick(self.t)
        before=(self.b.x,self.b.y,self.b.yaw)
        self.t += 1
        self.feedback(counts={n:0 for n in SIGNS});self.b.tick(self.t)
        self.assertEqual((self.b.x,self.b.y,self.b.yaw),before)

    def test_raw_odometry(self):
        self.running()
        self.b.tick(self.t)
        self.t+=.1
        self.feedback(state=8,counts={n:100+SIGNS[n] for n in SIGNS});self.b.tick(self.t)
        self.assertAlmostEqual(self.b.x,2*math.pi*RADIUS/72)
        self.assertAlmostEqual(self.b.yaw,0)

    def test_counter_wrap(self):
        self.assertEqual(signed_delta(-2**31,2**31-1),1)

    def test_reboot_active_fault(self):
        self.running()
        self.feedback(state=1)
        self.assertEqual(self.b.state,'FAULT')

    def test_explicit_reset_no_rearm(self):
        self.b.fault('test',self.t)
        self.assertTrue(self.b.reset(self.t)[0])
        self.assertEqual(self.b.state,'IDLE')
        self.assertFalse(self.b.command('owner',999,.1,0,self.t))

if __name__=='__main__':unittest.main()
