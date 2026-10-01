"""Controller acknowledgement must precede opening the motion gate."""
import threading
import time
import unittest
from types import SimpleNamespace as S
from unittest.mock import patch, MagicMock
from mrbobus_console.control import MotionGate
from mrbobus_console.server import Console


class ArmAcknowledgementTests(unittest.TestCase):
    def node(self, cancel=False):
        n=S(operation=threading.Lock(), lock=threading.RLock(), gate=MotionGate(),
            axes={str(i):dict(state=1,error=0,at=time.monotonic()) for i in range(4)},
            list_client='list',switch_client='switch',zero=lambda:None)
        n.stop=lambda:n.gate.stop()
        def call(client, request, *args):
            if client=='list':return S(controller=[S(name='diff_drive_controller',state='inactive')])
            def acknowledge():
                if cancel:n.gate.stop()
                else:
                    with n.lock:
                        for a in n.axes.values():a.update(state=8,at=time.monotonic())
            t=threading.Timer(.08,acknowledge);t.start();self.addCleanup(t.join)
            return S(ok=True)
        n.call=call
        return n

    def setUp(self):
        for name,kwargs in [('subprocess.run',dict(return_value=S(returncode=0))),
                            ('stop_request',dict(return_value={})),
                            ('Path',dict(return_value=MagicMock())),
                            ('allow_source',dict(return_value=True))]:
            p=patch('mrbobus_console.server.'+name,**kwargs);p.start();self.addCleanup(p.stop)

    def test_wait_for_axis_heartbeat(self):
        n=self.node();before=time.monotonic()
        Console.arm(n,'test-owner')
        self.assertGreaterEqual(time.monotonic()-before,.07)
        self.assertTrue(n.gate.active)
        self.assertTrue(all(a['state']==8 for a in n.axes.values()))

    def test_stop_during_ack_prevents_enable(self):
        n=self.node(cancel=True)
        with self.assertRaises(ValueError):Console.arm(n,'test-owner')
        self.assertFalse(n.gate.active)
    def test_warm_arm_does_not_restart_controller_or_probe_can(self):
        n=self.node();n.control_warm=True
        with patch('mrbobus_console.server.subprocess.run') as run:
            Console.arm(n,'test-owner')
            run.assert_not_called()
        self.assertTrue(n.gate.active)
    def test_warm_arm_resets_active_controller(self):
        n=self.node();n.control_warm=True;original=n.call;deactivated=[]
        def call(client, request, *args):
            if client=='list':return S(controller=[S(name='diff_drive_controller',state='active')])
            if request.deactivate_controllers:
                deactivated.extend(request.deactivate_controllers)
                return S(ok=True)
            return original(client,request,*args)
        n.call=call
        Console.arm(n,'test-owner')
        self.assertEqual(deactivated,['diff_drive_controller'])
        self.assertTrue(n.gate.active)
