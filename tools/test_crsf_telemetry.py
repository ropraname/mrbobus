import struct
import unittest
import threading
from crsf_monitor import Parser
from crsf_telemetry import battery, mode_frame, temperature, Telemetry


class TelemetryTests(unittest.TestCase):
    def payload(self, packet, kind):
        parser = Parser()
        frames = parser.feed(packet)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0][0],kind)
        self.assertEqual(parser.bad_crc,0)
        return frames[0][1]

    def test_voltage_and_unknown_values(self):
        payload = self.payload(battery(37.7),0x08)
        self.assertEqual(payload, b'\x01\x79' + b'\xff'*6)

    def test_mode(self):
        self.assertEqual(self.payload(mode_frame('NEUTRAL'),0x21),b'NEUTRAL\0')

    def test_temperature_scale(self):
        self.assertEqual(self.payload(temperature(56.3),0x0D),b'\0'+struct.pack('>h',563))

    def test_nonfinite_rejected(self):
        for number in [float('nan'),float('inf'),-5]:
            with self.assertRaises(ValueError):battery(number)
    def test_mode_waits_for_real_axis_ack(self):
        t=Telemetry.__new__(Telemetry)
        t.lock=threading.Lock();t.voltages={};t.sent=0
        t.axes={i:(0,1,10.) for i in range(4)}
        r=dict(held=False,mode='manual',gear=1)
        self.assertEqual(self.payload(t.packets(r,True,10.)[0],0x21),b'ARMING\0')
        t.axes={i:(0,8,10.) for i in range(4)}
        self.assertEqual(self.payload(t.packets(r,True,10.)[0],0x21),b'MANUAL\0')


if __name__ == '__main__':unittest.main()
