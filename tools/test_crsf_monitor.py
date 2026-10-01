"""Protocol checks without UART, ROS, or motors."""
import unittest
from crsf_monitor import Parser, channels, crc8


class CRSFTests(unittest.TestCase):
    def frame(self, values):
        packed = sum(v << (i * 11) for i, v in enumerate(values))
        body = bytes([0x16]) + packed.to_bytes(22, 'little')
        return bytes([0xc8, len(body) + 1]) + body + bytes([crc8(body)])

    def test_crc_reference(self):
        self.assertEqual(crc8(b'123456789'), 0xbc)

    def test_channels_and_split_packet(self):
        values = [172, 992, 1811, 2047, 0, 1000, 400, 1600] * 2
        frame = self.frame(values)
        for split in range(len(frame)):
            parser = Parser()
            result = parser.feed(frame[:split]) + parser.feed(frame[split:])
            self.assertEqual(len(result), 1)
            self.assertEqual(channels(result[0][1]), values)

    def test_corruption_and_resync(self):
        good = self.frame([992] * 16)
        bad = bytearray(good)
        bad[-1] ^= 1
        parser = Parser()
        result = parser.feed(b'\xff\xff\xff' + bad + good * 4)
        self.assertTrue(result)
        self.assertTrue(all(channels(payload) == [992] * 16 for _, payload in result))
        self.assertGreater(parser.bad_crc, 0)

    def test_reject_wrong_payload(self):
        with self.assertRaises(ValueError):
            channels(bytes(21))


if __name__ == '__main__':
    unittest.main()
