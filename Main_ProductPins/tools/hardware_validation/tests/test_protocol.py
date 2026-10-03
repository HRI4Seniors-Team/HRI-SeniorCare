import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from protocol import encode_vision_packet, parse_hwcsv_line, parse_vision_packet


class ProtocolTests(unittest.TestCase):
    def test_round_trip_crc(self):
        raw = encode_vision_packet({"seq": 3, "face": True, "emo": [0.1, 0.2, 0.3, 0.4]})
        packet = parse_vision_packet(raw)
        self.assertTrue(packet["crc_valid"])
        self.assertEqual(packet["seq"], 3)

    def test_tamper_is_detected(self):
        raw = encode_vision_packet({"seq": 3, "face": True, "emo": [0.1, 0.2, 0.3, 0.4]})
        tampered = raw.replace(b'"seq":3', b'"seq":4')
        self.assertFalse(parse_vision_packet(tampered)["crc_valid"])

    def test_hwcsv(self):
        record = parse_hwcsv_line("HWCSV,SER,123,7,480,4200,1,200000")
        self.assertIsNotNone(record)
        self.assertEqual(record.kind, "SER")
        self.assertEqual(record.seq, 7)


if __name__ == "__main__":
    unittest.main()
