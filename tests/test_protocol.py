import unittest

from f32c.protocol import (
    Command,
    FeedbackParser,
    FeedbackType,
    ProtocolError,
    build_feedback_frame_for_test,
    build_frame,
    calculate_bcc,
    parse_feedback_frame,
)


class ProtocolTests(unittest.TestCase):
    def test_bcc_and_reference_enable_frames(self):
        self.assertEqual(calculate_bcc(bytes((0x7A, 0x01, 0x06))), 0x7D)
        self.assertEqual(
            build_frame(1, Command.ENABLE), bytes((0x7A, 0x01, 0x06, 0x7D, 0x7B))
        )
        self.assertEqual(
            build_frame(2, Command.ENABLE), bytes((0x7A, 0x02, 0x06, 0x7E, 0x7B))
        )

    def test_reference_feedback_requests(self):
        self.assertEqual(
            build_frame(1, Command.FEEDBACK, b"\x01"),
            bytes((0x7A, 0x01, 0x0E, 0x01, 0x74, 0x7B)),
        )
        self.assertEqual(
            build_frame(2, Command.FEEDBACK, b"\x00"),
            bytes((0x7A, 0x02, 0x0E, 0x00, 0x76, 0x7B)),
        )

    def test_signed_multi_angle_payload(self):
        self.assertEqual(
            build_frame(1, Command.MULTI_POSITION, (-1234).to_bytes(4, "big", signed=True)),
            bytes((0x7A, 0x01, 0x02, 0xFF, 0xFF, 0xFB, 0x2E, 0xAC, 0x7B)),
        )

    def test_parse_multi_angle(self):
        frame = build_feedback_frame_for_test(
            1, FeedbackType.MULTI_ANGLE, (-1234).to_bytes(4, "big", signed=True)
        )
        result = parse_feedback_frame(frame)
        self.assertEqual(result.address, 1)
        self.assertEqual(result.raw_value, -1234)
        self.assertAlmostEqual(result.value, -123.4)
        self.assertEqual(result.unit, "deg")

    def test_parse_speed_single_angle_and_voltage(self):
        speed = parse_feedback_frame(
            build_feedback_frame_for_test(1, FeedbackType.SPEED, b"\x00\x00\xFF\xCE")
        )
        self.assertEqual(speed.raw_value, -50)
        self.assertEqual(speed.unit, "rpm")

        angle = parse_feedback_frame(
            build_feedback_frame_for_test(2, FeedbackType.SINGLE_ANGLE, b"\x00\x00\x0E\x0F")
        )
        self.assertAlmostEqual(angle.value, 359.9)

        voltage = parse_feedback_frame(
            build_feedback_frame_for_test(2, FeedbackType.VOLTAGE, b"\x00\x00\x09\x6F")
        )
        self.assertAlmostEqual(voltage.value, 24.15)
        self.assertEqual(voltage.unit, "V")

    def test_bad_checksum_is_rejected(self):
        frame = bytearray(
            build_feedback_frame_for_test(1, FeedbackType.MULTI_ANGLE, b"\x00\x00\x00\x00")
        )
        frame[7] ^= 0x01
        with self.assertRaises(ProtocolError):
            parse_feedback_frame(bytes(frame))

    def test_stream_parser_handles_noise_fragments_and_resync(self):
        first = build_feedback_frame_for_test(
            1, FeedbackType.MULTI_ANGLE, (100).to_bytes(4, "big", signed=True)
        )
        second = build_feedback_frame_for_test(
            2, FeedbackType.SPEED, b"\x00\x00\x00\x32"
        )
        damaged = bytearray(first)
        damaged[-1] = 0
        stream = b"noise" + bytes(damaged) + first + second

        parser = FeedbackParser()
        results = []
        for byte in stream:
            results.extend(parser.feed(bytes((byte,))))
        self.assertEqual(len(results), 2)
        self.assertEqual(results[0].value, 10.0)
        self.assertEqual(results[1].raw_value, 50)


if __name__ == "__main__":
    unittest.main()
