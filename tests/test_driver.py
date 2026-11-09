import unittest

from f32c import (
    Command,
    F32CBus,
    FeedbackTimeoutError,
    FeedbackType,
    Mode,
)
from f32c.protocol import build_feedback_frame_for_test


class FakeSerial:
    def __init__(self, *, auto_respond=True, chunk_size=2):
        self.port = "FAKE"
        self.is_open = True
        self.auto_respond = auto_respond
        self.chunk_size = chunk_size
        self.rx = bytearray()
        self.writes = []

    @property
    def in_waiting(self):
        return len(self.rx)

    def reset_input_buffer(self):
        self.rx.clear()

    def write(self, data):
        data = bytes(data)
        self.writes.append(data)
        if self.auto_respond and len(data) == 6 and data[2] == Command.FEEDBACK:
            address = data[1]
            kind = FeedbackType(data[3])
            payloads = {
                FeedbackType.SPEED: b"\x00\x00\xFF\xCE",
                FeedbackType.MULTI_ANGLE: (1234).to_bytes(4, "big", signed=True),
                FeedbackType.SINGLE_ANGLE: b"\x00\x00\x00\x64",
                FeedbackType.ACCELERATION: b"\x00\x00\x00\x14",
                FeedbackType.VOLTAGE: b"\x00\x00\x09\x6F",
            }
            self.rx.extend(build_feedback_frame_for_test(address, kind, payloads[kind]))
        return len(data)

    def read(self, size):
        size = min(size, self.chunk_size, len(self.rx))
        result = bytes(self.rx[:size])
        del self.rx[:size]
        return result

    def close(self):
        self.is_open = False


class DriverTests(unittest.TestCase):
    def test_enable_and_mode_frames_match_examples(self):
        fake = FakeSerial()
        with F32CBus(serial_instance=fake, command_interval=0) as bus:
            bus.enable(1)
            bus.set_mode(1, Mode.MULTI_POSITION_DIRECT)
        self.assertEqual(fake.writes[0], bytes((0x7A, 0x01, 0x06, 0x7D, 0x7B)))
        self.assertEqual(fake.writes[1], bytes((0x7A, 0x01, 0x00, 0x00, 0x03, 0x78, 0x7B)))

    def test_fragmented_feedback_round_trip(self):
        fake = FakeSerial(chunk_size=1)
        with F32CBus(serial_instance=fake, command_interval=0) as bus:
            self.assertAlmostEqual(bus.read_multi_angle(2), 123.4)
            self.assertEqual(bus.read_speed_rpm(2), -50)
            self.assertAlmostEqual(bus.read_voltage(2), 24.15)
        self.assertEqual(fake.writes[0], bytes((0x7A, 0x02, 0x0E, 0x01, 0x77, 0x7B)))

    def test_negative_speed_is_packed_as_signed_16_bit(self):
        fake = FakeSerial()
        with F32CBus(serial_instance=fake, command_interval=0) as bus:
            bus.set_speed_rpm(1, -50)
        self.assertEqual(fake.writes[0][3:5], b"\xFF\xCE")

    def test_motor_convenience_wrapper(self):
        fake = FakeSerial()
        with F32CBus(serial_instance=fake, command_interval=0) as bus:
            motor = bus.motor(1)
            motor.set_angle(12.3)
            self.assertAlmostEqual(motor.read_angle(), 123.4)
        self.assertEqual(fake.writes[0][3:7], b"\x00\x00\x00\x7B")

    def test_timeout(self):
        fake = FakeSerial(auto_respond=False)
        with F32CBus(serial_instance=fake, command_interval=0) as bus:
            with self.assertRaises(FeedbackTimeoutError):
                bus.read_multi_angle(1, timeout=0.01)

    def test_invalid_values_are_rejected_before_writing(self):
        fake = FakeSerial()
        with F32CBus(serial_instance=fake, command_interval=0) as bus:
            with self.assertRaises(ValueError):
                bus.set_single_angle(1, 360.0)
            with self.assertRaises(ValueError):
                bus.set_speed_rpm(1, 40000)
        self.assertEqual(fake.writes, [])


if __name__ == "__main__":
    unittest.main()
