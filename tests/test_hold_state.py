import importlib.util
import unittest
from pathlib import Path


EXAMPLE_PATH = Path(__file__).resolve().parents[1] / "examples" / "two_axis_position.py"
SPEC = importlib.util.spec_from_file_location("two_axis_position_example", EXAMPLE_PATH)
EXAMPLE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXAMPLE)


class HoldStateTests(unittest.TestCase):
    def test_timeout_can_happen_before_first_target_reached(self):
        reached_at, event = EXAMPLE.update_hold_phase(
            in_tolerance=False, now=11.0, deadline=10.0, reached_at=None, hold_time=0
        )
        self.assertIsNone(reached_at)
        self.assertEqual(event, "timeout")

    def test_infinite_hold_ignores_later_feedback_jitter_and_old_deadline(self):
        reached_at, event = EXAMPLE.update_hold_phase(
            in_tolerance=True, now=5.0, deadline=10.0, reached_at=None, hold_time=0
        )
        self.assertEqual(reached_at, 5.0)
        self.assertEqual(event, "reached")

        for now in (5.1, 10.0, 10.1, 100.0):
            reached_at, event = EXAMPLE.update_hold_phase(
                in_tolerance=False,
                now=now,
                deadline=10.0,
                reached_at=reached_at,
                hold_time=0,
            )
            self.assertEqual(reached_at, 5.0)
            self.assertEqual(event, "holding")

    def test_reached_event_is_not_repeated(self):
        reached_at, first_event = EXAMPLE.update_hold_phase(
            in_tolerance=True, now=5.0, deadline=10.0, reached_at=None, hold_time=0
        )
        reached_at, second_event = EXAMPLE.update_hold_phase(
            in_tolerance=True,
            now=5.2,
            deadline=10.0,
            reached_at=reached_at,
            hold_time=0,
        )
        self.assertEqual(first_event, "reached")
        self.assertEqual(second_event, "holding")
        self.assertEqual(reached_at, 5.0)

    def test_finite_hold_finishes_from_first_reached_time_despite_jitter(self):
        reached_at, _ = EXAMPLE.update_hold_phase(
            in_tolerance=True, now=5.0, deadline=10.0, reached_at=None, hold_time=5.0
        )
        reached_at, event = EXAMPLE.update_hold_phase(
            in_tolerance=False, now=8.0, deadline=10.0, reached_at=reached_at, hold_time=5.0
        )
        self.assertEqual(event, "holding")
        reached_at, event = EXAMPLE.update_hold_phase(
            in_tolerance=False, now=10.0, deadline=10.0, reached_at=reached_at, hold_time=5.0
        )
        self.assertEqual(event, "finished")
        self.assertEqual(reached_at, 5.0)


if __name__ == "__main__":
    unittest.main()
