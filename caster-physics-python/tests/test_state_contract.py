"""En küçük RED testi bağımlılık kurulmadan da çalışır (standart unittest)."""
import unittest


class StateContract(unittest.TestCase):
    def test_named_state_has_thirteen_round_trip_values(self):
        from caster_physics.state import State
        values = [float(value) for value in range(13)]
        state = State.from_vector(values)
        self.assertEqual(state.body_yaw_rad, 2)
        self.assertEqual(state.caster_angle_relative_rad, 3)
        self.assertEqual(list(state.as_vector()), values)


if __name__ == "__main__":
    unittest.main()
