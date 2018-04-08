import unittest
from oars_gb_pkg.nodes.acting.motor_controller import *


class TestMotorController(unittest.TestCase):

    def test_convert_degrees(self):
        self.motor = MotorController()

        expected_output = 0.5
        self.assertEqual(self.motor.convert_degrees(0), expected_output)

        expected_output = 0
        self.assertEqual(self.motor.convert_degrees(-90), expected_output)

        expected_output = 1
        self.assertEqual(self.motor.convert_degrees(90), expected_output)

        expected_output = 0
        self.assertEqual(self.motor.convert_degrees(-100), expected_output)

        expected_output = 1
        self.assertEqual(self.motor.convert_degrees(100), expected_output)

        expected_output = 0.75
        self.assertEqual(self.motor.convert_degrees(45), expected_output)

        expected_output = 0.25
        self.assertEqual(self.motor.convert_degrees(-45), expected_output)


if __name__ == '__main__':
    # Run the tests
    unittest.main()
