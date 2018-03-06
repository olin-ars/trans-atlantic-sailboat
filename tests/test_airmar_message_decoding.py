import unittest
from oars_gb_pkg.nodes.sensing.airmar import *
from geometry_msgs.msg import Pose2D


class TestAirmarMessageDecoding(unittest.TestCase):
    """
    Tests the conversion of a list of GridMap cells to navigate into a list of GPS waypoints.
    """

    def setUp(self):
        self.parser = AirmarParser(port=None, use_ros=False)
        # msg = '$GPGLL,4217.6274,N,07115.8634,W,202754.00,A,A*71'.split(',')

    def test_gps_message_decoding(self):
        msg = '$GPGLL,3500.0000,N,02500.0000,E,202754.00,A,A*71'.split(',')
        expected_output = Pose2D(x=25, y=35)
        self.assertEqual(self.parser.parse_gps_message(msg), expected_output)

        msg = '$GPGLL,3500.0000,S,02500.0000,W,202754.00,A,A*71'.split(',')
        expected_output = Pose2D(x=-25, y=-35)
        self.assertEqual(self.parser.parse_gps_message(msg), expected_output)

        msg = '$GPGLL,1510.2500,N,02515.5000,E,202754.00,A,A*71'.split(',')
        expected_output = Pose2D(x=self.to_deg(25, 15.5), y=self.to_deg(15, 10.25))
        self.assertEqual(self.parser.parse_gps_message(msg), expected_output)

        msg = '$GPGLL,0001.0200,S,00500.1000,E,202754.00,A,A*71'.split(',')
        expected_output = Pose2D(x=self.to_deg(5, 0.1), y=self.to_deg(0, -1.02))
        self.assertEqual(self.parser.parse_gps_message(msg), expected_output)

        msg = '$GPGLL,8024.1230,S,12501.2000,E,202754.00,A,A*71'.split(',')
        expected_output = Pose2D(x=self.to_deg(125, 1.2), y=self.to_deg(-80, 24.123))
        self.assertEqual(self.parser.parse_gps_message(msg), expected_output)

    @staticmethod
    def to_deg(degrees, minutes):
        return degrees + minutes / 60.0


if __name__ == '__main__':
    # Run the tests
    unittest.main()
