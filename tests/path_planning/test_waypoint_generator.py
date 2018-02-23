import unittest
from oars_gb.nodes.thinking.path_planning.waypoint_generator import WaypointGenerator


class TestWaypointGeneration(unittest.TestCase):
    """
    Tests the conversion of a list of GridMap cells to navigate into a list of GPS waypoints.
    """

    def test_waypoint_generation(self):
        wp_gen = WaypointGenerator()

        test_1_input = [(0, 0), (1, 1), (2, 2), (1, 3)]
        test_1_output = [(2, 2), (1, 3)]
        self.assertListEqual(wp_gen.make_waypoints(test_1_input), test_1_output)

        test_2_input = [(0, 0), (1, 1), (2, 2), (3, 3)]
        test_2_output = [(3, 3)]
        self.assertListEqual(wp_gen.make_waypoints(test_2_input), test_2_output)

        test_3_input = [(0, 0), (1, 0), (2, 0), (3, 0)]
        test_3_output = [(3, 0)]
        self.assertListEqual(wp_gen.make_waypoints(test_3_input), test_3_output)

        test_4_input = [(1, 2), (2, 2), (2, 1), (3, 0)]
        test_4_output = [(2, 2), (2, 1), (3, 0)]
        self.assertListEqual(wp_gen.make_waypoints(test_4_input), test_4_output)

        test_5_input = [(1, 1), (2, 1), (2, 2), (1, 2), (0, 2), (0, 1), (0, 0), (1, 0), (2, 0), (3, 0), (3, 1), (3, 2), (3, 3),
             (2, 3), (1, 3), (0, 3)]
        test_5_output = [(2, 1), (2, 2), (0, 2), (0, 0), (3, 0), (3, 3), (0, 3)]
        self.assertListEqual(wp_gen.make_waypoints(test_5_input), test_5_output)


if __name__ == '__main__':
    # Run the tests
    unittest.main()
