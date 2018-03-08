import unittest
from geometry_msgs.msg import Pose2D
from oars_gb_pkg.nodes.thinking.waypoint_navigator import WaypointNavigator

class TestWaypointGeneration(unittest.TestCase):
    """
    Tests the conversion of a list of GridMap
    cells to navigate into a list of GPS waypoints.
    """

    def test_calculate_desired_heading(self):
        wn = WaypointNavigator(6)
        test_position = Pose2D()
        test_position.x = 1
        test_position.y = 1
        wn.update_location(test_position,False)
        wn.update_wp_list([(3,3)],False)
        # Calculate heading
        self.assertEqual(wn.calculate_desired_heading(), 45.0)

    def test_check_distance_to_wp(self):
        wn = WaypointNavigator(6)
        test_position = Pose2D()
        test_position.x = 0
        test_position.y = 0
        wn.update_location(test_position,False)
        wn.update_wp_list([(3,4),(5,12),(7,24)],False)
        #radius = 6
        # Calculate heading
        self.assertEqual(wn.have_reached_wp(), True)


if __name__ == '__main__':

    unittest.main()
