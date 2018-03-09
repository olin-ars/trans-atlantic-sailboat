import unittest
from oars_gb_pkg.nodes.thinking.waypoint_navigator import WaypointNavigator
try:
    from geometry_msgs.msg import Pose2D
except ImportError:
    from tests.mock_ros_msgs import Pose2D


class TestWaypointGeneration(unittest.TestCase):
    """
    Tests the conversion of a list of GridMap
    cells to navigate into a list of GPS waypoints.
    """

    def test_calculate_desired_heading(self):
        wn = WaypointNavigator(6, use_ros=False)
        test_position = self.create_pose2d(1, 1)
        wn.update_location(test_position)
        wn.update_wp_list([(3, 3)])
        # Calculate heading
        self.assertEqual(wn.calculate_desired_heading(), 45.0)

    def test_check_distance_to_wp(self):
        wn = WaypointNavigator(6, use_ros=False)
        test_position = self.create_pose2d(0, 0)
        wn.update_location(test_position)
        wn.update_wp_list([(3, 4), (5, 12), (7, 24)])
        # Calculate heading
        self.assertEqual(wn.have_reached_wp(), True)

    @staticmethod
    def create_pose2d(x=0, y=0, theta=0):
        pos = Pose2D()
        pos.x = x
        pos.y = y
        pos.theta = theta
        return pos


if __name__ == '__main__':
    # Run the tests
    unittest.main()
