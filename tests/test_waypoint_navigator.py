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

    # def test_calculate_desired_heading(self):
    #     wn = WaypointNavigator(0.5)
    #     test_position = Pose2D()
    #     test_position.x = 1
    #     test_position.y = 1
    #     wn.update_location(test_position, False)
    #     wn.update_wp_list([(4,4), (6,6), (6,8), (8,8), (10,8), (11,11), (13,13)],False)
    #     # Calculate heading
    #     self.assertEqual(wn.calculate_desired_heading(), 45.0)
    #     print(wn.wp_list)

    def test_check_distance_to_wp(self):
        wn = WaypointNavigator(9)
        test_position = Pose2D()
        test_position.x = 0
        test_position.y = 0

        wn.update_wp_list([(4, 4), (6, 6), (6, 8), (8, 8), (10, 8), (11, 11), (13, 13)])
        wn.update_location(test_position)

        #radius = 6
        # Calculate heading
        self.assertEqual(wn.have_reached_wp(), True)
        print(wn.wp_list)

    def test_check_distance_to_wp2(self):
        wn = WaypointNavigator(13)
        test_position = Pose2D()
        test_position.x = 0
        test_position.y = 0

        wn.update_wp_list([(4, 4), (6, 6), (6, 8), (8, 8), (10, 8), (11, 11), (13, 13)])
        wn.update_location(test_position)

        # radius = 6
        # Calculate heading
        self.assertEqual(wn.have_reached_wp(), True)
        print(wn.wp_list)

    def test_check_distance_to_wp3(self):
        wn = WaypointNavigator(10)
        test_position = Pose2D()
        test_position.x = 0
        test_position.y = 0

        wn.update_wp_list([(4, 4), (6, 6), (6, 8), (8, 8), (10, 8), (11, 11), (13, 13)])
        wn.update_location(test_position)

        # radius = 6
        # Calculate heading
        self.assertEqual(wn.have_reached_wp(), True)
        print(wn.wp_list)

    def test_check_distance_to_wp4(self):
        wn = WaypointNavigator(12)
        test_position = Pose2D()
        test_position.x = 0
        test_position.y = 0

        wn.update_wp_list([(4, 4), (6, 6), (6, 8), (8, 8), (10, 8), (11, 11), (13, 13)])
        wn.update_location(test_position)

        # radius = 6
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
