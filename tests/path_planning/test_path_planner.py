import unittest
from oars_gb_pkg.nodes.thinking.path_planning.path_planner import PathPlanner
from oars_gb_pkg.nodes.thinking.path_planning.grid_map_generator import GridMapGenerator
from tests.mock_ros_msgs import *


class TestPathPlanner(unittest.TestCase):
    """
    Tests the conversion of a list of GridMap cells to navigate into a list of GPS waypoints.
    """

    def test_path_planner(self):
        # Generate a GridMap
        map_generator = GridMapGenerator(using_ros=False)
        map_generator.load_image('maps/waban_42.282368_42.293353_-71.314756_-71.302289.png')
        grid_map = map_generator.publish_map()

        # Create the path planner
        planner = PathPlanner(using_ros=False)

        # Define some mock ROS messages
        boat_pos_msg = Pose2D(x=-71.3115, y=42.2846)
        goal_pos_msg = Pose2D(x=-71.312, y=42.2923)
        wind_msg = Pose2D(x=5, theta=180)

        # Feed the messages to the planner
        planner.received_grid_map_msg(grid_map)
        planner.received_boat_pos_msg(boat_pos_msg)
        planner.received_desired_pos_msg(goal_pos_msg)
        planner.received_wind_msg(wind_msg)

        # Determine the path
        waypoints = planner.plan_path()
        pass


if __name__ == '__main__':
    # Run the tests
    unittest.main()
