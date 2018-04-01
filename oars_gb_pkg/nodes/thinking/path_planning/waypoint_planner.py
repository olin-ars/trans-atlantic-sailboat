#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from oars_gb.msg import GridMap
from PIL import Image as IMG
from oars_gb_pkg.helpers.waypoint_generator import *

class PathPlanner:

    def __init__(self):
        # self.image = IMG.open('map_grid.png')
        # self.image.load()
        # self.grid_map = [self.image, 42.282368, 42.293353, -71.314756, -71.302289]
        rospy.init_node('path_planner', anonymous=True)
        self.grid_map = rospy.Subscriber('/planning/map', GridMap, self.get_waypoints, queue_size = 1)
        self.waypoint_pub = rospy.Publisher('/planning/waypoints', Pose2D, queue_size=1)
        print('initialized')
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            r.sleep()

    def get_waypoints(self, msg):
        # Make grid of land/water cells based on image
        grid = Grid(msg.grid)
        lowleft = (msg.minLongitude, msg.minLatitude)
        topright = (msg.maxLongitude, msg.maxLatitude)
        g = AStarPlanner(grid)
        # Find an open starting and ending coordinate
        start = (0, 0)
        end = (grid.width - 1, grid.height - 1)
        while not grid.get_cell(start).is_water:
            start = (start[0] + 1, start[1] + 1)
        while not grid.get_cell(end).is_water:
            end = (end[0] - 1, end[1] - 1)
        # Run the path planner and save the path in an image
        path = g.run(start, end)
        waypoints = make_waypoints(path)
        gps_waypoints = []
        for point in waypoints:
            gps_point = cell_to_gps_coords(point, lowleft, topright, grid.width, grid.height)
            gps_waypoints.append(Pose2D(gps_point))

        # print(gps_waypoints)
        self.waypoint_pub.publish(gps_waypoints)

class Grid():
    def __init__(self, image):
        """ Creates a grid of cell objects based on an image. Each image pixel becomes
        a cell that is water if it's light colored, land if it's dark colored."""
        self.width = image.width
        self.height = image.height

        # Creates empty grid
        self.grid = [[Cell(coords = (x, y)) for x in range(self.width)] for y in range(self.height)]
        water = 0
        print(image.data)
        for y in range(self.height):
            for x in range(self.width):
                if image.data[0] > 128: # This does not work
                    self.grid[y][x].is_water = True

    def get_cell(self, coords):
        """ Returns the cell object at the given coordinate. """
        return self.grid[coords[1]][coords[0]]

    def _add_coords(self, a, b):
        """ Returns a third coord that is equivalent to
            (a[0]+b[0], a[1]+b[1]) """
        return tuple(map(sum, zip(a, b)))

    def _is_in_grid(self, cell_coord):
        """ tells us whether cell_coord is in range of the actual
            grid dimensions """
        valid_x = (-1 < cell_coord[0] < self.width)
        valid_y = (-1 < cell_coord[1] < self.height)
        return valid_x and valid_y

class Cell():
    def __init__(self, coords, is_water = True): #change back to false when working
        self.is_water = is_water
        self.coords = coords
        self.g_cost = None
        self.h_cost = None
        self.parents_coords = None

    @property
    def f_cost(self):
        if self.g_cost is None or self.h_cost is None:
            return None
        return self.g_cost + self.h_cost

class AStarPlanner():
    def __init__(self, grid):
        self.grid = grid
        self.open_list = []
        self.closed_list = []

    def get_h_cost(self, coord_a, coord_b):
        """ returns the h score, the manhattan distance between coord_a and
            the coord_b. """
        return abs(coord_a[0] - coord_b[0]) + abs(coord_a[1] - coord_b[1])

    def get_open_adj_coords(self, coords):
        """ returns list of valid coords that are adjacent to the argument,
            open, and not in the closed list. """
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, -1), (-1, 1)]
        all_adj = [self.grid._add_coords(coords, d) for d in directions]
        all_costs = [2, 3, 2, 10, 1, 1, 1, 1]
        in_bounds = [self.is_valid(c) for c in all_adj]
        costs = []
        open_adj = []
        for i, coord in enumerate(all_adj):
            if(in_bounds[i]):
                costs.append(all_costs[i])
                open_adj.append(coord)
        return open_adj, costs

    def is_valid(self, coord):
        return self.grid._is_in_grid(coord) \
            and self.grid.get_cell(coord).is_water \
            and coord not in self.closed_list

    def get_lowest_cost_open_coord(self):
        """ Returns the open coordinate with the lowest f cost. """
        sorted_cells = sorted(self.open_list, key=lambda cell: self.grid.get_cell(cell).f_cost)
        return sorted_cells[0]

    def get_path(self, start_coord, destination_coord):
        """ Follows cell parents backwards until the initial cell is reached to
            create a path, which is the list of coordinates that the boat will
            travel through to reach the destination. """
        coord_list = [destination_coord]
        print("final cost is {}".format(self.grid.get_cell(coord_list[-1]).f_cost))
        path = []
        while coord_list[-1] is not None:
            try:
                parent = self.grid.get_cell(coord_list[-1]).parents_coords
                coord_list.append(parent)
            except:
                print('No path found to destination coord!')
                break
        for coord in coord_list:
            if coord is not None:
                path.append(coord)
        path.reverse()
        return path

    def run(self, start_coord, destination_coord):
        """ Updates cells' g, h, f, and parent coordinates until the destination
            square is found. """
        self.open_list = [start_coord]
        self.closed_list = []
        self.destination_coord = destination_coord
        cell_s = self.grid.get_cell(start_coord)
        cell_s.g_cost = 0
        cell_s.h_cost = self.get_h_cost(start_coord, destination_coord)
        self.open_list = [start_coord]
        while len(self.open_list) > 0:
            start_coord = self.get_lowest_cost_open_coord()
            cell_s = self.grid.get_cell(start_coord)
            self.open_list.remove(start_coord)
            self.closed_list.append(start_coord)
            walkable_open_coords, costs = self.get_open_adj_coords(start_coord)
            for idx, coord in enumerate(walkable_open_coords):
                cell = self.grid.get_cell(coord)
                g_cost = cell_s.g_cost + costs[idx]
                h_cost = self.get_h_cost(coord, destination_coord)
                f_cost = g_cost + h_cost
                if coord in self.open_list:
                    old_f_cost = cell.f_cost
                    if f_cost < old_f_cost:
                        cell.g_cost = g_cost
                        cell.h_cost = h_cost
                        cell.parents_coords = start_coord
                else:
                    self.open_list.append(coord)
                    cell.g_cost = g_cost
                    cell.h_cost = h_cost
                    cell.parents_coords = start_coord

        return self.get_path(start_coord, destination_coord)

if __name__ == "__main__":
    PathPlanner()
