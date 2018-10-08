#!/usr/bin/env python
from os import path
from PIL import Image as PILImage
import time
try:
    import rospy
    from std_msgs.msg import Float32, Header
    from sensor_msgs.msg import Image
    from oars_gb.msg import GridMap
except ImportError:
    rospy = None
    from tests.mock_ros_msgs import Header, Float32, GridMap


class GridMapGenerator:
    def __init__(self, using_ros=True):
        self.grid = None
        self.minLatitude = None
        self.maxLatitude = None
        self.minLongitude = None
        self.maxLongitude = None
        self.using_ros = rospy and using_ros

        if self.using_ros:
            rospy.init_node('grid_generator')
            self.grid_pub = rospy.Publisher('/planning/map', GridMap, queue_size=1)
            # self.img_pub = rospy.Publisher('/planning/image', Image, queue_size=1)
            print('Grid generator node started')
        else:
            print('Running GridMapGenerator in unit test mode')

    def load_image(self, file_path):
        map_image = PILImage.open(file_path)
        map_image.load()

        # Resolve the boundary coordinates
        (directory, filename) = path.split(file_path)
        name_parts = filename.split('_')
        self.minLatitude = float(name_parts[1])
        self.minLongitude = float(name_parts[2])
        self.maxLatitude = float(name_parts[3])
        self.maxLongitude = float('.'.join(name_parts[4].split('.')[:-1]))  # Remove the file extension

        self.grid = Grid(map_image)

    def publish_map(self):
        map_image = self.grid.draw_map()
        grid_msg = GridMap(grid=map_image, minLatitude=Float32(self.minLatitude), maxLatitude=Float32(self.maxLatitude),
                           minLongitude=Float32(self.minLongitude), maxLongitude=Float32(self.maxLongitude))
        if self.using_ros:
            self.grid_pub.publish(grid_msg)
            # self.img_pub.publish(map_image)
        return grid_msg


class Grid():
    def __init__(self, image):
        """ Creates a grid of cell objects based on an image. Each image pixel becomes
            a cell that is water if it's light colored, land if it's dark colored."""
        self.width = 90
        self.height = self.width * image.size[1] // image.size[0]
        image.thumbnail((self.width, self.height))
        # Creates empty grid
        self.grid = [[Cell(coords=(x, y)) for x in range(self.width)] for y in range(self.height)]
        for y in range(self.height):
            for x in range(self.width):
                pixel = image.getpixel((x, y))
                if pixel[2] > 210 and pixel[2] > pixel[1] + 20:
                    self.grid[y][x].is_water = True
                else:
                    self.grid[y][x].is_water = False

    def get_cell(self, coords):
        """ Returns the cell object at the given coordinate. """
        return self.grid[coords[1]][coords[0]]

    def safe_distance(self, coords):
        """ Returns False if the given cell is land or is adjacent to land,
            True if it is water surrounded by water. Useful for creating a buffer. """
        if not self.get_cell(coords).is_water:
            return False
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, -1), (-1, 1)]
        all_adj = [self._add_coords(coords, d) for d in directions]
        for coord in all_adj:
            if self._is_in_grid(coord):
                if not self.get_cell(coord).is_water:
                    return False
        return True

    def _is_in_grid(self, cell_coord):
        """ tells us whether cell_coord is in range of the actual
            grid dimensions """
        valid_x = (-1 < cell_coord[0] < self.width)
        valid_y = (-1 < cell_coord[1] < self.height)
        return valid_x and valid_y

    def add_buffer(self):
        """ Blocks off cells adjacent to non-water cells. Returns nothing, replaces
            the grid with a grid with more blocked-off cells. """
        buffered_grid = [[Cell(coords=(x, y), is_water=self.safe_distance((x, y)))
                          for x in range(self.width)] for y in range(self.height)]
        self.grid = buffered_grid

    def draw_map(self):
        """ Creates an image with the map image as a background and the cells in
            the final path highlighted in green. """
        header = Header()
        height = self.height
        width = self.width
        encoding = 'rgb8'
        step = 3 * width
        data = []
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y][x].is_water:
                    data.extend([254] * 3)
                else:
                    data.extend([0] * 3)

        img = Image(header=header, height=height, width=width, encoding=encoding,
                    is_bigendian=False, step=step, data=data)
        return img


class Cell():
    def __init__(self, coords, lat=0, lon=0, is_water=False):
        self.is_water = is_water
        self.coords = coords


if __name__ == "__main__":
    map_generator = GridMapGenerator()
    # Load an image to base the map on
    map_generator.load_image('maps/waban_42.282818_-71.313751_42.293184_-71.302057.png')

    while not rospy.is_shutdown():
        map_generator.publish_map()
        print("Published GridMap")
        time.sleep(10)
