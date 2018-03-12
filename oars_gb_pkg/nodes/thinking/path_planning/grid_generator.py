#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Header
from oars_gb.msg import GridMap
from PIL import Image as IMG
import time

class GridGenerator:
    def __init__(self):
        rospy.init_node('grid_generator', anonymous=True)
        self.grid_pub = rospy.Publisher('/planning/map', GridMap, queue_size=0)
        self.image_pub = rospy.Publisher('/planning/image', Image, queue_size=0)

    def publish_map(self, map_image, minLat, maxLat, minLong, maxLong):
        # msg = GridMap(grid = map_image, minLatitude = minLat, maxLatitude = maxLat, \
        #         minLongitude = minLong, maxLongitude = maxLong)
        grid_msg = GridMap(grid=map_image, minLatitude=Float32(minLat))
        self.grid_pub.publish(grid_msg)
        self.image_pub.publish(map_image)

class Grid():
    def __init__(self, image):
        """ Creates a grid of cell objects based on an image. Each image pixel becomes
            a cell that is water if it's light colored, land if it's dark colored."""
        self.width = 180
        self.height = self.width * image.size[1] // image.size[0]
        image.thumbnail((self.width, self.height))
        # Creates empty grid
        self.grid = [[Cell(coords = (x, y)) for x in range(self.width)] for y in range(self.height)]

        for y in range(self.height):
            for x in range(self.width):
                pixel = image.getpixel((x, y))
                if pixel[2] > 210 and pixel[2] > pixel[1]+20:
                    self.grid[y][x].is_water = True

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
        buffered_grid = [[Cell(coords = (x, y), is_water = self.safe_distance((x, y))) \
            for x in range(self.width)] for y in range(self.height)]
        self.grid = buffered_grid

    def draw_map(self):
        """ Creates an image with the map image as a background and the cells in
            the final path highlighted in green. """
        header = Header()
        height = self.height
        width = self.width
        encoding = 'rgb8'
        is_bigendian = False
        step = 3 * width
        data = []

        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y][x].is_water:
                    data.extend([254]*3)
                else:
                    data.extend([0]*3)

        img = Image(header = header, height = height, width = width, encoding = encoding, \
        is_bigendian = is_bigendian, step = step, data = data)
        return img

class Cell():
    def __init__(self, coords, lat = 0, lon = 0, is_water = False):
        self.is_water = is_water
        self.coords = coords

if __name__ == "__main__":
    # Load an image to base the map on
    filename = 'oars_gb_pkg/nodes/thinking/path_planning/lakewaban.png' # + input('Filename: ')
    map_image = IMG.open(filename)
    map_image.load()
    # Make grid of land/water cells based on image
    grid = Grid(map_image)
    #grid.add_buffer()
    map_grid = grid.draw_map()

    g = GridGenerator()
    while True:
        g.publish_map(map_grid, 42.282368, 42.293353, -71.314756, -71.302289)
        print("published")
        time.sleep(10)
