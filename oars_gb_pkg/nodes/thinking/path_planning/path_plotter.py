from PIL import Image
from datetime import datetime

class GridMap():
    def __init__(self, image = None):
        """ Creates a grid of cell objects. If an image is provided, each pixel
            becomes a cell that is water if it's light colored, land if it's dark colored.
            If no image is provided, a 10x10 all-water grid is made."""
        if image:
            self.width = image.size[0]
            self.height = image.size[1]
        else:
            self.width = 10
            self.height = 10
        # Creates empty grid
        self.grid = [[Cell(coords = (x, y)) for x in range(self.width)] for y in range(self.height)]

        # If an image is provided, adds land where there are dark pixels
        if image:
            for y in range(self.height):
                for x in range(self.width):
                    if map_gs.getpixel((x, y)) < 50:
                        self.grid[y][x].is_water = False

    def draw_path(self, path, map_image, saveas):
        """ Creates an image with the map image as a background and the cells in
            the final path highlighted in green. """
        img = Image.new("RGB", (self.width, self.height))
        pixels = img.load()
        for y in range(self.height):
            for x in range(self.width):
                if (x, y) in path:
                    pixels[x, y] = (0, 255, 0)
                else:
                    pixels[x, y] = map_image.getpixel((x, y))

        img.save(saveas)

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

    def add_buffer(self):
        """ Blocks off cells adjacent to non-water cells. Returns nothing, replaces
            the grid with a grid with more blocked-off cells. """
        buffered_grid = [[Cell(coords = (x, y), is_water = self.safe_distance((x, y))) \
            for x in range(self.width)] for y in range(self.height)]
        self.grid = buffered_grid

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
    def __init__(self, coords, is_water = True):
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
    # Load an image to base the map on
    filename = input('Filename: ')
    map_image = Image.open(filename)
    map_image.load()
    # Convert image to grayscale
    map_gs = map_image.convert('L')
    # Make grid of land/water cells based on image
    grid = GridMap(image = map_gs)
    g = AStarPlanner(grid)
    # grid.add_buffer()
    # Find an open starting and ending coordinate
    start = (0, 0)
    end = (grid.width - 1, grid.height - 1)
    while not grid.get_cell(start).is_water:
        start = (start[0] + 1, start[1] + 1)
    while not grid.get_cell(end).is_water:
        end = (end[0] - 1, end[1] - 1)
    # Run the path planner and save the path in an image
    path = g.run(start, end)
    saveas = 'paths/' + datetime.now().__format__('%j%H%M') + '.png'
    grid.draw_path(path, map_image, saveas)
