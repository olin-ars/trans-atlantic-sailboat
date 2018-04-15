class AStarPlanner:
    def __init__(self, grid):
        self.grid = grid
        self.open_list = []
        self.closed_list = []

    def plan(self, start_coord, destination_coord, wind_angle):
        """ Updates cells' g, h, f, and parent coordinates until the destination
            square is found. """
        self.wind_direction = ((wind_angle + 22.5) % 360) // 45
        self.open_list = [start_coord]
        self.closed_list = []
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

    @staticmethod
    def get_h_cost(coord_a, coord_b):
        """ returns the h score, the manhattan distance between coord_a and
            the coord_b. """
        return abs(coord_a[0] - coord_b[0]) + abs(coord_a[1] - coord_b[1])

    def get_open_adj_coords(self, coords):
        """ returns list of valid coords that are adjacent to the argument,
            open, and not in the closed list. """
        directions = [(0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1), (1, 0), (1, 1)]
        all_adj = [self.grid.add_coords(coords, d) for d in directions]
        all_costs = [10, 1, 2, 1, 3, 1, 2, 1]
        w = self.wind_direction
        costs_shifted = all_costs[w:] + all_costs[:w]
        in_bounds = [self.is_valid(c) for c in all_adj]
        costs = []
        open_adj = []
        for i, coord in enumerate(all_adj):
            if in_bounds[i]:
                costs.append(costs_shifted[i])
                open_adj.append(coord)
        return open_adj, costs

    def is_valid(self, coord):
        return self.grid.is_in_grid(coord) \
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


class Grid:
    def __init__(self, image):
        """
        Creates a grid of cell objects based on an image. Each image pixel becomes
        a cell that is water if it's light colored, land if it's dark colored.
        :param image: the map, where each pixel values corresponds to the type of terrain at that point
        :type image: Image (ROS message)
        """
        self.width = image.width
        self.height = image.height

        # Creates empty grid
        self.grid = [[Cell(coords=(x, y)) for x in range(self.width)] for y in range(self.height)]
        for y in range(self.height):
            for x in range(self.width):
                index = (y * self.width + x) * 3  # 3 is because we have RGB data
                if image.data[index] > 128:
                    self.grid[y][x].is_water = True

    def get_cell(self, coords):
        """ Returns the cell object at the given coordinate. """
        return self.grid[coords[1]][coords[0]]

    @staticmethod
    def add_coords(a, b):
        """ Returns a third coord that is equivalent to
            (a[0]+b[0], a[1]+b[1]) """
        return tuple(map(sum, zip(a, b)))

    def is_in_grid(self, cell_coord):
        """ tells us whether cell_coord is in range of the actual
            grid dimensions """
        valid_x = (-1 < cell_coord[0] < self.width)
        valid_y = (-1 < cell_coord[1] < self.height)
        return valid_x and valid_y


class Cell:
    def __init__(self, coords, is_water=True):  # Change back to false when working
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
