import pygame
# http://www.raywenderlich.com/4946/introduction-to-a-pathfinding


class GridMap():
    """ Grid world that contains Pauls (and other things) living in cells. """
    def __init__(self, width=10, height=10):
        self.grid = [[Cell() for x in range(width)] for y in range(height)]
        self.width = width
        self.height = height

    def _add_coords(self, a, b):
        """ Returns a third coord that is equivalent to
            (a[0]+b[0], a[1]+b[1]) """
        return tuple(map(sum, zip(a, b)))

    def _is_in_grid(self, cell_coord):
        """ tells us whether cell_coord is valid and in range of the actual
            grid dimensions """
        valid_x = (-1 < cell_coord[0] < self.width)
        valid_y = (-1 < cell_coord[1] < self.height)
        return valid_x and valid_y

class Cell():
    def __init__(self, is_water = True):
        self.is_water = is_water
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
        # modify directions and costs as needed
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, -1), (-1, 1), (2, 0), (0, 2), (-2, 0), (0, -2)]
        all_adj = [self.world._add_coords(coords, d) for d in directions]
        all_costs = [1, 1, 1, 1, 3, 3, 3, 3, 8, 8, 8, 8]
        in_bounds = [self.is_valid(c) for c in all_adj]
        costs = []
        open_adj = []
        for i, coord in enumerate(all_adj):
            if(in_bounds[i]):
                costs.append(all_costs[i] + self.world.get_terrain_cost(coord))
                open_adj.append(coord)
        return open_adj, costs

    def is_valid(self, coord):
        return self.world._is_in_grid(coord) \
            and coord not in self.closed_list

    def get_lowest_cost_open_coord(self):
        open_cells = self.open_list
        sorted_cells = sorted(open_cells, key=lambda s: self.cells[s].f_cost)
        return sorted_cells[0]

    def get_path(self):
        """ Follows cell parents backwards until the initial cell is reached to
            create a path, which is the list of coordinates that paul will
            travel through to reach the destination. """
        coord_list = [self.destination_coord]
        print("final cost is {}".format(self.cells[coord_list[-1]].f_cost))
        while self.start_coord not in coord_list:
            try:
                coord_list.append(self.cells[coord_list[-1]].parents_coords)
            except:
                print('No path found to destination coord!')
                break
        for coord in coord_list:
            if coord is not None:
                self.cells[coord].color = (0, 255, 0)
        return coord_list

    def run(self, start_coord, destination_coord):
        """ Updates cells g,h,f, and parent coordinates until the destination
            square is found. """
        self.open_list = [start_coord]
        self.closed_list = []
        self.destination_coord = destination_coord
        cell_s = self.cells[start_coord]
        cell_s.g_cost = 0
        cell_s.h_cost = self.get_h_cost(start_coord, destination_coord)
        self.open_list = [start_coord]
        while len(self.open_list) > 0:
            start_coord = self.get_lowest_cost_open_coord()
            cell_s = self.cells[start_coord]
            self.open_list.remove(start_coord)
            self.closed_list.append(start_coord)
            walkable_open_coords, costs = self.get_open_adj_coords(start_coord)
            for idx, coord in enumerate(walkable_open_coords):
                cell = self.cells[coord]
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


if __name__ == "__main__":
    g = AStarPlanner()
    g.run()
