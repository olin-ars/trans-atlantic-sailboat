

class WaypointGenerator:


    def make_waypoints(self, path):
        """
        Finds all of the waypoints, the points at which the path switches
        directions, by comparing the change in x and y directions from the
        previous position step
        :param path: lists of cordiantes
        :returns: a list of the way waypoints

        Examples:
        >>> WaypointGenerator().make_waypoints([(0,0),(1,1), (2,2), (1,3)])
        [(2, 2), (1, 3)]
        >>> WaypointGenerator().make_waypoints([(0,0), (1,1), (2,2), (3,3)])
        [(3, 3)]
        """

        change_x, change_y = self.find_difference(path, 0)
        waypoints = []

        for i in range(1, len(path) - 1):

            dx, dy = self.find_difference(path, i)

            if dx != change_x or dy != change_y:
                waypoints.append(path[i])
                change_x = dx
                change_y = dy

        waypoints.append(path[-1])

        return waypoints

    def find_difference(self, path, x):
        """
        Calculates the difference in the way driection and the difference
        in the x direction between two different steps in the path
        :param path: list of cordiantes
        :param x: the current cordinate position

        :returns: change in x and change in y between two consecutive cordinates

        Examples:
        >>> WaypointGenerator().find_difference([(0,0), (1,1)], 0)
        (1, 1)
        >>> WaypointGenerator().find_difference([(2,2), (1,3)], 0)
        (-1, 1)
        """

        x_curr, y_curr = path[x]

        x_next, y_next = path[x+1]

        change_x = x_next - x_curr
        change_y = y_next - y_curr

        return change_x, change_y

    def cell_to_GPS_coords(self, coord, p1, p2, cols, rows):
        """
        Converts cell coordinates to GPS cordinates

        coord: tuple of cell coordinate
        p1: bottom left coordinate position
        p2: top right coordinate position
        rows: number of rows
        cols: number of columns

        returns: tuple of GPS cordinate

        Examples:
        >>> WaypointGenerator().cell_to_GPS_coords((1, 2), (10, 20), (13, 23), 3, 3)
        (11.5, 22.5)
        >>> WaypointGenerator().cell_to_GPS_coords((1, 2), (10.5, 20.5), (13.5, 23.5), 3, 3)
        (12.0, 23.0)
        >>> WaypointGenerator().cell_to_GPS_coords((1, 3), (10, 70), (40, 134), 3, 4)
        (25.0, 126.0)
        >>> WaypointGenerator().cell_to_GPS_coords((3, 2), (-2, 3), (3, 7), 5, 4)
        (1.5, 5.5)
        >>> WaypointGenerator().cell_to_GPS_coords((2, 5), (-2, -4), (4, 3), 5, 8)
        (0.5, 1.5)
        """

        p1_x, p1_y = p1
        p2_x, p2_y = p2
        # print("p1:", p1)
        # print("p2:", p2)

        dx = p2_x - p1_x
        dy = p2_y - p1_y
        # print("dx:", dx)
        # print("dy:", dy)

        row_width = dy / rows
        col_width = dx / cols
        # print(row_width, col_width)

        x_val, y_val = coord
        # print("coord:", coord)

        dx_GPS_pos = (col_width * x_val) + (0.5 * col_width)
        dy_GPS_pos = (row_width * y_val) + (0.5 * row_width)
        # print("change in x:", dx_GPS_pos)
        # print("change in y:", dy_GPS_pos)

        x_GPS_coord = p1_x + dx_GPS_pos
        y_GPS_coord = p1_y + dy_GPS_pos
        # print("x_GPS:", x_GPS_coord)
        # print("y_GPS:", y_GPS_coord)

        return x_GPS_coord, y_GPS_coord


if __name__ == "__main__":
    import doctest
    #doctest.testmod()
    doctest.run_docstring_examples(WaypointGenerator().cell_to_GPS_coords, globals(), verbose=True)
