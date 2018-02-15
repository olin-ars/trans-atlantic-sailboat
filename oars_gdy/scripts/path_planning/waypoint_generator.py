

class WaypointGenerator:


    def make_waypoints(self, path):
        """
        Finds all of the waypoints, the points at which the path switches
        directions, by comparing the change in x and y directions from the
        previous position step

        path: lists of cordiantes

        returns: a list of the way waypoints

        Examples:
            >>> WaypointGenerator().make_waypoints([(0,0),(1,1), (2,2), (1,3)])
            [(2, 2), (1, 3)]
            >>> WaypointGenerator().make_waypoints([(0,0), (1,1), (2,2), (3,3)])
            [(3, 3)]
            >>> WaypointGenerator().make_waypoints([(0,0), (1,0), (2,0), (3,0)])
            [(3, 0)]
            >>> WaypointGenerator().make_waypoints([(1,2), (2,2), (2,1), (3,0)])
            [(2, 2), (2, 1), (3, 0)]
            >>> WaypointGenerator(). make_waypoints([(1,1), (2,1), (2,2), (1,2), (0,2), (0,1), (0,0), (1,0), (2,0), (3,0), (3,1), (3,2), (3,3), (2,3), (1,3), (0,3)])
            [(2, 1), (2, 2), (0, 2), (0, 0), (3, 0), (3, 3), (0, 3)]

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
        Calculates the differnce in the way driection and the difference
        in the x direction between two different steps in the path

        path: list of cordiantes
        x: the current cordinate position

        returns: change in x and change in y between two consecutive cordinates

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


if __name__ == "__main__":
    import doctest
    #doctest.testmod()
    doctest.run_docstring_examples(WaypointGenerator().make_waypoints, globals(), verbose=True)
