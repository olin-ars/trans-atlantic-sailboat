def make_waypoints(path):
    """
    Finds all of the waypoints, the points at which the path switches
    directions, by comparing the change in x and y directions from the
    previous position step
    :param path: lists of coordinates
    :return a list of the way waypoints

    Examples:
    >>> make_waypoints([(0,0),(1,1), (2,2), (1,3)])
    [(2, 2), (1, 3)]
    >>> make_waypoints([(0,0), (1,1), (2,2), (3,3)])
    [(3, 3)]
    """

    change_x, change_y = _find_difference(path, 0)
    waypoints = []

    for i in range(1, len(path) - 1):

        dx, dy = _find_difference(path, i)

        if dx != change_x or dy != change_y:
            waypoints.append(path[i])
            change_x = dx
            change_y = dy

    waypoints.append(path[-1])

    return waypoints


def _find_difference(path, x):
    """
    Calculates the difference in the way direction and the difference
    in the x direction between two different steps in the path
    :param path: list of coordinates
    :param x: the current coordinate position

    :return change in x and change in y between two consecutive cordinates

    Examples:
    >>> _find_difference([(0,0), (1,1)], 0)
    (1, 1)
    >>> _find_difference([(2,2), (1,3)], 0)
    (-1, 1)
    """

    x_curr, y_curr = path[x]

    x_next, y_next = path[x+1]

    change_x = x_next - x_curr
    change_y = y_next - y_curr

    return change_x, change_y


def cell_to_gps_coords(coord, p1, p2, cols, rows):
    """
    Converts cell coordinates to GPS coordinates

    :param coord: tuple of cell coordinate
    :param p1: bottom left coordinate position
    :param p2: top right coordinate position
    :param rows: number of rows
    :param cols: number of columns

    :return tuple of GPS coordinates

    Examples:
    >>> cell_to_gps_coords((1, 2), (10, 20), (13, 23), 3, 3)
    (11.5, 22.5)
    >>> cell_to_gps_coords((1, 2), (10.5, 20.5), (13.5, 23.5), 3, 3)
    (12.0, 23.0)
    >>> cell_to_gps_coords((1, 3), (10, 70), (40, 134), 3, 4)
    (25.0, 126.0)
    >>> cell_to_gps_coords((3, 2), (-2, 3), (3, 7), 5, 4)
    (1.5, 5.5)
    >>> cell_to_gps_coords((2, 5), (-2, -4), (4, 3), 5, 8)
    (0.5, 1.5)
    """

    p1_x, p1_y = p1
    p2_x, p2_y = p2

    dx = p2_x.data - p1_x.data
    dy = p2_y.data - p1_y.data

    row_width = dy / rows
    col_width = dx / cols

    x_val, y_val = coord

    dx_gps_pos = (col_width * x_val) + (0.5 * col_width)
    dy_gps_pos = (row_width * y_val) + (0.5 * row_width)

    x_gps_coord = p1_x.data + dx_gps_pos
    y_gps_coord = p1_y.data + dy_gps_pos

    return x_gps_coord, y_gps_coord


def gps_coords_to_cell(coord, p1, p2, cols, rows):
    """
    Converts cell coordinates to GPS coordinates

    :param coord: tuple of cell coordinate
    :param p1: bottom left coordinate position
    :param p2: top right coordinate position
    :param rows: number of rows
    :param cols: number of columns

    :return tuple of GPS coordinates
    """

    p1_x, p1_y = p1
    p2_x, p2_y = p2

    dx = p2_x.data - p1_x.data
    dy = p2_y.data - p1_y.data

    f = rows / dy
    g = cols / dx

    x_pos, y_pos = coord

    x_pos -= p1_x.data
    y_pos -= p1_y.data

    cell_x = int(g * x_pos)
    cell_y = int(f * y_pos)

    return cell_x, cell_y


if __name__ == "__main__":
    import doctest
    doctest.testmod()
