

class WayGenerator:


    def make_waypoints(self, path):

        change_x, change_y = find_differnce(path, 1)
        waypoints = []

        for i in range(1, len(path) - 1):

            dx, dy = find_differnce(path, i)

            if dx != change_x or dy != change_y:
                waypoints.append(path[i])
                change_x = dx
                change_y = dy

        waypoints.append(path[-1])

        return waypoints

    def find_differernce(self, path, x):
        x_curr, y_curr = path[x]
        x_next, y_next = path[x+1]

        change_x = x_next - x_curr
        change_y = y_next - y_curr

        return change_x, change_y
