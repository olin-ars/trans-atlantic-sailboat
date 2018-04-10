from math import sin, cos, atan2, radians, sqrt


def dist_between_points(p1, p2, earth_radius=6371e3):
    """
    Calculates the distance between two coordinate points using the Haversine formula.
    Source: https://www.movable-type.co.uk/scripts/latlong.html
    :param p1: the first (long, lat) coordinate point
    :type p1: tuple
    :param p2: the second (long, lat) coordinate point
    :type p2: tuple
    :param earth_radius: the Earth's radius (in meters) at the given surface region
    :type earth_radius: float
    :return: the distance, in meters, between the two points
     on the surface of the earth
    :rtype float
    """
    (lon1, lat1) = p1
    (lon2, lat2) = p2

    # Convert everything from degrees to radians
    lat1 = radians(lat1)
    lat2 = radians(lat2)
    delta_lat = radians(lat2 - lat1)
    delta_lon = radians(lon2 - lon1)

    a = sin(delta_lat / 2) * sin(delta_lat / 2) + cos(lat1) * cos(lat2) * sin(delta_lon / 2) * sin(delta_lon / 2)

    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    return earth_radius * c
