class Pose2D:

    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta


class Float32:

    def __init__(self, data=float(0)):
        self.data = data


class UInt32:

    def __init__(self, data=0):
        self.data = data


class Float32MultiArray:

    def __init__(self, data=None):
        self.data = data if data is not None else []


class WaypointList:

    def __init__(self, latitudes=None, longitudes=None):
        self.latitudes = latitudes if latitudes is not None else []
        self.longitudes = longitudes if longitudes is not None else []


class Header:

    def __init__(self):
        pass


class Image:

    def __init__(self, header=None, height=0, width=0, encoding=None, is_bigendian=False, step=0, data=None):
        self.header = header if header is not None else Header()
        self.height = height
        self.width = width
        self.encoding = encoding
        self.is_bigendian = is_bigendian
        self.step = step
        self.data = data if data is not None else Float32MultiArray()


class GridMap:

    def __init__(self, grid=None, minLatitude=0, maxLatitude=0, minLongitude=0, maxLongitude=0):
        self.grid = grid if grid is not None else Image()
        self.minLatitude = minLatitude
        self.maxLatitude = maxLatitude
        self.minLongitude = minLongitude
        self.maxLongitude = maxLongitude
