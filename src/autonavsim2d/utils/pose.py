
class Pose:

    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation


class Point:

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Orientation:

    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w