class Point:
    def __init__(self, x, y, z=0.0):
        self.x, self.y, self.z = x, y, z


class Orientation:
    def __init__(self, roll=0.0, pitch=0.0, yaw=0.0):
        self.x, self.y, self.z = roll, pitch, yaw


class Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z
