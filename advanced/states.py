import math
from typing import Tuple
from geometry import Vector3, Orientation, Point


class BodyMotion:
    def __init__(self, vx, vy, wz):
        self.linear_velocity = Vector3(vx, vy, 0.0)
        self.angular_velocity = Vector3(0.0, 0.0, wz)


class BodyState:
    def __init__(self):
        self.position = Point(0, 0, 0)
        self.orientation = Orientation()
        self.motion = BodyMotion(0, 0, 0)


class DriveModuleDesiredValues:
    def __init__(self, name, steer, vel):
        self.name = name
        self.steering_angle_in_radians = steer
        self.drive_velocity_in_meters_per_second = vel


class DriveModuleMeasuredValues:
    def __init__(self, name, x, y, steer, steer_vel, drive_v):
        self.name = name
        self.pos_xy = Point(x, y, 0)
        self.steer_angle = steer
        self.steer_vel = steer_vel
        self.drive_v = drive_v

    def xy_drive_velocity(self) -> Tuple[float, float]:
        v_x = self.drive_v * math.cos(self.steer_angle)
        v_y = self.drive_v * math.sin(self.steer_angle)
        return v_x, v_y
