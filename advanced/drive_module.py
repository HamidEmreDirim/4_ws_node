from geometry import Point

class DriveModule:
    def __init__(self, name, steer_joint, drive_joint,
                 axis_pos: Point, wheel_r, wheel_w, v_max):
        self.name = name
        self.steer_joint = steer_joint
        self.drive_joint = drive_joint
        self.axis_pos = axis_pos
        self.wheel_radius = wheel_r
        self.wheel_width = wheel_w
        self.drive_motor_max = v_max
