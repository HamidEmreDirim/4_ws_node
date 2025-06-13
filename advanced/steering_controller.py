import time, math
from typing import List
from profile import LinearProfile
from control_model import Simple4WS
from motor_interface import MotorClient
from states import (BodyMotion, DriveModuleMeasuredValues,
                    DriveModuleDesiredValues)
from drive_module import DriveModule
from control import BodyMotionCommand


class FourWSController:
    def __init__(self, modules: List[DriveModule], log_cb,
                 host='127.0.0.1', port=9000):
        self.log = log_cb
        self.mods = modules
        self.model = Simple4WS(modules)
        self.motor = MotorClient(host, port)

        self.states = [DriveModuleMeasuredValues(m.name, m.axis_pos.x,
                                                 m.axis_pos.y, 0, 0, 0)
                       for m in modules]
        self.body_vx_prof = self.body_vy_prof = self.body_wz_prof = None
        self.T = 1.0
        self.t0 = time.time()

    # ---------------- external API -----------------------
    def set_command(self, cmd: BodyMotionCommand):
        bm = cmd.to_body()
        vx0 = self.model.body_from_wheels(self.states).linear_velocity.x
        vy0 = self.model.body_from_wheels(self.states).linear_velocity.y
        wz0 = self.model.body_from_wheels(self.states).angular_velocity.z

        self.body_vx_prof = LinearProfile(vx0, bm.linear_velocity.x)
        self.body_vy_prof = LinearProfile(vy0, bm.linear_velocity.y)
        self.body_wz_prof = LinearProfile(wz0, bm.angular_velocity.z)
        self.T  = cmd.duration()
        self.t0 = time.time()
        self.log(f'Cmd set: vx {bm.linear_velocity.x:.2f} vy {bm.linear_velocity.y:.2f} wz {bm.angular_velocity.z:.2f}')

    def update_measurements(self, new_states: List[DriveModuleMeasuredValues]):
        self.states = new_states

    def periodic(self):
        if self.body_vx_prof is None:
            return
        t_rel = (time.time() - self.t0) / self.T
        if t_rel >= 1.0:
            self.body_vx_prof = self.body_vy_prof = self.body_wz_prof = None
            return

        bm = BodyMotion(self.body_vx_prof.value_at(t_rel),
                        self.body_vy_prof.value_at(t_rel),
                        self.body_wz_prof.value_at(t_rel))
        pairs = self.model.wheels_from_body(bm)

        desired: List[DriveModuleDesiredValues] = []
        for i, (opt1, opt2) in enumerate(pairs):
            curr = self.states[i].steer_angle
            desired.append(opt1 if abs(opt1.steering_angle_in_radians - curr) <
                                   abs(opt2.steering_angle_in_radians - curr) else opt2)

        self.motor.apply(desired)
