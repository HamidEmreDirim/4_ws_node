import math, numpy as np
from numpy.linalg import pinv
from typing import List, Tuple
from drive_module import DriveModule
from states import BodyMotion, DriveModuleDesiredValues, DriveModuleMeasuredValues


def _norm(a):  # [-π,π]
    return (a + math.pi) % (2*math.pi) - math.pi


class Simple4WS:
    def __init__(self, modules: List[DriveModule]):
        self.mods = modules
        A = []
        for m in modules:
            A += [[1, 0, -m.axis_pos.y],
                  [0, 1,  m.axis_pos.x]]
        self.M_inv = np.array(A)
        self.M_fwd = pinv(self.M_inv)

    # wheels → body
    def body_from_wheels(self, states: List[DriveModuleMeasuredValues]):
        vec = []
        for st in states:
            vec += list(st.xy_drive_velocity())
        V_i = np.array(vec)
        v = self.M_fwd @ V_i
        return BodyMotion(v[0], v[1], v[2])

    # body → wheels
    def wheels_from_body(self, motion: BodyMotion
                         ) -> List[Tuple[DriveModuleDesiredValues,
                                         DriveModuleDesiredValues]]:
        V = np.array([motion.linear_velocity.x,
                      motion.linear_velocity.y,
                      motion.angular_velocity.z])
        V_i = self.M_inv @ V
        out = []
        for i, m in enumerate(self.mods):
            vx, vy = V_i[2*i], V_i[2*i+1]
            spd = math.hypot(vx, vy)
            ang = _norm(math.atan2(vy, vx))
            fwd = DriveModuleDesiredValues(m.name, ang,  spd)
            rev = DriveModuleDesiredValues(m.name, _norm(ang+math.pi), -spd)
            out.append((fwd, rev))
        return out
