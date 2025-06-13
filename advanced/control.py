from abc import ABC, abstractmethod
from states import BodyMotion

class MotionCommand(ABC):
    @abstractmethod
    def duration(self): ...
    @abstractmethod
    def to_body(self) -> BodyMotion: ...

class BodyMotionCommand(MotionCommand):
    def __init__(self, dt, vx, vy, wz):
        self._dt = dt
        self._bm = BodyMotion(vx, vy, wz)

    def duration(self): return self._dt
    def to_body(self):  return self._bm
