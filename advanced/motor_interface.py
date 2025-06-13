import socket, json, math, logging
from typing import Sequence
from states import DriveModuleDesiredValues


class MotorClient:
    POS_SCALE = 27.777777778 * 105.26   # deg→encoder
    VEL_SCALE = 100                     # m/s→raw

    def __init__(self, host='127.0.0.1', port=9000):
        self.log = logging.getLogger('MotorClient')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        self.log.info(f'Connected to MotorServer {host}:{port}')

    def _send(self, d):  # satır sonu şart
        self.sock.sendall((json.dumps(d) + '\n').encode())

    def apply(self, states: Sequence[DriveModuleDesiredValues]):
        for i, st in enumerate(states):
            enc = int(math.degrees(st.steering_angle_in_radians) * self.POS_SCALE)
            vel = int(st.drive_velocity_in_meters_per_second * self.VEL_SCALE)
            self._send({'command': 'set_position', 'index': 2*i,   'value': enc})
            self._send({'command': 'set_velocity', 'index': 2*i+1, 'value': vel})

    def stop_all(self):
        self._send({'command': 'stop_all'})
