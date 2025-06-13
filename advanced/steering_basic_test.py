#!/usr/bin/env python3
# 4ws_node_simple.py – 2025-06-11  “v5-CrabStep±5°”

import math, json, socket
import numpy as np
import rclpy
from rclpy.node      import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg   import Joy

# ───────────────────────────────
# Paylaşılan durum
# ───────────────────────────────
vel_msg      = Twist()  # sadece linear.x kullanıyoruz
steer_target = 0.0      # tüm tekerler için ortak açı (rad)

MAX_STEER_DEG = 360     # güvenlik sınırı (±)
STEP_DEG      = 10       # her basışta kaç derece?

# joystick düğme index’leri – gerekirse düzelt
BUTTON_X  = 0   # açıyı sıfırla
BUTTON_LB = 4   # sola çevir
BUTTON_RB = 5   # sağa çevir
# ───────────────────────────────


class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')

        self.declare_parameter('deadzone', 0.05)
        self.deadzone = self.get_parameter(
            'deadzone').get_parameter_value().double_value
        self.add_on_set_parameters_callback(self._param_cb)

        self.create_subscription(Joy, 'joy', self.cb, 10)
        self.prev_buttons = [0]*16

    def _param_cb(self, _):
        self.deadzone = self.get_parameter(
            'deadzone').get_parameter_value().double_value
        return rclpy.parameter.SetParametersResult(successful=True)

    @staticmethod
    def _dz(v, dz):             # yardımcı: dead-zone
        return 0.0 if abs(v) < dz else v

    def cb(self, joy: Joy):
        global vel_msg, steer_target
        b = joy.buttons

        # hız (ileri-geri)
        vel_msg.linear.x = self._dz(joy.axes[1], self.deadzone) * 7.5

        # LB / RB / X kenar-tetik
        if b[BUTTON_LB] and not self.prev_buttons[BUTTON_LB]:
            steer_target -= math.radians(STEP_DEG)
            self.get_logger().info(f"Krab ←   ({math.degrees(steer_target):.1f}°)")
        elif b[BUTTON_RB] and not self.prev_buttons[BUTTON_RB]:
            steer_target += math.radians(STEP_DEG)
            self.get_logger().info(f"Krab →   ({math.degrees(steer_target):.1f}°)")
        elif b[BUTTON_X] and not self.prev_buttons[BUTTON_X]:
            steer_target = 0.0
            self.get_logger().info("Direksiyon sıfırlandı")

        # ± MAX_STEER_DEG sınırı
        limit = math.radians(MAX_STEER_DEG)
        steer_target = max(-limit, min(limit, steer_target))

        self.prev_buttons = list(b)


class FourWheelSteeringNode(Node):
    def __init__(self):
        super().__init__('four_wheel_steering_node')

        self.declare_parameter('steering_gain',      0.20)
        self.declare_parameter('max_steer_rate_deg', 90.0)
        self.declare_parameter('speed_deadzone',     0.02)
        self.add_on_set_parameters_callback(self._update_params)
        self._refresh_params()

        self.pos_per_deg = 27.7*105.26
        self.prev_pos    = 0.0
        self.dt          = 0.02
        self.create_timer(self.dt, self.timer_cb)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect(('127.0.0.1', 9000))
            self.get_logger().info("MotorServer bağlı")
        except Exception as e:
            self.get_logger().error(f"MotorServer bağlanamadı: {e}")

    def _refresh_params(self):
        gp = self.get_parameter
        self.steering_gain  = gp('steering_gain').value
        self.max_rate_rad   = math.radians(gp('max_steer_rate_deg').value)
        self.speed_deadzone = gp('speed_deadzone').value

    def _update_params(self, _):
        self._refresh_params()
        return rclpy.parameter.SetParametersResult(successful=True)

    def timer_cb(self):
        global vel_msg, steer_target

        # hedef açı (slew-rate filtresi)
        target = steer_target * self.steering_gain
        max_d  = self.max_rate_rad * self.dt
        smooth = np.clip(target - self.prev_pos, -max_d, +max_d) + self.prev_pos
        self.prev_pos = smooth

        # hız değeri
        v = vel_msg.linear.x
        speed_val = 0 if abs(v) < self.speed_deadzone else int(v*100)

        # dört teker – aynı komut
        servo_cmd = int(math.degrees(smooth) * self.pos_per_deg)
        for i in range(4):
            self._send({'command':'set_position', 'index':2*i,   'value':servo_cmd})
            self._send({'command':'set_velocity', 'index':2*i+1, 'value':speed_val})

    def _send(self, d):
        try:
            self.sock.sendall((json.dumps(d)+'\n').encode())
        except Exception as e:
            self.get_logger().error(f"Send failed: {e}")

    def stop_all(self): self._send({'command':'stop_all'})


def main(args=None):
    rclpy.init(args=args)
    joy  = JoySubscriber()
    four = FourWheelSteeringNode()

    exec_ = MultiThreadedExecutor()
    exec_.add_node(joy); exec_.add_node(four)

    try:
        exec_.spin()
    except KeyboardInterrupt:
        pass
    finally:
        four.stop_all()
        exec_.shutdown(); rclpy.shutdown()


if __name__ == '__main__':
    main()
