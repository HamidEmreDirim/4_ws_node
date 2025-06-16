#!/usr/bin/env python3
# four_ws_node.py – v13-fix-pivot
#
#  • ACK  : gerçek Ackermann
#  • CRAB : sabit ±90 ° – stick X yalnızca hız
#  • PIVOT: sabit ±45 ° / ±135 ° – stick X yalnızca dönme hızı
#  • STOP : her şey sıfır
#
#  – Hız ancak teker açıları ±4 ° toleransta iken verilir
#  – Açılar ±90 ° bandına katlanır; işaret uyumsuzsa teker hızının
#    yönü otomatik ters çevrilir
#  – Tek satır log:  [MOD] pos={ … } vel={ … }

import math, json, socket, numpy as np, rclpy
from rclpy.node      import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Joy

# ────────── PS5 düğmeleri
BTN_X, BTN_O, BTN_SQ, BTN_TRI = 0, 1, 2, 3

# ────────── modlar
MODE_ACK, MODE_PIVOT, MODE_CRAB, MODE_STOP = range(4)
MODE_NAME = {MODE_ACK: "ACK", MODE_PIVOT: "PIVOT",
             MODE_CRAB: "CRAB", MODE_STOP: "STOP"}
mode = MODE_ACK

# ────────── eksenler
AX_LY, AX_RSX = 1, 3        # LY = +ileri,  RSX = +sol / –sağ

# ────────── joystick değerleri
axis_fwd = axis_rsx = 0.0

# ─────────────────────────── JOYSTICK ───────────────────────────
class JoySub(Node):
    def __init__(self):
        super().__init__("joy_sub")
        self.declare_parameter("dz", 0.05)
        self.dz   = self.get_parameter("dz").value
        self.prev = [0]*16
        self.create_subscription(Joy, "joy", self.cb, 10)

    def _dz(self, v): return 0.0 if abs(v) < self.dz else v

    def cb(self, j: Joy):
        global mode, axis_fwd, axis_rsx
        b, a = j.buttons, j.axes

        if b[BTN_X]  and not self.prev[BTN_X]:   mode = MODE_ACK;   self.get_logger().info("↺ ACK")
        if b[BTN_O]  and not self.prev[BTN_O]:   mode = MODE_PIVOT; self.get_logger().info("↺ PIVOT")
        if b[BTN_SQ] and not self.prev[BTN_SQ]:  mode = MODE_CRAB;  self.get_logger().info("↺ CRAB")
        if b[BTN_TRI]and not self.prev[BTN_TRI]: mode = MODE_STOP;  self.get_logger().info("↺ STOP")
        self.prev = list(b)

        axis_fwd = -self._dz(a[AX_LY])      # +ileri
        axis_rsx =  self._dz(a[AX_RSX])     # +sol / –sağ

        if mode == MODE_STOP:
            axis_fwd = axis_rsx = 0.0

# ─────────────────────────── 4WS NODE ───────────────────────────
class FourWS(Node):
    def __init__(self):
        super().__init__("four_ws")

        self.declare_parameter("max_rate_deg", 40.0)
        self.declare_parameter("speed_dead",   0.02)
        self.dt         = 0.02
        self.max_rate   = math.radians(self.get_parameter("max_rate_deg").value)
        self.speed_dead = self.get_parameter("speed_dead").value

        # geometri
        self.L, self.W  = 1.48996, 1.245     # m
        self.pos_per_deg= 27.7*105.26         # servo cnt / °
        self.xy = np.array([[ self.L/2,  self.W/2], [ self.L/2,-self.W/2],
                            [-self.L/2,  self.W/2], [-self.L/2,-self.W/2]])

        # durum
        self.prev_pos     = np.zeros(4)
        self.prev_mode    = mode
        self.crab_angles  = np.zeros(4)

        # --- PIVOT açıları ---
        self.pivot_angles = np.array([ math.atan2(x, -y) for x, y in self.xy ])
        self.pivot_angles[0:2] *= -1      # << düzeltme: ön iki tekerin açısını tersine çevir

        self.create_timer(self.dt, self.loop)

        # TCP motor sunucusu
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try: self.sock.connect(("127.0.0.1", 9000)); self.sim = False
        except Exception as e:
            self.get_logger().warning(f"MotorServer yok, SIM: {e}"); self.sim = True

    # ────────── yardımcı: ±90 ° banda katla ──────────
    def _wrap90(self, angles, speeds):
        out_a = angles.copy(); out_v = speeds.copy()
        for i,phi in enumerate(out_a):
            if   phi >  math.pi/2:
                out_a[i] = phi - math.pi
                out_v[i]*=-1
            elif phi < -math.pi/2:
                out_a[i] = phi + math.pi
                out_v[i]*=-1
        return out_a, out_v

    # ────────── açı toleransı (wrap-safe) ──────────
    def _aligned(self, targ, tol_deg=4):
        diff = np.angle(np.exp(1j*(targ - self.prev_pos)))
        return np.max(np.abs(diff)) < math.radians(tol_deg)

    # ────────── motor paket gönder ──────────
    def _send_all(self, pos, vel):
        for i in range(4):
            servo = int(round(math.degrees(pos[i])*self.pos_per_deg))
            speed = 0 if abs(vel[i]) < self.speed_dead else int(round(vel[i]*100))
            self._send({'command':'set_position','index':2*i,   'value':servo})
            self._send({'command':'set_velocity','index':2*i+1, 'value':speed})

    def _send(self, pkt):
        if self.sim: self.get_logger().debug(f"SIM ▶ {pkt}")
        else:
            try: self.sock.sendall((json.dumps(pkt)+'\n').encode())
            except Exception as e: self.get_logger().error(f"TCP FAIL {e}")

    def _log(self, pos, vel):
        degs = ", ".join(f"{math.degrees(p):6.1f}" for p in pos)
        vels = ", ".join(f"{v:6.2f}"               for v in vel)
        self.get_logger().info(f"[{MODE_NAME[mode]}] pos={{ {degs} }} vel={{ {vels} }}")

    # ────────── ana döngü ──────────
    def loop(self):
        global mode, axis_fwd, axis_rsx

        # moda yeni girildiyse sabit açıları ayarla
        if mode != self.prev_mode and mode == MODE_CRAB:
            self.crab_angles[:] = (1 if axis_rsx >= 0 else -1) * math.pi/2
        self.prev_mode = mode

        targ_raw = np.zeros(4); vel_raw = np.zeros(4)

        # ─── ACK ───
        # ─── ACK ───
        if mode == MODE_ACK:
            steer_c = axis_rsx * math.radians(35)        # orta noktadaki direksiyon açısı
            v_c     = axis_fwd * 2.0                     # aracın ileri hızı (merkez)
            targ_raw[:] = 0.0
            vel_raw[:]  = 0.0

            # Direksiyon sıfıra çok yakınsa klasik düz-sürüş
            if abs(steer_c) < 1e-3:
                vel_raw[:] = v_c

            else:
                R = self.L / math.tan(steer_c)           # ICC yarıçapı (işareti korur)
                ω = v_c / R                              # açısal hız  rad s-¹

                for i, (x, y) in enumerate(self.xy):
                    # Teker ekseni, ICC’ye çizilen yarıçapın normaline paralel olmalı
                    targ_raw[i] = math.atan2(x, R - y)

                    # O tekerin yol yarıçapı
                    r_i = math.hypot(x, R - y)

                    # Hız = ω · r_i   (işaret v_c ile aynı kalır)
                    vel_raw[i] = ω * r_i

            # ±90 ° bandına katla → yön tersse hız işareti de tersine döner
            targ, vel = self._wrap90(targ_raw, vel_raw)


        # ─── CRAB ───
        elif mode == MODE_CRAB:
            targ_raw = self.crab_angles.copy()
            targ, vel = self._wrap90(targ_raw, vel_raw)
            if self._aligned(targ):
                vel[:] = abs(axis_rsx)*2.0 * math.copysign(1, axis_rsx)

        # ─── PIVOT ───
        elif mode == MODE_PIVOT:
            targ_raw = self.pivot_angles.copy()
            targ, vel = self._wrap90(targ_raw, vel_raw)
            if self._aligned(targ):
                w = axis_rsx * 1.2          # rad s⁻¹
                for i,(x,y) in enumerate(self.xy):
                    v = math.hypot(-w*y, w*x)
                    vel[i] = v * math.copysign(1, w)

        # ─── STOP ───
        else:
            targ, vel = targ_raw, vel_raw   # hepsi 0

        # Slew-rate
        max_d  = self.max_rate * self.dt
        smooth = np.clip(targ - self.prev_pos, -max_d, +max_d) + self.prev_pos
        self.prev_pos = smooth

        self._send_all(smooth, vel)
        self._log(smooth, vel)

    def stop_all(self): self._send({'command':'stop_all'})

# ─────────────────────────── main ───────────────────────────
def main(args=None):
    rclpy.init(args=args)
    joy, four = JoySub(), FourWS()
    exe = MultiThreadedExecutor(); exe.add_node(joy); exe.add_node(four)
    try: exe.spin()
    except KeyboardInterrupt: pass
    finally:
        four.stop_all(); exe.shutdown(); rclpy.shutdown()

if __name__ == "__main__":
    main()
